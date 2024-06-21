#include "ranging-engine.h"

#if CONFIG_DWT_MTM_ON_DEVICE_PROCESSING
static uint8_t g_ranging_ids[MAX_ROUND_LENGTH];
static struct freq_offset g_offsets[2*MAX_ROUND_LENGTH*MAX_ROUND_LENGTH]; // in the worst case we will get a quadratic amount of g_measurements + for each node pair a passively extracted frequency offset
static struct measurement g_measurements[MAX_ROUND_LENGTH * MAX_ROUND_LENGTH];
static struct twr_template g_templates[MAX_ROUND_LENGTH * MAX_ROUND_LENGTH];

static inline uint64_t correct_overflow(uint64_t end_ts, uint64_t start_ts) {
    if (end_ts < start_ts) {
        end_ts += 0xFFFFFFFFFF;
    }

    return end_ts - start_ts;
}

int valid_fp_index_p(uint16_t fp_index) {
    int32_t diff = (int32_t) 750 - ((int32_t) fp_index >> 6);
    return ( diff <  FP_INDEX_VALIDY_RANGE )
        && ( diff > -FP_INDEX_VALIDY_RANGE );
}

int32_t
compute_prop_time(int32_t initiator_roundtrip, int32_t initiator_reply,
    int32_t replier_roundtrip, int32_t replier_reply) {

    return (int32_t)(( ((int64_t) initiator_roundtrip * replier_roundtrip)
            - ((int64_t) initiator_reply * replier_reply))
        /
        ((int64_t) initiator_roundtrip
            +  replier_roundtrip
            +  initiator_reply
            +  replier_reply));
}


float time_to_dist(float tof) {
    return (float)tof * SPEED_OF_LIGHT_M_PER_UWB_TU;
}

float calculate_propagation_time_alternative(uint64_t tx_init, uint64_t rx_init, uint64_t tx_resp,
					     uint64_t rx_resp, uint64_t tx_final, uint64_t rx_final)
{
    static float relative_drift_offset;
    static int64_t responder_duration, initiator_duration;
    static int64_t round_duration_a, delay_duration_b;
    /* static float drift_offset_int, two_tof_int; */
    static int64_t drift_offset_int, two_tof_int;

    initiator_duration   = correct_overflow(tx_final, tx_init);
    responder_duration = correct_overflow(rx_final, rx_init);

    // factor determining whether B's clock runs faster or slower measured from the perspective of our clock
    // a positive factor here means that the clock runs faster than ours
    /* relative_drift_offset = (float)((int64_t)initiator_duration-(int64_t)responder_duration) / (float)(responder_duration); */
    relative_drift_offset = (float)((int64_t)initiator_duration-(int64_t)responder_duration) / (float)(responder_duration);


    round_duration_a = correct_overflow(rx_resp, tx_init);
    delay_duration_b = correct_overflow(tx_resp, rx_init);

    /* drift_offset_int = -relative_drift_offset * (float) delay_duration_b; */
    drift_offset_int = round(-relative_drift_offset * (float) delay_duration_b);

    /* two_tof_int = (float)round_duration_a - (float)delay_duration_b + drift_offset_int; */
    two_tof_int = (int64_t)round_duration_a - (int64_t)delay_duration_b + drift_offset_int;

    return ((float) two_tof_int) * 0.5;
}

/* Assumptions: A node may not change its position in the ranging round in subsequent phases, i.e.,
   if it transmitted in slot N in round K it should also transmit in slot N in round K+1.

   returns amount of calculated twr distances for given initiatior id.
 */
int fill_twr_template_from_frames(const struct dwt_ranging_frame_info *frame_infos,
    uint8_t initiator_id, int round_length,
    int phases, struct twr_template *twr_templates,
    int with_reject
    )
{
    int responder_count = 0;
    uint64_t tx_init = UINT64_MAX, tx_final = UINT64_MAX;

    // these we will reference more often
    const struct dwt_ranging_frame_buffer *initiator_phase_1_frame = NULL, *initiator_phase_2_frame = NULL;
    uint8_t initiation_node_slot_offset = UINT8_MAX;

    if (round_length <= 1) {
	LOG_ERR("Invalid round length");
        return -EINVAL;
    }

    // first sweep find the initiation frame and finalization frame of the initiating node
    for(int i = round_length; i < phases*round_length; i++) {
        const struct dwt_ranging_frame_info *frame_info = &frame_infos[i];

        if(frame_info->frame != NULL) {
            const struct dwt_ranging_frame_buffer *frame = frame_info->frame;

            if(frame->ranging_id == initiator_id) {
                if(!initiator_phase_1_frame) {
                    initiator_phase_1_frame = frame;
                    initiation_node_slot_offset = i - round_length;

                    // grab tx_init from buffer
		    tx_init = from_packed_dwt_ts(initiator_phase_1_frame->tx_ts);
                } else {
                    initiator_phase_2_frame = frame;
                    // grab tx_final from buffer
		    tx_final = from_packed_dwt_ts(initiator_phase_2_frame->tx_ts);
                    break;
                }
            }
        }
    }

    // swipe once through frames to capture all ranging ids and associate them with a position in distances
    for(int i = 0; i < round_length; i++) {
        const struct dwt_ranging_frame_info *frame_info = &frame_infos[i];

        if(frame_info->frame != NULL) {
            const struct dwt_ranging_frame_buffer *frame = frame_info->frame;

            if(frame->ranging_id != initiator_id) {
                twr_templates[responder_count].ranging_initiator_id = initiator_id;
                twr_templates[responder_count].ranging_responder_id =
                    frame->ranging_id;

                // init values to UINT64_MAX
                twr_templates[responder_count].tx_init = tx_init;
                twr_templates[responder_count].tx_final = tx_final;

                twr_templates[responder_count].rx_init = UINT64_MAX;
                twr_templates[responder_count].tx_resp = UINT64_MAX;
                twr_templates[responder_count].rx_resp = UINT64_MAX;
                twr_templates[responder_count].rx_final = UINT64_MAX;

                twr_templates[responder_count].local_rx_init = UINT64_MAX;
                twr_templates[responder_count].local_rx_resp = UINT64_MAX;
                twr_templates[responder_count].local_rx_final = UINT64_MAX;

                responder_count++;
            }
        }
    }

/* next we retrieve all timestamps that are specific to other nodes and are not shared */
/* between all calculations */

    // first find rx_init, tx_resp and rx_resp
    for(int i = 0; i < round_length; i++) {
        if(i != initiation_node_slot_offset) {
            uint64_t rx_init = UINT64_MAX, tx_resp = UINT64_MAX, rx_resp = UINT64_MAX, rx_final = UINT64_MAX;
            uint64_t local_rx_init = UINT64_MAX, local_rx_resp = UINT64_MAX, local_rx_final = UINT64_MAX;


            //  TODO the naming here is not good yet, the frames do not respond to the respective
            // names for the protocol frames i.e., the initiation frame is originally send in phase
            // 1, depending on the slot order the response frame might either be part of phase 1 or
            // the subsequent phase. i.e., we should probably find a better name here.
	    const struct dwt_ranging_frame_info *responder_phase_1_frame_info =
		    &frame_infos[round_length + i];
	    const struct dwt_ranging_frame_buffer *responder_phase_1_frame =
		    responder_phase_1_frame_info->frame;

	    const struct dwt_ranging_frame_info *responder_phase_2_frame_info = NULL;
	    const struct dwt_ranging_frame_buffer *responder_phase_2_frame = NULL;

	    if (responder_phase_1_frame_info->frame == NULL) {
                continue;
	    }

            if(phases > 2) {
		responder_phase_2_frame_info = &frame_infos[2 * round_length + i];
                responder_phase_2_frame = responder_phase_2_frame_info->frame;
	    }

	    uint8_t responder_id = responder_phase_1_frame->ranging_id;

	    // extract info from phase_1 frame
	    if (responder_phase_1_frame_info->frame != NULL) {
                if(with_reject && !valid_fp_index_p(responder_phase_1_frame_info->fp_index) ) {
                    continue;
		}

                // in any case the received phase_1 frame will contain rx_init
                for(int k = 0; k < responder_phase_1_frame->rx_ts_count; k++) {
                    if(responder_phase_1_frame->rx_ts[k].ranging_id == initiator_id) {
                        rx_init = from_packed_dwt_ts(responder_phase_1_frame->rx_ts[k].ts);
                        break;
                    }
		}

		if (i > initiation_node_slot_offset) {
                    tx_resp = from_packed_dwt_ts(responder_phase_1_frame->tx_ts);

                    // in our own initiation frame we will find rx_resp
                    for(int k = 0; k < initiator_phase_1_frame->rx_ts_count; k++) {
                        if(initiator_phase_1_frame->rx_ts[k].ranging_id == responder_id) {
                            rx_resp = from_packed_dwt_ts(initiator_phase_1_frame->rx_ts[k].ts);
                            break;
                        }
                    }
                }
	    }

	    if (responder_phase_2_frame_info != NULL && responder_phase_2_frame_info->frame != NULL) {
                if(with_reject && !valid_fp_index_p(responder_phase_2_frame_info->fp_index) ) {
                    continue;
		}

                // in case of contention based access this might happen
                if(responder_phase_1_frame->ranging_id != responder_phase_2_frame->ranging_id) {
                    LOG_ERR("Initiation and finalization frame do not match %u != %u", responder_phase_1_frame->ranging_id, responder_phase_2_frame->ranging_id);
                    continue;
                }

                // in any case we also find the rx_final timestamp in the received phase_2 frame
                for(int k = 0; k < responder_phase_2_frame->rx_ts_count; k++) {
                    if(responder_phase_2_frame->rx_ts[k].ranging_id == initiator_id) {
                        rx_final = from_packed_dwt_ts(responder_phase_2_frame->rx_ts[k].ts);
                        break;
                    }
		}

                if(i < initiation_node_slot_offset) {
                    tx_resp = from_packed_dwt_ts(responder_phase_2_frame->tx_ts);

                    // in our own finalization frame we will find rx_resp
                    for(int k = 0; k < initiator_phase_2_frame->rx_ts_count; k++) {
                        if(initiator_phase_2_frame->rx_ts[k].ranging_id == responder_id) {
                            rx_resp = from_packed_dwt_ts(initiator_phase_2_frame->rx_ts[k].ts);
                            break;
                        }
                    }
                }
            }

            // --- Passive extraction ---
            if(initiator_id != node_ranging_id) {
                // Find local timestamps, for this we now directly look at the frame reception
                // timestamps in the info frames (Above we found the timestamps from phase N in
                // Phase N+1) We read this directly from the info frames, since in case of a
                // fully passive receiver, we won't have any transmission frames to get retrieve
                // that data from that node
                const struct dwt_ranging_frame_info *local_init_info         = &frame_infos[initiation_node_slot_offset];
                const struct dwt_ranging_frame_info *local_response_info     = NULL;
		const struct dwt_ranging_frame_info *local_finalization_info = NULL;

		if (phases > 1) {
                    local_response_info = &frame_infos[i > initiation_node_slot_offset ? i : round_length + i];
		}

		if (phases > 2) {
                    local_finalization_info = &frame_infos[round_length + initiation_node_slot_offset];
                }

                if(local_init_info->frame != NULL && local_init_info->frame->ranging_id == initiator_id) {
                    local_rx_init = local_init_info->timestamp;
                }

                if(local_response_info->frame != NULL && local_response_info->frame->ranging_id == responder_id) {
                    local_rx_resp = local_response_info->timestamp;
                }

                if(local_finalization_info->frame != NULL && local_finalization_info->frame->ranging_id == initiator_id) {
                    local_rx_final = local_finalization_info->timestamp;
                }
            }

            // store in twr_templates
            for(int k = 0; k < responder_count; k++) {
                if(twr_templates[k].ranging_responder_id == responder_id) {
                    twr_templates[k].rx_init = rx_init;
                    twr_templates[k].tx_resp = tx_resp;
                    twr_templates[k].rx_resp = rx_resp;
                    twr_templates[k].rx_final = rx_final;

                    twr_templates[k].local_rx_init  = local_rx_init;
                    twr_templates[k].local_rx_resp  = local_rx_resp;
                    twr_templates[k].local_rx_final = local_rx_final;
                    break;
                }
            }
        }
    }

    return responder_count;
}



int have_freq_offset(struct freq_offset *offsets, int offsets_length, uint8_t from_id, uint8_t to_id) {
    for(int i = 0; i < offsets_length; i++) {
        if(offsets[i].from_id == from_id && offsets[i].to_id == to_id) {
            return 1;
        }
    }

    return 0;
}

int frequencies_from_cfos(const struct dwt_ranging_frame_info *frame_infos, int round_length,
			  int phases, struct freq_offset *offsets)
{
    int offset_cnt = 0;

    for(int i = 0; i < round_length; i++) {
        const struct dwt_ranging_frame_info *frame_info = &frame_infos[i];

        if(frame_info->frame != NULL) {
	    struct freq_offset *curr_off = &offsets[offset_cnt];

            curr_off->from_id = frame_info->frame->ranging_id;
	    curr_off->to_id = node_ranging_id;

	    if (have_freq_offset(offsets, offset_cnt, curr_off->from_id, curr_off->to_id)) {
                continue;
            }

	    curr_off->offset = -(frame_info->cfo_ppm / 1.0e6);

            offset_cnt++;
        }
    }

    return offset_cnt;
}


int extract_round_templates(const struct dwt_ranging_frame_info *frame_infos,
    const struct mtm_ranging_config *conf,
    enum extraction_mode mode)
{
    int ranging_id_count = get_ranging_ids(frame_infos, g_ranging_ids, conf->round_length);

    int extracted_templates = 0;
    if(mode == EXTRACT_LOCAL_ONLY) {
        int template_cnt = fill_twr_template_from_frames(frame_infos, node_ranging_id, conf->round_length, conf->phases, g_templates, EXTRACT_WITH_REJECT);
        extracted_templates += template_cnt;
    } else if (mode == EXTRACT_LOCAL_BIDIRECTIONAL) {
        LOG_ERR("Not implemented yet");
        return -1;
    } else {
        for(int i = 0; i < ranging_id_count; i++) {
            int template_cnt = fill_twr_template_from_frames(
                frame_infos, g_ranging_ids[i], conf->round_length, conf->phases,
                g_templates + extracted_templates, EXTRACT_WITH_REJECT);

            if (template_cnt > 0) {
                // TODO remove again
                extracted_templates += template_cnt;
            }
        }
    }

    return extracted_templates;
}

int calculate_frequency_offsets(const struct twr_template *twr_templates, int template_count, struct freq_offset *offsets) {
    int offset_cnt = 0;

    for(int i = 0; i < template_count; i++) {
        const struct twr_template *curr_tmpl = &twr_templates[i];
        struct freq_offset *curr_off = &offsets[offset_cnt];
        uint8_t initiator_id = curr_tmpl->ranging_initiator_id, responder_id = curr_tmpl->ranging_responder_id;


        if(curr_tmpl->tx_init == UINT64_MAX || curr_tmpl->rx_init == UINT64_MAX || curr_tmpl->tx_final == UINT64_MAX || curr_tmpl->rx_final == UINT64_MAX) {
            continue;
        }

        uint64_t initiator_duration = correct_overflow(curr_tmpl->tx_final,  curr_tmpl->tx_init);
        uint64_t responder_duration = correct_overflow(curr_tmpl->rx_final, curr_tmpl->rx_init);

        curr_off->from_id = responder_id;
	curr_off->to_id = initiator_id;

        if (!have_freq_offset(offsets, offset_cnt, curr_off->to_id, curr_off->from_id)) {
	    curr_off->offset = (float)((int64_t)initiator_duration - (int64_t)responder_duration) /
	        	       (float)(responder_duration);
            offset_cnt++;
        }


        // TODO maybe we can do this code somewhat cleaner, for instance i am still not sure if we
        // should do this independent of the detection of a completed twr round (for now this code
        // assumes that we only want to know the offset for the later calculation of a tdoa
        // measurement based on a overheard TWR exchange, but maybe we also have use for these
        // values independent of the existance of a successfull round?
        if(initiator_id != node_ranging_id && responder_id != node_ranging_id) {
            curr_off = &offsets[offset_cnt];

            if(curr_tmpl->local_rx_final == UINT64_MAX || curr_tmpl->local_rx_init == UINT64_MAX) {
		continue;
            }

            // we are the other in this case
            responder_duration = correct_overflow(curr_tmpl->local_rx_final, curr_tmpl->local_rx_init);

            curr_off->from_id = initiator_id;
            curr_off->to_id = node_ranging_id;

	    if (!have_freq_offset(offsets, offset_cnt, curr_off->to_id, curr_off->from_id)) {
                curr_off->offset = (float)((int64_t)responder_duration - (int64_t)initiator_duration) /
                    (float)(initiator_duration);
                offset_cnt++;
            }
        }
    }

    return offset_cnt;
}

/* calculate twr and tdoa estimataes from extracted templates.  In case that offsets is null do not
   compensate for frequency deviation
*/
int calculate_measurements(
    const struct twr_template *twr_templates,
    int template_count,
    const struct freq_offset *offsets,
    int frequency_count,
    struct measurement *measurements) {
    int meas_cnt = 0;

    // iterate over all g_templates
    for (int i = 0; i < template_count; i++) {
	const struct twr_template *curr_tmpl = &twr_templates[i];

	if (curr_tmpl->tx_init == UINT64_MAX || curr_tmpl->rx_init == UINT64_MAX ||
	    curr_tmpl->tx_resp == UINT64_MAX || curr_tmpl->rx_resp == UINT64_MAX) {
	    continue;
        }

        uint64_t round_dur_initiator =
            correct_overflow(curr_tmpl->rx_resp, curr_tmpl->tx_init);
        uint64_t delay_dur_responder =
            correct_overflow(curr_tmpl->tx_resp,  curr_tmpl->rx_init);
	int64_t drift_offset_int = 0;

        if(offsets != NULL) {
            const struct freq_offset *responder_offset_to_initiator = NULL;
            for (int j = 0; j < frequency_count; j++) {
		if (offsets[j].from_id == curr_tmpl->ranging_responder_id &&
		    offsets[j].to_id == curr_tmpl->ranging_initiator_id) {
                    responder_offset_to_initiator = &offsets[j];
                    break;
                }
            }

            if (responder_offset_to_initiator != NULL) {
                drift_offset_int = round(-responder_offset_to_initiator->offset * (float)delay_dur_responder);
            }
	}

        measurements[meas_cnt].ranging_initiator_id = curr_tmpl->ranging_initiator_id;
        measurements[meas_cnt].ranging_responder_id = curr_tmpl->ranging_responder_id;
        measurements[meas_cnt].tof = (float)((int64_t)round_dur_initiator - (int64_t)delay_dur_responder + drift_offset_int) * 0.5;
	measurements[meas_cnt].tdoa = NAN;

        // calculate passive tdoa value
	if (curr_tmpl->ranging_initiator_id != node_ranging_id &&
	    curr_tmpl->ranging_responder_id != node_ranging_id) {

            if (curr_tmpl->local_rx_init == UINT64_MAX || curr_tmpl->local_rx_resp == UINT64_MAX) {
                continue;
	    }

	    uint64_t tx_reception_diff =
		    correct_overflow(curr_tmpl->local_rx_resp, curr_tmpl->local_rx_init);
	    int64_t passive_to_initiator_drift_offset_int = 0;

	    if (offsets != NULL) {
		const struct freq_offset *passive_offset_to_initiator = NULL;
		const struct freq_offset *passive_offset_to_responder = NULL;

                for (int j = 0; j < frequency_count; j++) {
		    if (offsets[j].from_id == curr_tmpl->ranging_initiator_id &&
			offsets[j].to_id == node_ranging_id) {
                        passive_offset_to_initiator = &offsets[j];
                        break;
                    }
		}

                for (int j = 0; j < frequency_count; j++) {
		    if (offsets[j].from_id == curr_tmpl->ranging_responder_id &&
			offsets[j].to_id == node_ranging_id) {
                        passive_offset_to_responder = &offsets[j];
                        break;
                    }
		}

                if(passive_offset_to_initiator == NULL || passive_offset_to_responder == NULL) {
		    continue;
                }

		passive_to_initiator_drift_offset_int =
			round(passive_offset_to_initiator->offset * (float)round_dur_initiator +
			      passive_offset_to_responder->offset * (float)delay_dur_responder);
	    }

            measurements[meas_cnt].tdoa =
                ((float)((int64_t)round_dur_initiator + (int64_t)delay_dur_responder +
                    passive_to_initiator_drift_offset_int -
                    (int64_t)(2 * tx_reception_diff))) * 0.5;

	    /* measurements[meas_cnt].tdoa = (int64_t)round_dur_initiator - */
	    /*     			  measurements[meas_cnt].tof - tx_reception_diff + */
	    /*     			  passive_to_initiator_drift_offset_int; */
        }

        meas_cnt++;
    }

    return meas_cnt;
}
#endif
