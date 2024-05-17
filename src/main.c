#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include <zephyr/devicetree.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/drivers/ieee802154/dw1000.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/hash_function.h>
#include <stdio.h>

#include <zephyr/sys/timeutil.h>



#include "nodes.h"
#include "log.h"
#include "history.h"

LOG_MODULE_REGISTER(main);

#define USE_GPIO_DEBUG 1
#if USE_GPIO_DEBUG

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>

#define DEB0_NODE DT_ALIAS(deb0)
#define DEB1_NODE DT_ALIAS(deb1)

static const struct gpio_dt_spec deb0 = GPIO_DT_SPEC_GET(DEB0_NODE, gpios);;
static const struct gpio_dt_spec deb1 = GPIO_DT_SPEC_GET(DEB1_NODE, gpios);;

static void setup_debug_gpios() {
	gpio_pin_configure_dt(&deb0, GPIO_OUTPUT | GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&deb1, GPIO_OUTPUT | GPIO_OUTPUT_ACTIVE);
}

#define TOGGLE_DEBUG_GPIO(GPIO) do { \
    gpio_pin_toggle_dt(&deb##GPIO); \
} while(0)

#define SET_GPIO_HIGH(GPIO) do { \
		gpio_pin_set_dt(&deb##GPIO, 1);	\
} while(0)

#define SET_GPIO_LOW(GPIO) do { \
		gpio_pin_set_dt(&deb##GPIO, 0);	\
} while(0)
#else
#define TOGGLE_DEBUG_GPIO(GPIO)
#define TOGGLE_DEBUG_GPIOS()
#endif

#if LOG_FLUSH_DIRECTLY
#define log_out uart_out
#else
#define log_out log_out
#endif


#define UART0_NODE DT_NODELABEL(uart0)
/* PINCTRL_DT_DEFINE(UART0_NODE); */
static const struct device *const uart_dev = DEVICE_DT_GET(UART0_NODE);


/* ieee802.15.4 device */
static struct ieee802154_radio_api *radio_api;
static const struct device *ieee802154_dev;

static inline uint64_t correct_overflow(uint64_t end_ts, uint64_t start_ts){
    if (end_ts < start_ts) {
        end_ts += 0xFFFFFFFFFF;
    }

    return end_ts - start_ts;
}


#define DWT_TS_MASK 0xFFFFFFFFFF
#define DWT_TS_TO_US(X) (((X)*15650)/1000000000)
#define UUS_TO_DWT_TS(X) (((uint64_t)X)*(uint64_t)65536)

#define EXPERIMENT_NAME "driver mtm ranging"

static uint16_t node_ranging_id;

enum network_event {
    RANGING_NONE,
    RANGING_CHECK_TIMESYNC_GPIO,
    RANGING_GLOSSY_SLOT,
    RANGING_MTM_SLOT,
    RANGING_MTM_CCA_SLOT,
    RANGING_MTM_ALOHA_SLOT,
    RANGING_MTM_HASHED_SLOT
};

static enum network_event next_event = RANGING_NONE;
static uint64_t next_event_time;
static uint32_t timesync_skew_update_count = 0;

#define UART_TX_RESULTS 0
#define UART_POLLING    0

#define SPEED_OF_LIGHT_M_PER_S 299702547.236
#define SPEED_OF_LIGHT_M_PER_UWB_TU                                                                \
	((SPEED_OF_LIGHT_M_PER_S * 1.0E-15) * 15650.0) // around 0.00469175196

#define DIST_OUTPUT_MM 1000
#define DIST_OUTPUT_CM 100
#define DIST_OUTPUT_M  1
#define DIST_OUTPUT_UNIT DIST_OUTPUT_CM


#define FILTER_MEASUREMENTS_THRESHOLD -80
#define MAX_ROUND_LENGTH 15
#define PASSIVE_TDOA 0
#define EXTRACT_ALL_RANGES 0
#define FP_INDEX_VALIDY_RANGE 20
#define EXTRACT_WITH_REJECT           0 // TODO we do this in the ranging directly now

/* #define SLOT_LENGTH_TIMESTAMP_MS 30 */
#define SLOT_LENGTH_TIMESTAMP_MS 200 // for debugging do 500ms slots
#define SYS_TICK_ROUND_LENGTH    ((CONFIG_SYS_CLOCK_TICKS_PER_SEC * SLOT_LENGTH_TIMESTAMP_MS) / 1000)

#define WITHOUT_INITIATION_FRAME 0
#define WITH_INITIATION_FRAME 1 // TODO maybe create enumerations

// CCA related
#define PAC_SIZE 8
#define PREAMBLE_SYMBOLS 128  //TODO  + 120 // guard times
#define MAX_PAC_TIMEOUT  (PREAMBLE_SYMBOLS / PAC_SIZE)

#define MTM_HASHED_SLOTS 15
#define MTM_ALOHA_SLOTS  15

K_SEM_DEFINE(uart_transfer_finished, 1, 1);


static void uart_poll_serial_output_msg(char* msg) {
    while (*msg != '\0') {
        uart_poll_out(uart_dev, *msg);
        msg++;
    }
}

void uart_finished_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
    if(evt->type == UART_TX_DONE) {
        /* LOG_WRN("UART TX done"); */
    } else if(evt->type == UART_TX_ABORTED) {
        LOG_WRN("UART TX aborted");
    }

    k_sem_give(&uart_transfer_finished);
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



/* TODO this is not that nice yet, maybe we should just switch over to functions which calculate the delay duration, round duration etc.
 * for given ids, i.e., instead of preprocessing the timestamps into a fixed template we have functions which calculate the relevant
 * durations directly from the set of received frames.
 */

/* all these data structures are expected to be only be relevant for the duration of a ranging round
 * therefore the additional space we have to spent for associating each item with the pair of nodes
 * is assumed to be negligible.
 */
struct twr_template
{
    uint8_t ranging_initiator_id, ranging_responder_id;
    uint64_t tx_init, rx_init, tx_resp, rx_resp, tx_final, rx_final;

    // these are only relevant if ranging_initiator_id and ranging_responder_id are both not our own ranging id
    uint64_t local_rx_init, local_rx_resp, local_rx_final;
};


// frequency offsets are determined through two transmissions by one node and the subsequent two
// receptions at another node
struct freq_offset
{
    uint8_t transmitter_id, receiver_id;
    float offset;
};

struct measurement
{
    uint8_t ranging_initiator_id, ranging_responder_id;
    float tof, tdoa;
    float quality; // currently unused but useful in the future
};

float calculate_propagation_time_alternative(uint64_t tx_init, uint64_t rx_init, uint64_t tx_resp, uint64_t rx_resp, uint64_t tx_final, uint64_t rx_final) {
    static float relative_drift_offset;
    static int64_t other_duration, own_duration;
    static int64_t round_duration_a, delay_duration_b;
    /* static float drift_offset_int, two_tof_int; */
    static int64_t drift_offset_int, two_tof_int;

    own_duration   = correct_overflow(tx_final, tx_init);
    other_duration = correct_overflow(rx_final, rx_init);

    // factor determining whether B's clock runs faster or slower measured from the perspective of our clock
    // a positive factor here means that the clock runs faster than ours
    /* relative_drift_offset = (float)((int64_t)own_duration-(int64_t)other_duration) / (float)(other_duration); */
    relative_drift_offset = (float)((int64_t)own_duration-(int64_t)other_duration) / (float)(other_duration);


    round_duration_a = correct_overflow(rx_resp, tx_init);
    delay_duration_b = correct_overflow(tx_resp, rx_init);

    /* drift_offset_int = -relative_drift_offset * (float) delay_duration_b; */
    drift_offset_int = round(-relative_drift_offset * (float) delay_duration_b);

    /* two_tof_int = (float)round_duration_a - (float)delay_duration_b + drift_offset_int; */
    two_tof_int = (int64_t)round_duration_a - (int64_t)delay_duration_b + drift_offset_int;

    return ((float) two_tof_int) * 0.5;
}


static char serial_buf[4096]; // when TDOA is used has to be quite large, we pretty much use here most spare memory we have left over. Could off course easily be optimized


// fills ids with ranging ids found in the first round of the whole ranging process, return the amount of ids found.
int get_ranging_ids(const struct dwt_ranging_frame_info *frames_info, uint8_t *ids, int round_length) {
    int id_count = 0;
    for(int i = 0; i < round_length; i++) {
        const struct dwt_ranging_frame_info *frame_info = &frames_info[i];

        if(frame_info->frame != NULL) {
            ids[id_count] = frame_info->frame->ranging_id;
            id_count++;
        }
    }

    return id_count;
}


int valid_fp_index_p(uint16_t fp_index) {
    int32_t diff = (int32_t) 750 - ((int32_t) fp_index >> 6);
    return ( diff <  FP_INDEX_VALIDY_RANGE )
        && ( diff > -FP_INDEX_VALIDY_RANGE );
}


/* Assumptions: A node may not change its position in the ranging round in subsequent phases, i.e.,
   if it transmitted in slot N in round K it should also transmit in slot N in round K+1.

   returns amount of calculated twr distances for given initiatior id.
 */
int fill_twr_template_from_frames(const struct dwt_ranging_frame_info *frame_infos, uint8_t initiator_id, int round_length, int repetitions, struct twr_template *twr_templates, int with_reject)
{
    int responder_count = 0;
    uint64_t tx_init = UINT64_MAX, tx_final = UINT64_MAX;

    // these we will reference more often
    const struct dwt_ranging_frame_buffer *initiator_initiation_frame = NULL, *initiator_finalization_frame = NULL;
    uint8_t initiation_node_slot_offset = UINT8_MAX;

    // first sweep find the initiation frame and finalization frame of the initiating node
    for(int i = round_length; i < repetitions*round_length; i++) {
        const struct dwt_ranging_frame_info *frame_info = &frame_infos[i];

        if(frame_info->frame != NULL) {
            const struct dwt_ranging_frame_buffer *frame = frame_info->frame;

            if(frame->ranging_id == initiator_id) {
                if(!initiator_initiation_frame) {
                    initiator_initiation_frame = frame;
                    initiation_node_slot_offset = i - round_length;

                    // grab tx_init from buffer
		    tx_init = from_packed_dwt_ts(initiator_initiation_frame->tx_ts);
                } else {
                    initiator_finalization_frame = frame;
                    // grab tx_final from buffer
		    tx_final = from_packed_dwt_ts(initiator_finalization_frame->tx_ts);
                    break;
                }
            }
        }
    }

    // -- small sanity check
    if(!initiator_initiation_frame || !initiator_finalization_frame) {
        /* LOG_ERR("Could not find initiation or finalization buffer"); */
        return -ENODATA;
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
            const struct dwt_ranging_frame_info *responder_initiation_frame_info   = &frame_infos[  round_length + i];
            const struct dwt_ranging_frame_info *responder_finalization_frame_info = &frame_infos[2*round_length + i];

            if( responder_initiation_frame_info->frame != NULL && responder_finalization_frame_info->frame != NULL ) {
                const struct dwt_ranging_frame_buffer *received_initiation_frame = responder_initiation_frame_info->frame;
                const struct dwt_ranging_frame_buffer *received_finalization_frame = responder_finalization_frame_info->frame;
                uint8_t responder_id = received_initiation_frame->ranging_id;

                if(with_reject) {
                    if(!valid_fp_index_p(responder_initiation_frame_info->fp_index) || !valid_fp_index_p(responder_finalization_frame_info->fp_index)) {
                        continue;
                    }
                }

                if(received_initiation_frame->ranging_id != received_finalization_frame->ranging_id) {
                    LOG_ERR("Initiation and finalization frame do not match %u != %u", received_initiation_frame->ranging_id, received_finalization_frame->ranging_id);
                    return -EINVAL;
                }

                // in any case the received initiation frame will contain rx_init
                for(int k = 0; k < received_initiation_frame->rx_ts_count; k++) {
                    if(received_initiation_frame->rx_ts[k].ranging_id == initiator_id) {
                        rx_init = from_packed_dwt_ts(received_initiation_frame->rx_ts[k].ts);
                        break;
                    }
                }

                // in any case we also find the rx_final timestamp in the received finalization frame
                for(int k = 0; k < received_finalization_frame->rx_ts_count; k++) {
                    if(received_finalization_frame->rx_ts[k].ranging_id == initiator_id) {
                        rx_final = from_packed_dwt_ts(received_finalization_frame->rx_ts[k].ts);
                        break;
                    }
                }

                if(i < initiation_node_slot_offset) {
                    tx_resp = from_packed_dwt_ts(received_finalization_frame->tx_ts);

                    // in our own finalization frame we will find rx_resp
                    for(int k = 0; k < initiator_finalization_frame->rx_ts_count; k++) {
                        if(initiator_finalization_frame->rx_ts[k].ranging_id == responder_id) {
                            rx_resp = from_packed_dwt_ts(initiator_finalization_frame->rx_ts[k].ts);
                            break;
                        }
                    }
                } else {
                    tx_resp = from_packed_dwt_ts(received_initiation_frame->tx_ts);

                    // in our own initiation frame we will find rx_resp
                    for(int k = 0; k < initiator_initiation_frame->rx_ts_count; k++) {
                        if(initiator_initiation_frame->rx_ts[k].ranging_id == responder_id) {
                            rx_resp = from_packed_dwt_ts(initiator_initiation_frame->rx_ts[k].ts);
                            break;
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
                    const struct dwt_ranging_frame_info *local_response_info     = &frame_infos[i > initiation_node_slot_offset ? i : round_length + i];
                    const struct dwt_ranging_frame_info *local_finalization_info = &frame_infos[round_length + initiation_node_slot_offset];

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
    }

    return responder_count;
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

        uint64_t own_duration = correct_overflow(curr_tmpl->tx_final,  curr_tmpl->tx_init);
        uint64_t other_duration = correct_overflow(curr_tmpl->rx_final, curr_tmpl->rx_init);

        curr_off->transmitter_id = initiator_id;
        curr_off->receiver_id = responder_id;
        curr_off->offset = (float)((int64_t)own_duration-(int64_t)other_duration) / (float) (other_duration);
        offset_cnt++;

        // TODO maybe we can do this code somewhat cleaner, for instance i am still not sure if we
        // should do this independent of the detection of a completed twr round (for now this code
        // assumes that we only want to know the offset for the later calculation of a tdoa
        // measurement based on a overheard TWR exchange, but maybe we also have use for these
        // values independent of the existance of a successfull round?
        if(initiator_id != node_ranging_id && responder_id != node_ranging_id) {
            curr_off = &offsets[offset_cnt];

            if(curr_tmpl->local_rx_final == UINT64_MAX && curr_tmpl->local_rx_init == UINT64_MAX)
                continue;

            // we are the other in this case
            other_duration = correct_overflow(curr_tmpl->local_rx_final, curr_tmpl->local_rx_init);

            curr_off->transmitter_id = initiator_id;
            curr_off->receiver_id = node_ranging_id;
            curr_off->offset = (float)((int64_t)own_duration-(int64_t)other_duration) / (float) (other_duration);
            offset_cnt++;
        }
    }

    return offset_cnt;
}


/* Although this function gets passed the whole twr_template it will only access
 * the timestamps of the first exchange between initiator and responder.  This
 * is because we require the second exchange for calculating the clock frequency
 * offset between the ranging devices, but this frequency might also be derived
 * by other means for example using the Carrier Frequency Offset register of the
 * DW1000. Because of this we pass the frequency as a seperate argument, so it
 * may be replaced by CFO derived frequency offset. In this case we expect that
 * the timestamps from the second exchange are remaining at their unitialized
 * state which is the value UINT64_MAX
 */

enum measurement_calculation_mode {
  CALCULATE_DSTDOA,
  CALCULATE_CFOTDOA,
  CALCULATE_DSTWR,
  CALCULATE_SSTWR,
  CALCULATE_CFOTWR,
};

int calculate_measurements(const struct twr_template *twr_templates,
    int template_count,
    const struct freq_offset *offsets,
    int frequency_count,
    struct measurement *meas) {
    int meas_cnt = 0;

    // iterate over all templates
    for (int i = 0; i < template_count; i++) {
        const struct twr_template *curr_tmpl = &twr_templates[i];
        const struct freq_offset *responder_offset_to_initiator = NULL;

        for (int j = 0; j < frequency_count; j++) {
            responder_offset_to_initiator = &offsets[j];

            if(responder_offset_to_initiator->transmitter_id == curr_tmpl->ranging_initiator_id && responder_offset_to_initiator->receiver_id == curr_tmpl->ranging_responder_id) {
                break;
            }
        }

        if (responder_offset_to_initiator == NULL || curr_tmpl->tx_init == UINT64_MAX || curr_tmpl->rx_init == UINT64_MAX || curr_tmpl->tx_resp == UINT64_MAX || curr_tmpl->rx_resp == UINT64_MAX)
            continue;

        uint64_t round_dur_initiator =
            correct_overflow(curr_tmpl->rx_resp, curr_tmpl->tx_init);
        uint64_t delay_dur_responder =
            correct_overflow(curr_tmpl->tx_resp,  curr_tmpl->rx_init);

        int64_t drift_offset_int = round(- responder_offset_to_initiator->offset * (float)delay_dur_responder);

        meas[meas_cnt].ranging_initiator_id = curr_tmpl->ranging_initiator_id;
        meas[meas_cnt].ranging_responder_id = curr_tmpl->ranging_responder_id;
        meas[meas_cnt].tof = (float)((int64_t)round_dur_initiator -
                                     (int64_t)delay_dur_responder + drift_offset_int) *
                             0.5;

        // calculate passive tdoa value
        if (curr_tmpl->ranging_initiator_id != node_ranging_id &&
            curr_tmpl->ranging_responder_id != node_ranging_id) {
            const struct freq_offset *passive_offset_to_initiator = NULL;

            for (int j = 0; j < frequency_count; j++) {
                passive_offset_to_initiator = &offsets[j];

                if (passive_offset_to_initiator->transmitter_id == curr_tmpl->ranging_initiator_id && passive_offset_to_initiator->receiver_id == node_ranging_id) {
                    break;
                }
            }

            if (curr_tmpl->local_rx_init == UINT64_MAX || curr_tmpl->local_rx_resp == UINT64_MAX)
                continue;

            uint64_t tx_reception_diff = correct_overflow(curr_tmpl->local_rx_resp, curr_tmpl->local_rx_init);

            int64_t passive_to_initiator_drift_offset_int = round(-passive_offset_to_initiator->offset * (float)tx_reception_diff);

            meas[meas_cnt].tdoa = (int64_t)round_dur_initiator -
                                  meas[meas_cnt].tof - tx_reception_diff +
                                  passive_to_initiator_drift_offset_int;
        } else {
            // we don't want to calculate tdoa values from other nodes
            // perspective, or if we are involved in the measurement, so we
            // always put NaN for now
            meas[meas_cnt].tdoa = NAN;
        }

        meas_cnt++;
    }

    return meas_cnt;
}

enum output_mode {
    OUTPUT_ASYNC = 0, // if output to be written is smaller than 255 bytes use this (this will be the case for TWR), we can than directly start with the next ranging round
    OUTPUT_ASYNC_LOOP = 1, // otherwise this can be used to use DMA API in a looping fashion, should still be faster than default polling interface.
    OUTPUT_TDOA = 2,
    OUTPUT_TWR = 4,
    OUTPUT_FORMAT_JSON = 8,
    OUTPUT_FORMAT_BASE64 = 16
};

static void print_node_information() {
    int written = 0;

    written +=
	    snprintf(serial_buf, sizeof(serial_buf),
                "{\"event\": \"node_info\", \"node_id\": \"0x%04hx\", \"node_ranging_id\": %u}\n", get_own_node_id(), node_ranging_id);


    uart_poll_serial_output_msg(serial_buf);
}

// output measurements. Only output measurements in which we are either responder, initiator, or passive listener
void output_measurements(const struct measurement *meas, int tof_count,
    enum output_mode mode, uint64_t rtc_slot_ts) {
    int written = 0;

    // there is currently no chance that a full round will fit into our budget for async transfers.
    if(mode & OUTPUT_ASYNC && mode & OUTPUT_TDOA) {
        LOG_ERR("Not supported");
        return;
    }

    if (mode & OUTPUT_TWR) {
        // Create output
	written += snprintf(serial_buf, sizeof(serial_buf), "{\"event\":\"twr\",\"i\":%u,\"rtc_round_ts\": %llu,\"t\":[",
            node_ranging_id, rtc_slot_ts);

        uint8_t insert_comma = 0;
	for (int i = 0; i < tof_count; i++) {
            if(meas[i].ranging_initiator_id == node_ranging_id) {
                uint8_t id = meas[i].ranging_responder_id;
		float dist = time_to_dist(meas[i].tof) * DIST_OUTPUT_UNIT;

                if (insert_comma) {
                    written += snprintf(serial_buf + written, sizeof(serial_buf) - written, ",");
                }

		written += snprintf(serial_buf + written, sizeof(serial_buf) - written,
				    "{\"i\":%u,\"d\":%d,\"q\":%d}", id, (int32_t)(dist), 100);

                insert_comma = 1;
            }
	}

        written += snprintf(serial_buf + written, sizeof(serial_buf) - written, "]}\r\n");

        // Output output
        if(written > 0 && mode == OUTPUT_ASYNC) {
            if(written > 255) {
                LOG_ERR("Not supported on NRF52832");
                k_sem_give(&uart_transfer_finished);
            } else {
		uart_callback_set(uart_dev, uart_finished_callback, NULL);

                k_sem_take(&uart_transfer_finished, K_FOREVER);
                int ret = uart_tx(uart_dev, serial_buf, written, SYS_FOREVER_US);

                if (ret) {
                    LOG_ERR("UART TX failed: %d", ret);
                    k_sem_give(&uart_transfer_finished);
                }
            }
        } else {
            uart_poll_serial_output_msg(serial_buf);
        }
    }

    written = 0;
    if (mode & OUTPUT_TDOA) {
        written +=
            snprintf(serial_buf, sizeof(serial_buf),
                "{\"event\":\"td\",\"i\":%u,\"rtc_round_ts\": %llu,\"t\":[", node_ranging_id, rtc_slot_ts);
        uint8_t insert_comma = 0;

        for (int i = 0; i < tof_count; i++) {
            uint8_t initiator = meas[i].ranging_responder_id, responder = meas[i].ranging_initiator_id;

            if(initiator != node_ranging_id && responder != node_ranging_id && meas[i].tdoa != NAN) {
		float dist = time_to_dist(meas[i].tdoa) * DIST_OUTPUT_UNIT;

		if (insert_comma) {
		    written += snprintf(serial_buf + written, sizeof(serial_buf) - written, ",");
                }

                written +=
                    snprintf(serial_buf + written, sizeof(serial_buf) - written,
                             "{\"i\":[%u,%u],\"d\":%d,\"q\":%d}",
                             initiator, responder, (int32_t)(dist), 100);
                insert_comma = 1;
            }
        }

        written += snprintf(serial_buf + written, sizeof(serial_buf) - written, "]}\r\n");

	if (written > 0) {
	    if (mode & OUTPUT_ASYNC_LOOP) {
		// write out buffer in chunks of 255 bytes
                uart_callback_set(uart_dev, uart_finished_callback, NULL);

                int uart_written = 0;
		while (written - uart_written > 0) {
		    k_sem_take(&uart_transfer_finished, K_FOREVER);

		    int ret = uart_tx(uart_dev, serial_buf + uart_written, MIN(written - uart_written, 255), SYS_FOREVER_US);

                    if (ret) {
                        LOG_ERR("UART TX failed: %d", ret);
                        k_sem_give(&uart_transfer_finished);
		    }

                    uart_written += MIN(written - uart_written, 255);
                }
            } else {
		uart_poll_serial_output_msg(serial_buf);
            }
        }
    }
}

enum extraction_mode {
  EXTRACT_LOCAL_ONLY,
  EXTRACT_LOCAL_BIDIRECTIONAL,
  EXTRACT_ALL
};



void calculate_distances_from_frames(const struct dwt_ranging_frame_info *frame_infos, enum extraction_mode mode, uint8_t round_length, uint8_t repetitions, uint64_t rtc_slot_ts) {
    // we are a little bit wastefull with our space here but that should be alright.
    static struct twr_template templates[MAX_ROUND_LENGTH*MAX_ROUND_LENGTH];
    static struct freq_offset offsets[2*MAX_ROUND_LENGTH*MAX_ROUND_LENGTH]; // in the worst case we will get a quadratic amount of measurements + for each node pair a passively extracted frequency offset
    static struct measurement meas[MAX_ROUND_LENGTH*MAX_ROUND_LENGTH];
    static uint8_t ranging_ids[MAX_ROUND_LENGTH];

    int ranging_id_count = get_ranging_ids(frame_infos, ranging_ids, round_length);
    int extracted_templates = 0;

    SET_GPIO_HIGH(0);
    if(mode == EXTRACT_LOCAL_ONLY) {
        int template_cnt = fill_twr_template_from_frames(frame_infos, node_ranging_id, round_length, repetitions, templates, EXTRACT_WITH_REJECT);
        extracted_templates += template_cnt;
    } else if (mode == EXTRACT_LOCAL_BIDIRECTIONAL) {
        LOG_ERR("Not implemented yet");
        return;
    } else {
        for(int i = 0; i < ranging_id_count; i++) {
            int template_cnt = fill_twr_template_from_frames(
                frame_infos, ranging_ids[i], round_length, repetitions,
                templates + extracted_templates, EXTRACT_WITH_REJECT);

            if (template_cnt > 0) {
                // TODO remove again
                extracted_templates += template_cnt;
            }
        }
    }

    int offset_cnt = calculate_frequency_offsets(templates, extracted_templates, offsets);
    int meas_cnt = calculate_measurements(
        templates, extracted_templates, offsets, offset_cnt, meas);

    output_measurements(meas, meas_cnt, OUTPUT_TDOA | OUTPUT_TWR, rtc_slot_ts); // OUTPUT_ASYNC_LOOP
    /* output_measurements(meas, meas_cnt, OUTPUT_TWR, rtc_slot_ts); //OUTPUT_ASYNC_LOOP */

    SET_GPIO_LOW(0);
}


void output_frame_timestamps(const struct dwt_ranging_frame_info *frame_infos, uint8_t round_size, uint8_t repetitions, uint64_t rtc_slot_ts) {
    for (int i = 0; i < repetitions-1; i++) { // the last repetition/phase is not of importance for us
        for(int j = 0; j < round_size; j++) {
            const struct dwt_ranging_frame_info *curr_frame_info = &frame_infos[i*round_size + j];

            if(curr_frame_info->frame != NULL) {
                const struct dwt_ranging_frame_buffer *curr_frame = curr_frame_info->frame;

                if(curr_frame->ranging_id == node_ranging_id) {
                    snprintf(serial_buf, sizeof(serial_buf), "{\"event\": \"tx\", \"own_id\": %u, \"rtc_round_ts\": %llu, \"phase\": %u, \"slot\": %u, \"ts\": %llu}\n", node_ranging_id, rtc_slot_ts, i, j, curr_frame_info->timestamp);
                    uart_poll_serial_output_msg(serial_buf);
                } else {
                    snprintf(serial_buf, sizeof(serial_buf), "{\"event\": \"rx\", \"own_id\": %u, \"other_id\": %u, \"rtc_round_ts\": %llu, \"phase\": %u, \"slot\": %u, \"ts\": %llu, \"fp_index\": %u}\n", node_ranging_id, curr_frame->ranging_id, rtc_slot_ts, i, j, curr_frame_info->timestamp, curr_frame_info->fp_index >> 6);
                    uart_poll_serial_output_msg(serial_buf);
                }
            }
        }
    }
}

static struct timeutil_sync_config sync_conf = {
    .ref_Hz = CONFIG_SYS_CLOCK_TICKS_PER_SEC,
    .local_Hz = CONFIG_SYS_CLOCK_TICKS_PER_SEC
};

static struct timeutil_sync_state rtc_clock_sync_state = {
    .cfg = &sync_conf
};


// big TODO i am not happy at all yet with how the schedule next events
void schedule_network_next_event();

void timesync_check_gpio_handler(struct k_work *work) {
    SET_GPIO_HIGH(0);
    SET_GPIO_LOW(0);

    schedule_network_next_event();
}

void dwt_glossy_handler(struct k_work *work)
{
    struct dwt_glossy_tx_result sync_result;
    int written = 0;

#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->start(ieee802154_dev);
#endif
    written +=
        snprintf(serial_buf, sizeof(serial_buf), "{\"event\": \"glossy\", \"node_id\": \"0x%04hx\"", get_own_node_id());

    // wait at at most 10 ms for glossy packet
    if ( dwt_glossy_tx_timesync(ieee802154_dev, node_ranging_id == CONFIG_GLOSSY_TX_FLOOD_START_NODE_ID, node_ranging_id, 0, &sync_result) >= 0) {
        if( timeutil_sync_state_update(&rtc_clock_sync_state, &sync_result.clock_sync_instant) ) {
            float skew;

            skew = timeutil_sync_estimate_skew(&rtc_clock_sync_state);
            timeutil_sync_state_set_skew(&rtc_clock_sync_state, skew, NULL);

	    written += snprintf(serial_buf + written, sizeof(serial_buf) - written,
				", \"rtc_offset\": %d", timeutil_sync_skew_to_ppb(skew));

            timesync_skew_update_count++;

	}
    }

    written += snprintf(serial_buf + written, sizeof(serial_buf) - written, "}\n");
    uart_poll_serial_output_msg(serial_buf);

#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->stop(ieee802154_dev);
#endif

    schedule_network_next_event();
}


void dwt_mtm_handler(struct k_work *work)
{
#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->start(ieee802154_dev);
#endif

    // wait at most 500 us for start
    struct dwt_ranging_frame_info *frame_infos;


    uint8_t ranging_slot = node_ranging_id;

    if (node_ranging_id == 5) {
        ranging_slot = DWT_NO_TX_SLOT;
    }

    struct mtm_ranging_config conf = {
	    .round_length = NUM_NODES,
            .repetitions = 3,
	    .tx_slot_offset = ranging_slot,
	    .use_initiation_frame = 0,
	    .cca = 0,
	    .reject_frames = 1,
	    .valid_fp_index_range = 10,
	    .timeout_us = 500,
    };


    SET_GPIO_HIGH(0);
    if (!dwt_mtm_ranging(ieee802154_dev, node_ranging_id, &conf, &frame_infos)) {
        SET_GPIO_LOW(0);
        // TODO outputting should not be done in this function i think
	/* calculate_distances_from_frames(frame_infos, EXTRACT_ALL, NUM_NODES, 3); */
        output_frame_timestamps(frame_infos, NUM_NODES, 3, next_event_time);
    }
    SET_GPIO_LOW(0);

#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->stop(ieee802154_dev);
#endif

    schedule_network_next_event();
}



void dwt_mtm_cca_handler(struct k_work *work) {
#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->start(ieee802154_dev);
#endif

    SET_GPIO_HIGH(0);
    // wait at most 500 us for start

    LOG_WRN("CCA");

    uint8_t cca_duration = (sys_rand32_get() % MAX_PAC_TIMEOUT);

    struct mtm_ranging_config conf = {
        .round_length = NUM_NODES,
        .repetitions = 3,
        .tx_slot_offset = DWT_TX_AUTO,
        .use_initiation_frame = 0,
        .cca = 1,
        .cca_duration = cca_duration,
        .reject_frames = 1,
        .valid_fp_index_range = 10,
        .timeout_us = 500,
    };


    struct dwt_ranging_frame_info *frame_infos;
    if ( !dwt_mtm_ranging(ieee802154_dev, node_ranging_id, &conf, &frame_infos) ) {
        calculate_distances_from_frames(frame_infos, EXTRACT_LOCAL_ONLY, NUM_NODES, 3, next_event_time);
    }

    SET_GPIO_LOW(0);
#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->stop(ieee802154_dev);
#endif

    schedule_network_next_event();
}

void dwt_mtm_aloha_handler(struct k_work *work)
{
#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->start(ieee802154_dev);
#endif

    // map nodes onto 2 slots
    uint8_t slot_offset = (sys_rand32_get() % MTM_ALOHA_SLOTS);

    // try to maximize channel utilization
    uint8_t access_medium = (sys_rand32_get() % ALOHA_ACCESS_PROB_MULTIPLIER) < aloha_access_prob[node_ranging_id];

    // for now put every node on slot 1
    if (!access_medium) {
        slot_offset = DWT_NO_TX_SLOT;
    }

    struct dwt_ranging_frame_info *frame_infos;

    struct mtm_ranging_config conf = {
        .round_length = MTM_ALOHA_SLOTS,
        .repetitions = 3,
        .tx_slot_offset = slot_offset,
        .use_initiation_frame = 0,
        .cca = 0,
        .reject_frames = 0,
        .valid_fp_index_range = 20,
        .timeout_us = 500,
    };

    if ( !dwt_mtm_ranging(ieee802154_dev, node_ranging_id, &conf, &frame_infos) ) {
        calculate_distances_from_frames(frame_infos, EXTRACT_ALL, MTM_ALOHA_SLOTS, 3, next_event_time);
        /* output_frame_timestamps(frame_infos, MTM_ALOHA_SLOTS, 3, next_event_time); */
    }

#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->stop(ieee802154_dev);
#endif

    schedule_network_next_event();
}

static void seeded_hash_permute_until_index(uint8_t permutation[], size_t overall_size, size_t permutation_size, uint64_t seed) // TODO: use PRNG!
{
        // we only swap until we reach at least permutation_size items
        int limit = permutation_size > 0 ? MIN(overall_size-2, permutation_size) : overall_size-2;

        for (int i = 0; i <= limit; i++) {
                uint64_t hash_input = seed + i; // TODO: Use PRNG which should be more efficient than this
                uint32_t hash_output = sys_hash32((void *)&hash_input, sizeof(hash_input));

                int j = i + hash_output % (overall_size-i); /* A random integer such that i ≤ j < n */

                uint8_t val = permutation[i]; /* Swap the randomly picked element with permutation[i] */
                permutation[i] = permutation[j];
                permutation[j] = val;
        }
}

void dwt_mtm_hashed_handler(struct k_work *work)
{
#if CONFIG_RANGING_RADIO_SLEEP
        radio_api->start(ieee802154_dev);
#endif

        int tx_slot = DWT_NO_TX_SLOT;
        int ret = 0;

        uint8_t slot_node_assignment[NUM_NODES];
        //memset(slot_node_assignment, 0, sizeof(slot_node_assignment));
        for(int i = 0; i < NUM_NODES; i++){
                slot_node_assignment[i] = i; // we assign slot i to node i for now and then we shuffle them. // TODO is the ranging ID starting from 0 or 1?!
        }

        seeded_hash_permute_until_index(slot_node_assignment, sizeof(slot_node_assignment)/sizeof(slot_node_assignment[0]), MTM_HASHED_SLOTS, (uint64_t) next_event_time);

        for (int k = 0; k < MIN(MTM_HASHED_SLOTS, NUM_NODES); k++) {

                if (slot_node_assignment[k] == node_ranging_id) {
                    tx_slot = k;
                }
        }

        struct dwt_ranging_frame_info *frame_infos;

        struct mtm_ranging_config conf = {
                .round_length = MTM_HASHED_SLOTS,
                .repetitions = 3,
                .tx_slot_offset = tx_slot,
                .use_initiation_frame = 0,
                .cca = 0,
                .reject_frames = 0,
                .valid_fp_index_range = 20,
                .timeout_us = 500,
        };

        if ( (ret = !dwt_mtm_ranging(ieee802154_dev, node_ranging_id, &conf, &frame_infos)) ) {
                calculate_distances_from_frames(frame_infos, EXTRACT_ALL, MTM_HASHED_SLOTS, 3, next_event_time);
                /* output_frame_timestamps(frame_infos, MTM_HASHED_SLOTS, 3, next_event_time); */
        }

        if(ret < 0) {
                int written = 0;

                written += snprintf(serial_buf, sizeof(serial_buf),
                                    "{\"event\": \"dwt_round_error\", \"error\": %d}\n", ret);

                uart_poll_serial_output_msg(serial_buf);
        }

#if CONFIG_RANGING_RADIO_SLEEP
        radio_api->stop(ieee802154_dev);
#endif

        schedule_network_next_event();
}


/* void dwt_mtm_hashed_handler(struct k_work *work) */
/* { */
/* #if CONFIG_RANGING_RADIO_SLEEP */
/*     radio_api->start(ieee802154_dev); */
/* #endif */

/*     // map nodes onto 2 slots */
/*     uint64_t hash_input = next_event_time; */
/*     int tx_slot = DWT_NO_TX_SLOT; */
/*     int ret = 0; */

/*     static uint8_t slot_assignment[MTM_HASHED_SLOTS]; */
/*     memset(slot_assignment, 0xFF, MTM_HASHED_SLOTS * sizeof(uint8_t)); */

/*     snprintf(serial_buf, sizeof(serial_buf), "{\"event\": \"hashed_event\", \"type\": \"start\"}\n"); */
/*     uart_poll_serial_output_msg(serial_buf); */

/*     uint32_t m = 0; */
/*     for (int k = 0; k < MIN(MTM_HASHED_SLOTS, NUM_NODES); k++) { */
/* 	uint8_t found_unassigned = 1; */
/* 	uint8_t tx_id = 0; */

/* 	do { */
/* 	    found_unassigned = 1; */

/*             hash_input += m; */
/*             uint32_t hash_output = sys_hash32_murmur3((void *)&hash_input, sizeof(hash_input)); */
/*             tx_id = hash_output % NUM_NODES; */

/*             for (int i = 0; i < k; i++) { */
/* 	        if (slot_assignment[i] == tx_id) { */
/*                     found_unassigned = 0; */
/*                 } */
/* 	    } */

/* 	    // log transmission_id using serial buf, might be good to access fairness of hash */
/* 	    // function */
/*             /\* snprintf(serial_buf, sizeof(serial_buf), *\/ */
/*                 /\* "{\"event\": \"hashed_event\", \"k\": \"%u\"}\n", k); *\/ */
/*             /\* snprintf(serial_buf, sizeof(serial_buf), *\/ */
/*             /\*     "{\"event\": \"hashed_event\", \"hash_in\": \"%llu\", \"hash_out\": \"%u\" \"id\": \"0x%04hx\"}\n", hash_input, hash_output, tx_id); *\/ */

/*             /\* uart_poll_serial_output_msg(serial_buf); *\/ */
/* 	} while (!found_unassigned); */
/*         m++; */

/*         slot_assignment[k] = tx_id; */

/* 	if (tx_id == node_ranging_id) { */
/*             tx_slot = k; */
/* 	} */
/*     } */

/*     /\* snprintf(serial_buf, sizeof(serial_buf), *\/ */
/*     /\*     "{\"event\": \"hashed_event\", \"type\": \"finished\"}\n"); *\/ */

/*     /\* uart_poll_serial_output_msg(serial_buf); *\/ */

/*     struct dwt_ranging_frame_info *frame_infos; */

/*     struct mtm_ranging_config conf = { */
/*         .round_length = MTM_HASHED_SLOTS, */
/*         .repetitions = 3, */
/*         .tx_slot_offset = tx_slot, */
/*         .use_initiation_frame = 0, */
/*         .cca = 0, */
/*         .reject_frames = 0, */
/*         .valid_fp_index_range = 20, */
/*         .timeout_us = 500, */
/*     }; */

/*     if ( (ret = !dwt_mtm_ranging(ieee802154_dev, node_ranging_id, &conf, &frame_infos)) ) { */
/*         calculate_distances_from_frames(frame_infos, EXTRACT_ALL, MTM_HASHED_SLOTS, 3, next_event_time); */
/*         /\* output_frame_timestamps(frame_infos, MTM_HASHED_SLOTS, 3, next_event_time); *\/ */
/*     } */

/*     if(ret < 0) { */
/*         int written = 0; */

/* 	written += snprintf(serial_buf, sizeof(serial_buf), */
/*             "{\"event\": \"dwt_round_error\", \"error\": %d}\n", ret); */

/*         uart_poll_serial_output_msg(serial_buf); */
/*     } */

/* #if CONFIG_RANGING_RADIO_SLEEP */
/*     radio_api->stop(ieee802154_dev); */
/* #endif */

/*     schedule_network_next_event(); */
/* } */


K_WORK_DEFINE(dwt_glossy_work_item, dwt_glossy_handler);
K_WORK_DEFINE(dwt_mtm_work_item, dwt_mtm_handler);
K_WORK_DEFINE(dwt_mtm_aloha_work_item, dwt_mtm_aloha_handler);
K_WORK_DEFINE(dwt_mtm_cca_work_item, dwt_mtm_cca_handler);
K_WORK_DEFINE(dwt_mtm_hashed_work_item, dwt_mtm_hashed_handler);
K_WORK_DEFINE(gpio_work_item, timesync_check_gpio_handler);

void network_event_handler() {
    if(next_event == RANGING_GLOSSY_SLOT) {
        k_work_submit(&dwt_glossy_work_item);
    } else if(next_event == RANGING_MTM_SLOT) {
        k_work_submit(&dwt_mtm_work_item);
    }

    switch(next_event) {
    case RANGING_GLOSSY_SLOT:
        k_work_submit(&dwt_glossy_work_item);
        break;
    case RANGING_MTM_SLOT:
        k_work_submit(&dwt_mtm_work_item);
        break;
    case RANGING_MTM_ALOHA_SLOT:
        k_work_submit(&dwt_mtm_aloha_work_item);
	break;
    case RANGING_MTM_HASHED_SLOT:
        k_work_submit(&dwt_mtm_hashed_work_item);
        break;
    case RANGING_MTM_CCA_SLOT:
        k_work_submit(&dwt_mtm_cca_work_item);
        break;
    case RANGING_CHECK_TIMESYNC_GPIO:
        k_work_submit(&gpio_work_item);
        break;
    default:
        break;
    }
}

K_TIMER_DEFINE(event_timer, network_event_handler, NULL);

void schedule_network_next_event() {
    uint64_t ref_now, ref_slot_start_ts, local_slot_start_ts;
    k_timeout_t next_event_delay;

    if( node_ranging_id == CONFIG_GLOSSY_TX_FLOOD_START_NODE_ID ) {
        ref_now = k_uptime_ticks();
    } else if(timeutil_sync_ref_from_local(&rtc_clock_sync_state, k_uptime_ticks(), &ref_now) < 0) {//timesync_skew_update_count > 4 || timeutil_sync_ref_from_local(&rtc_clock_sync_state, k_uptime_ticks(), &ref_now) < 0) {
        ref_now = 0;
    }

    if( ref_now > 0 ) {
	ref_slot_start_ts = ref_now + (SYS_TICK_ROUND_LENGTH - ref_now % SYS_TICK_ROUND_LENGTH);

        next_event_time = ref_slot_start_ts;

        if(node_ranging_id != CONFIG_GLOSSY_TX_FLOOD_START_NODE_ID) {
            timeutil_sync_local_from_ref(&rtc_clock_sync_state, ref_slot_start_ts, &local_slot_start_ts);
            next_event_delay = K_TIMEOUT_ABS_TICKS(local_slot_start_ts);
        } else {
            next_event_delay = K_TIMEOUT_ABS_TICKS(ref_slot_start_ts);
        }

        if(ref_slot_start_ts % (SYS_TICK_ROUND_LENGTH * 5) == 0) {
            next_event = RANGING_GLOSSY_SLOT;
        /* } else if(ref_slot_start_ts % (SYS_TICK_ROUND_LENGTH * 5) == 0) { */
            /* next_event = RANGING_CHECK_TIMESYNC_GPIO; */
        } else {
	    next_event = RANGING_MTM_HASHED_SLOT;
	    /* next_event = RANGING_MTM_ALOHA_SLOT; */
            /* next_event = RANGING_MTM_SLOT; */
            /* next_event = RANGING_MTM_CCA_SLOT; */
        }
    } else { // if we are not able yet to transform between timescales we will glossy until we can
        LOG_WRN("Not synced yet, glossy until we are");
        next_event = RANGING_GLOSSY_SLOT;
        next_event_delay = K_NO_WAIT;
    }

    k_timer_start(&event_timer, next_event_delay, K_NO_WAIT);
}

/* https://docs.zephyrproject.org/apidoc/latest/group__timeutil__sync__apis.html */
/* https://docs.zephyrproject.org/latest/services/pm/system.html */
/* https://infocenter.nordicsemi.com/index.jsp?topic=%2Fps_nrf5340%2Fchapters%2Fcurrent_consumption%2Fdoc%2Fcurrent_consumption.html&cp=4_0_0_3_3_0_0&anchor=unique_1675781758 */
// see above Current at ON_IDLE7 which should be about 1.5 uAmps.
int main(void) {
    int ret = 0;
    LOG_INF("Starting ...");
    LOG_INF("Getting node id");

    int16_t signed_node_id = get_node_number(get_own_node_id());
    node_ranging_id = signed_node_id;

    print_node_information();

    if (signed_node_id < 0) {
        LOG_WRN("Node number NOT FOUND! Shutting down :( I am: 0x%04hx", get_own_node_id());
        return 0;
    }

    LOG_INF("Initialize ieee802.15.4");
    ieee802154_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_ieee802154));

    if (!ieee802154_dev) {
        LOG_ERR("Cannot get ieee 802.15.4 device");
        return false;
    }

    LOG_WRN("Own number: %u", node_ranging_id);

    radio_api = (struct ieee802154_radio_api *)ieee802154_dev->api;

    LOG_INF("GOT node id: %u", node_ranging_id);

    LOG_INF("Start IEEE 802.15.4 device");
    ret = radio_api->start(ieee802154_dev);

    if(ret) {
        LOG_ERR("Could not start ieee 802.15.4 device");
        return false;
    }

    if (!device_is_ready(uart_dev)) {
        printk("UART device not found!");
        return 0;
    }

    schedule_network_next_event();

    while (1) {
        k_msleep(5000);
    }
}

enum net_verdict ieee802154_handle_ack(struct net_if *iface, struct net_pkt *pkt)
{
    return NET_CONTINUE;
}

/**
 * Interface to the network stack, will be called when the packet is
 * received
 */
int net_recv_data(struct net_if *iface, struct net_pkt *pkt)
{
    int ret = 0;

    net_pkt_unref(pkt);

    return ret;
}
