#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include <zephyr/devicetree.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/drivers/ieee802154/dw1000.h>
#include <zephyr/random/random.h>
#include <stdio.h>

#include <zephyr/sys/timeutil.h>



#include "nodes.h"
#include "log.h"
#include "history.h"

LOG_MODULE_REGISTER(main);

#define NUM_ROUNDS (2500)
#define INITIAL_DELAY_MS 5000

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

static inline void ts_from_uint(ts_t *ts_out, uint64_t val) {
    uint8_t dst[sizeof(uint64_t)];
    sys_put_le64(val, dst);
    memcpy(ts_out, dst, sizeof(ts_t));
}

static inline uint64_t ts_to_uint(const ts_t *ts) {
    uint8_t buf[sizeof(uint64_t)] = {0};
    memcpy(&buf, ts, sizeof(ts_t));
    return sys_get_le64(buf);
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
    RANGING_MTM_ALOHA_SLOT
};

static enum network_event next_event = RANGING_NONE;
static uint64_t next_event_time;

#define UART_TX_RESULTS 0
#define UART_POLLING 0

K_SEM_DEFINE(uart_transfer_finished, 1, 1);



void uart_finished_callback(struct device *dev, struct uart_event *evt, void *user_data)
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

#define SPEED_OF_LIGHT_M_PER_S 299702547.236
#define SPEED_OF_LIGHT_M_PER_UWB_TU ((SPEED_OF_LIGHT_M_PER_S * 1.0E-15) * 15650.0) // around 0.00469175196

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

struct freq_offset
{
    uint8_t ranging_initiator_id, ranging_responder_id;
    float offset;
};

struct tof_estimate
{
    uint8_t ranging_initiator_id, ranging_responder_id;
    float tof;
    float quality; // currently unused but useful in the future
};


float calculate_propagation_time_alternative(uint64_t tx_init, uint64_t rx_init, uint64_t tx_resp, uint64_t rx_resp, uint64_t tx_final, uint64_t rx_final) {
    static float relative_drift_offset;
    static int64_t other_duration, own_duration;
    static int64_t round_duration_a, delay_duration_b;
    /* static float drift_offset_int, two_tof_int; */
    static int64_t drift_offset_int, two_tof_int;

    own_duration   = (tx_final - tx_init) & DWT_TS_MASK;
    other_duration = (rx_final - rx_init) & DWT_TS_MASK;

    // factor determining whether B's clock runs faster or slower measured from the perspective of our clock
    // a positive factor here means that the clock runs faster than ours
    /* relative_drift_offset = (float)((int64_t)own_duration-(int64_t)other_duration) / (float)(other_duration); */
    relative_drift_offset = (float)((int64_t)own_duration-(int64_t)other_duration) / (float)(other_duration);


    round_duration_a = (rx_resp - tx_init) & DWT_TS_MASK;
    delay_duration_b = (tx_resp - rx_init) & DWT_TS_MASK;

    /* drift_offset_int = -relative_drift_offset * (float) delay_duration_b; */
    drift_offset_int = round(-relative_drift_offset * (float) delay_duration_b);

    /* two_tof_int = (float)round_duration_a - (float)delay_duration_b + drift_offset_int; */
    two_tof_int = (int64_t)round_duration_a - (int64_t)delay_duration_b + drift_offset_int;

    return ((float) two_tof_int) * 0.5;
}


static char serial_buf[1024];
static atomic_t tx_state;

static inline uint8_t empty_frame_p(const struct dwt_ranging_frame_buffer *frame) {
    return (frame->ranging_id == 0xFF);
}

#define DIST_OUTPUT_MM 1000
#define DIST_OUTPUT_CM 100
#define DIST_OUTPUT_M  1

#define DIST_OUTPUT_UNIT DIST_OUTPUT_CM
#define MAX_ROUND_LENGTH 15
#define FILTER_MEASUREMENTS_THRESHOLD -80

#define PASSIVE_TDOA 0
#define EXTRACT_ALL_RANGES 0

// fills ids with ranging ids found in the first round of the whole ranging process, return the amount of ids found.
int get_ranging_ids(const struct dwt_ranging_frame_info *frames_info, uint8_t *ids, int round_length) {
    uint8_t id_count = 0;
    for(int i = 0; i < round_length; i++) {
        const struct dwt_ranging_frame_info *frame_info = &frames_info[i];

        if(frame_info->frame != NULL) {
            const struct dwt_ranging_frame_buffer *frame = frame_info->frame;
            if(frame->ranging_id != 0xFF) {
                ids[id_count] = frame->ranging_id;
                id_count++;
            }
        }
    }
    return id_count;
}


/* Assumptions: A node may not change its position in the ranging round in subsequent phases, i.e.,
   if it transmitted in slot N in round K it should also transmit in slot N in round K+1.

   returns amount of calculated twr distances for given initiatior id.
 */
int fill_twr_template_from_frames(const struct dwt_ranging_frame_info *frame_infos, uint8_t initiator_id, int round_length, int repetitions, struct twr_template *twr_templates)
{
    /* static struct tof_estimate distances[MAX_ROUND_LENGTH]; */
    uint8_t responder_count = 0;
    uint64_t tx_init, tx_final;

    // these we will reference more often
    const struct dwt_ranging_frame_buffer *transmitted_initiation_frame = NULL, *transmitted_finalization_frame = NULL;
    uint8_t initiation_node_slot_offset = UINT8_MAX;

    // first sweep find the initiation buffer and finalization buffer of the initiating node
    for(int i = round_length; i < repetitions*round_length; i++) {
        const struct dwt_ranging_frame_info *frame_info = &frame_infos[i];

        if(frame_info->frame != NULL) {
            const struct dwt_ranging_frame_buffer *buffer = frame_info->frame;

            if(buffer->ranging_id == initiator_id) {
                if(!transmitted_initiation_frame) {
                    transmitted_initiation_frame = buffer;
                    initiation_node_slot_offset = i - round_length;

                    // grab tx_init from buffer
                    tx_init = transmitted_initiation_frame->tx_ts;
                } else {
                    transmitted_finalization_frame = buffer;
                    // grab tx_final from buffer
                    tx_final = transmitted_finalization_frame->tx_ts;
                    break;
                }
            }
        }
    }

    // -- small sanity check
    if(!transmitted_initiation_frame || !transmitted_finalization_frame) {
        /* LOG_ERR("Could not find initiation or finalization buffer"); */
        return -ENODATA;
    }

    // swipe once through frames to capture all ranging ids and associate them with a position in distances
    for(int i = 0; i < round_length; i++) {
        const struct dwt_ranging_frame_info *frame_info = &frame_infos[i];

        if(frame_info->frame != NULL) {
            const struct dwt_ranging_frame_buffer *frame = frame_info->frame;

            if(frame->ranging_id != initiator_id && !empty_frame_p(frame) ) {
                twr_templates[responder_count].ranging_responder_id = frame->ranging_id;
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
            const struct dwt_ranging_frame_info *received_initiation_frame_info   = &frame_infos[  round_length + i];
            const struct dwt_ranging_frame_info *received_finalization_frame_info = &frame_infos[2*round_length + i];

            if( received_initiation_frame_info->frame != NULL && received_finalization_frame_info->frame != NULL ) {
                const struct dwt_ranging_frame_buffer *received_initiation_frame = received_initiation_frame_info->frame;
                const struct dwt_ranging_frame_buffer *received_finalization_frame = received_finalization_frame_info->frame;
                uint8_t other_ranging_id = received_initiation_frame->ranging_id;

                if( empty_frame_p(received_initiation_frame) || empty_frame_p(received_finalization_frame) ) {
                    continue;
                }

                if(received_initiation_frame->ranging_id != received_finalization_frame->ranging_id) {
                    LOG_ERR("Initiation and finalization frame do not match %hhu != %hhu", received_initiation_frame->ranging_id, received_finalization_frame->ranging_id);
                    return -EINVAL;
                }

                // in any case the received initiation frame will contain rx_init
                for(int k = 0; k < received_initiation_frame->rx_ts_count; k++) {
                    if(received_initiation_frame->rx_ts[k].ranging_id == initiator_id) {
                        rx_init = received_initiation_frame->rx_ts[k].dwt_ts;
                        break;
                    }
                }

                // in any case we also find the rx_final timestamp in the received finalization frame
                for(int k = 0; k < received_finalization_frame->rx_ts_count; k++) {
                    if(received_finalization_frame->rx_ts[k].ranging_id == initiator_id) {
                        rx_final = received_finalization_frame->rx_ts[k].dwt_ts;
                        break;
                    }
                }

                if(i < initiation_node_slot_offset) {
                    tx_resp = received_finalization_frame->tx_ts;

                    // in our own finalization frame we will find rx_resp
                    for(int k = 0; k < transmitted_finalization_frame->rx_ts_count; k++) {
                        if(transmitted_finalization_frame->rx_ts[k].ranging_id == other_ranging_id) {
                            rx_resp = transmitted_finalization_frame->rx_ts[k].dwt_ts;
                            break;
                        }
                    }
                } else {
                    tx_resp = received_initiation_frame->tx_ts;

                    // in our own initiation frame we will find rx_resp
                    for(int k = 0; k < transmitted_initiation_frame->rx_ts_count; k++) {
                        if(transmitted_initiation_frame->rx_ts[k].ranging_id == other_ranging_id) {
                            rx_resp = transmitted_initiation_frame->rx_ts[k].dwt_ts;
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

                    if(local_init_info->frame != NULL) {
                        local_rx_init = local_init_info->timestamp;
                    }

                    if(local_response_info->frame != NULL) {
                        local_rx_resp = local_response_info->timestamp;
                    }

                    if(local_finalization_info->frame != NULL) {
                        local_rx_final = local_finalization_info->timestamp;
                    }
                }

                // store in twr_templates
                for(int k = 0; k < responder_count; k++) {
                    if(twr_templates[k].ranging_responder_id == other_ranging_id) {
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

// TODO implement
/* int twr_templates_to_dists(const struct twr_template *twr_templates, int template_count, struct tof_estimate *dists) { */
/*     for(int i = 0; i < template_count; i++) { */
/*         const struct twr_template *twr_template = &twr_templates[i]; */
/*         if (twr_template->rx_init == UINT64_MAX || */
/*             twr_template->tx_resp == UINT64_MAX || */
/*             twr_template->rx_resp == UINT64_MAX || */
/*             twr_template->rx_final == UINT64_MAX ) { */
/*             return -ENODATA; */
/*         } */

/*         float tof = calculate_propagation_time_alternative(twr_template->tx_init, twr_template->rx_init, twr_template->tx_resp, twr_template->rx_resp, twr_template->tx_final, twr_template->rx_final); */
/*         float dist = time_to_dist((float)tof)  * DIST_OUTPUT_UNIT; */

/*         struct tof_estimate *dist = &dists[i]; */

/*         if(dist > FILTER_MEASUREMENTS_THRESHOLD) { */
/*             // add to return distances */
/*             dist->ranging_initiator_id = twr_template->ranging_initiator_id; */
/*             dist->ranging_responder_id = twr_template->ranging_responder_id; */
/*             dist->dist = dist; */
/*             dist->quality = 1.0; */
/*         } */
/*     } */

/*     return 0; */
/* } */

int calculate_frequency_offsets(const struct twr_template *twr_templates, uint8_t template_count, struct freq_offset *offsets) {
    int offset_cnt = 0;

    for(int i = 0; i < template_count; i++) {
        const struct twr_template *curr_tmpl = &twr_templates[i];
        struct freq_offset *curr_off = &offsets[offset_cnt];

        if(curr_tmpl->tx_init == UINT64_MAX || curr_tmpl->rx_init == UINT64_MAX || curr_tmpl->tx_final == UINT64_MAX || curr_tmpl->rx_final == UINT64_MAX) {
            continue;
        }

        int64_t own_duration = curr_tmpl->tx_final - curr_tmpl->tx_init;
        int64_t other_duration = curr_tmpl->rx_final - curr_tmpl->rx_init;

        curr_off->ranging_initiator_id = curr_tmpl->ranging_initiator_id;
        curr_off->ranging_responder_id = curr_tmpl->ranging_responder_id;
        curr_off->offset = (float32_t)(own_duration-other_duration) / (float32_t) (other_duration);

        offset_cnt++;
    }
}


/* Although this function gets passed the whole twr_template it will only access the timestamps of
 * the first exchange between initiator and responder.  This is because we require the second
 * exchange for calculating the clock frequency offset between the ranging devices, but this
 * frequency might also be derived by other means for example using the Carrier Frequency Offset
 * register of the DW1000. Because of this we pass the frequency as a seperate argument, so it may
 * be replaced by CFO derived frequency offset. In this case we expect that the timestamps from the
 * second exchange are remaining at their unitialized state which is the value UINT64_MAX
 */
int calculate_time_of_flights(const struct twr_template *twr_templates, uint8_t template_count,
    const struct freq_offset *offsets, uint8_t frequency_count, struct tof_estimate *time_of_flights) {
    uint8_t tof_cnt = 0;

    // iterate over all templates
    for(uint8_t i = 0; i < template_count; i++) {
        const struct freq_offset *curr_offset;

        for(uint8_t j = 0; j < frequency_count; j++) {
        }

        /* int64_t drift_offset_int = round(relative_drift_offsets[a]*(float32_t)round_dur_a - relative_drift_offsets[b]*(float32_t)delay_dur_b); */
        /* time_of_flights[tof_cnt] = (int64_t)round_dur_a - (int64_t)delay_dur_b + drift_offset_int; */

        tof_cnt++;
    }

    return tof_cnt;
}

void output_measurements(const struct tof_estimate *tofs, uint8_t tof_count) {
    k_sem_take(&uart_transfer_finished, K_FOREVER);
    memset(serial_buf, 0, sizeof(serial_buf));

    int written = 0;
    written += snprintf(serial_buf, sizeof(serial_buf), "{\"event\":\"r\",\"i\":%hhu,\"t\":[", node_ranging_id);

    for (int i = 0; i < tof_count; i++) {
        if(tofs[i].ranging_initiator_id == node_ranging_id) {
            uint8_t id = tofs[i].ranging_responder_id;
            float dist = tofs[i].tof;

            written += snprintf(serial_buf+written, sizeof(serial_buf)-written, "{\"i\":%hhu,\"d\":%d,\"q\":%d}%s", id, (int32_t) (dist), 100, i < tof_count-1 ? "," : "");
        }
    }

    written += snprintf(serial_buf+written, sizeof(serial_buf)-written, "]}\r\n");

    /// --- output over UART
    if(written > 255) {
        LOG_ERR("Not supported on NRF52832");
        k_sem_give(&uart_transfer_finished);
    } else {
        uart_callback_set(uart_dev, uart_finished_callback, NULL);

        int ret = uart_tx(uart_dev, serial_buf, written, SYS_FOREVER_US);

        if (ret) {
            LOG_ERR("UART TX failed: %d", ret);
            k_sem_give(&uart_transfer_finished);
        }
    }
}

void calculate_distances_from_timestamps(const struct dwt_ranging_frame_info *frame_infos, uint8_t initiator_id, uint8_t round_length, uint8_t repetitions) {
    // we are a little bit wastefull with our space here but that should be alright.
    static struct twr_template templates[MAX_ROUND_LENGTH*MAX_ROUND_LENGTH];
    static struct freq_offset offsets[MAX_ROUND_LENGTH*MAX_ROUND_LENGTH];
    static struct tof_estimate time_of_flights[MAX_ROUND_LENGTH*MAX_ROUND_LENGTH];

    uint8_t ranging_ids[round_length];
    uint8_t ranging_id_count = get_ranging_ids(frame_infos, ranging_ids, round_length);
    uint16_t extracted_templates = 0;

    SET_GPIO_HIGH(0);
    for(uint8_t i = 0; i < ranging_id_count; i++) {
        int template_cnt = fill_twr_template_from_frames(frame_infos, ranging_ids[i], round_length, repetitions, templates + extracted_templates);

        // TODO remove again

        extracted_templates += template_cnt;
    }

    int offset_cnt = calculate_frequency_offsets(templates, extracted_templates, offsets);
    int tof_cnt  = calculate_time_of_flights(templates, extracted_templates, offsets, offset_cnt, time_of_flights);

    output_measurements(time_of_flights, tof_cnt);

    SET_GPIO_LOW(0);
}

void poll_serial_output_msg(char* msg) {
    while (*msg != '\0') {
        uart_poll_out(uart_dev, *msg);
        msg++;
    }
}

void output_frame_timestamps(const struct dwt_ranging_frame_info *frame_infos, uint8_t round_size, uint8_t repetitions, uint64_t rtc_slot_ts) {
    for (int i = 0; i < repetitions-1; i++) { // the last repetition/phase is not of importance for us
        for(int j = 0; j < round_size; j++) {
            const struct dwt_ranging_frame_info *curr_frame_info = &frame_infos[i*round_size + j];

            if(curr_frame_info->frame != NULL) {
                const struct dwt_ranging_frame_buffer *curr_frame = curr_frame_info->frame;

                if(curr_frame->ranging_id == node_ranging_id) {
                    snprintf(serial_buf, sizeof(serial_buf), "{\"event\": \"tx\", \"own_id\": %u, \"rtc_round_ts\": %llu, \"phase\": %u, \"slot\": %u, \"ts\": %llu}\n", node_ranging_id, rtc_slot_ts, i, j, curr_frame_info->timestamp);
                    poll_serial_output_msg(serial_buf);
                } else {
                    snprintf(serial_buf, sizeof(serial_buf), "{\"event\": \"rx\", \"own_id\": %u, \"other_id\": %u, \"rtc_round_ts\": %llu, \"phase\": %u, \"slot\": %u, \"ts\": %llu}\n", node_ranging_id, curr_frame->ranging_id, rtc_slot_ts, i, j, curr_frame_info->timestamp);
                    poll_serial_output_msg(serial_buf);
                }
            }
        }
    }
}


/* void calculate_tdoa_twr_from_timestamps(const struct dwt_ranging_frame_buffer *frames, int round_length, int repetitions) */
/* { */
/*     static struct tof_estimate distances[MAX_ROUND_LENGTH]; */
/*     uint8_t dist_count = 0; */
/*     uint8_t responder_count = 0; */
/*     static struct _ts { */
/*         uint8_t ranging_id_initiator, ranging_id_responder; */
/*         uint64_t tx_init, tx_final, rx_init, tx_resp, rx_resp, rx_final; */

/* #if PASSIVE_TDOA */
/*         uint64_t rx_pass_init, rx_pass_resp, rx_pass_final; */
/* #endif */
/*     } twr_templates[MAX_ROUND_LENGTH-1]; */
/* } */

/* RANGING_DATA_CALLBACK_DEFINE(DEVICE_DT_GET(DT_CHOSEN(zephyr_ieee802154)), calculate_distances_from_timestamps); */

static struct timeutil_sync_config sync_conf = {
    .ref_Hz = CONFIG_SYS_CLOCK_TICKS_PER_SEC,
    .local_Hz = CONFIG_SYS_CLOCK_TICKS_PER_SEC
};

static struct timeutil_sync_state rtc_clock_sync_state = {
    .cfg = &sync_conf
};

/* #define SLOT_LENGTH_TIMESTAMP_MS 30 */
#define SLOT_LENGTH_TIMESTAMP_MS 100 // for debugging do 500ms slots
#define SYS_TICK_ROUND_LENGTH    ((CONFIG_SYS_CLOCK_TICKS_PER_SEC * SLOT_LENGTH_TIMESTAMP_MS) / 1000)

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

#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->start(ieee802154_dev);
#endif

    // wait at at most 10 ms for glossy packet
    if ( dwt_glossy_tx_timesync(ieee802154_dev, node_ranging_id == CONFIG_GLOSSY_TX_FLOOD_START_NODE_ID, node_ranging_id, 0, &sync_result) >= 0) {
        if( timeutil_sync_state_update(&rtc_clock_sync_state, &sync_result.clock_sync_instant) ) {
            float skew;

            skew = timeutil_sync_estimate_skew(&rtc_clock_sync_state);
            timeutil_sync_state_set_skew(&rtc_clock_sync_state, skew, NULL);

            LOG_WRN("offset: %d PPB", timeutil_sync_skew_to_ppb(skew));
        }
    }

#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->stop(ieee802154_dev);
#endif

    schedule_network_next_event();
}

#define WITHOUT_INITIATION_FRAME 0
#define WITH_INITIATION_FRAME 1 // TODO maybe create enumerations
#define MTM_ALOHA_SLOTS 10

void dwt_mtm_handler(struct k_work *work)
{
#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->start(ieee802154_dev);
#endif

    // wait at most 500 us for start
    struct dwt_ranging_frame_info *frame_infos;
    if ( !dwt_mtm_ranging(ieee802154_dev, NUM_NODES, node_ranging_id, node_ranging_id, WITHOUT_INITIATION_FRAME, 500, 0, 0, &frame_infos) ) {
        /* calculate_distances_from_timestamps(frames, node_ranging_id, NUM_NODES, 3); */
        output_frame_timestamps(frame_infos, NUM_NODES, 3, next_event_time);
    }
#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->stop(ieee802154_dev);
#endif

    schedule_network_next_event();
}


#define PAC_SIZE 8
#define PREAMBLE_SYMBOLS 128  //TODO  + 120 // guard times
#define MAX_PAC_TIMEOUT (PREAMBLE_SYMBOLS/PAC_SIZE)
void dwt_mtm_cca_handler(struct k_work *work) {
#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->start(ieee802154_dev);
#endif

    SET_GPIO_HIGH(0);
    // wait at most 500 us for start

    LOG_WRN("CCA");

    uint8_t cca_duration = (sys_rand32_get() % MAX_PAC_TIMEOUT);

    struct dwt_ranging_frame_info *frame_infos;
    if ( !dwt_mtm_ranging(ieee802154_dev, NUM_NODES, UINT8_MAX, node_ranging_id, 0, 500, 1, cca_duration, &frame_infos) ) {
        calculate_distances_from_timestamps(frame_infos, node_ranging_id, NUM_NODES, 3);
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
    uint8_t nodes_in_vicinity = 3;
    uint8_t access_medium = (sys_rand32_get() % nodes_in_vicinity);

    access_medium = 0;

    // choose some random jitter value from 0 to 40 usf
    uint32_t rand_jitter_us = sys_rand32_get() % 100;
    k_sleep(K_USEC(rand_jitter_us));

    // for now put every node on slot 1
    if(!access_medium) {
        struct dwt_ranging_frame_info *frame_infos;
        if ( !dwt_mtm_ranging(ieee802154_dev, MTM_ALOHA_SLOTS, slot_offset, node_ranging_id, 0, 500, 0, 0, &frame_infos) ) {
            calculate_distances_from_timestamps(frame_infos, node_ranging_id, MTM_ALOHA_SLOTS, 3);
        }
    }

#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->stop(ieee802154_dev);
#endif

    schedule_network_next_event();
}


K_WORK_DEFINE(dwt_glossy_work_item, dwt_glossy_handler);
K_WORK_DEFINE(dwt_mtm_work_item, dwt_mtm_handler);
K_WORK_DEFINE(dwt_mtm_aloha_work_item, dwt_mtm_aloha_handler);
K_WORK_DEFINE(dwt_mtm_cca_work_item, dwt_mtm_cca_handler);
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
    } else if(timeutil_sync_ref_from_local(&rtc_clock_sync_state, k_uptime_ticks(), &ref_now) < 0) {
        ref_now = 0;
    }

    if( ref_now > 0 ) {
        ref_slot_start_ts = ref_now + ( SYS_TICK_ROUND_LENGTH - ref_now % SYS_TICK_ROUND_LENGTH );

        next_event_time = ref_slot_start_ts;

        if(node_ranging_id != CONFIG_GLOSSY_TX_FLOOD_START_NODE_ID) {
            timeutil_sync_local_from_ref(&rtc_clock_sync_state, ref_slot_start_ts, &local_slot_start_ts);
            next_event_delay = K_TIMEOUT_ABS_TICKS(local_slot_start_ts);
        } else {
            next_event_delay = K_TIMEOUT_ABS_TICKS(ref_slot_start_ts);
        }

        if(ref_slot_start_ts % (SYS_TICK_ROUND_LENGTH * 20) == 0) {
            next_event = RANGING_GLOSSY_SLOT;
        } else if(ref_slot_start_ts % (SYS_TICK_ROUND_LENGTH * 10) == 0) {
            next_event = RANGING_CHECK_TIMESYNC_GPIO;
        } else {
            /* next_event = RANGING_MTM_ALOHA_SLOT; */
            next_event = RANGING_MTM_SLOT;
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

    if (signed_node_id < 0) {
        LOG_WRN("Node number NOT FOUND! Shutting down :( I am: 0x%04hx", get_own_node_id());
        return 0;
    }

    node_ranging_id = signed_node_id;

    LOG_INF("Initialize ieee802.15.4");
    ieee802154_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_ieee802154));

    if (!ieee802154_dev) {
        LOG_ERR("Cannot get ieee 802.15.4 device");
        return false;
    }

    LOG_WRN("Own number: %hhu", node_ranging_id);

    radio_api = (struct ieee802154_radio_api *)ieee802154_dev->api;

    LOG_INF("GOT node id: %hhu", node_ranging_id);

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

    k_sleep(K_MSEC(INITIAL_DELAY_MS));

    LOG_DBG("Starting round timer");
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
