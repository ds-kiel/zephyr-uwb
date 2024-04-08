#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include <zephyr/devicetree.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/drivers/ieee802154/dw1000.h>
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
    RANGING_MTM_ALOHA_SLOT
};

enum network_event next_event = RANGING_NONE;

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


struct distance_estimate
{
    uint8_t ranging_id;
    float dist;
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


static char serial_buf[256];
static atomic_t tx_state;

// for now this function works on the assumption that all ranging round concluded without any dropped packages
void calculate_distances_from_timestamps(struct dwt_ranging_frame_buffer *frames, size_t round_length, size_t repetitions)
{
    struct distance_estimate distances[round_length];
    struct _ts {
        uint8_t ranging_id;
        uint64_t rx_init, tx_resp, rx_resp, rx_final;
    } responder_timestamps[round_length-1];
    uint64_t tx_init, tx_final;

    // swipe once through frames to capture all ranging ids and associate them with a position in distances
    for(size_t i = 0, k = 0; i < round_length; i++) {
        const struct dwt_ranging_frame_buffer *frame = &frames[i];
        if(frame->ranging_id != node_ranging_id) {
            responder_timestamps[k++].ranging_id = frame->ranging_id;
        }
    }

    // these we will reference more often
    const struct dwt_ranging_frame_buffer *transmitted_initiation_frame = NULL, *transmitted_finalization_frame = NULL;
    size_t our_slot_offset;

    // first sweep find our initiation buffer and finalization response
    for(size_t i = round_length; i < repetitions*round_length; i++) {
        const struct dwt_ranging_frame_buffer *buffer = &frames[i];
        if(buffer->ranging_id == node_ranging_id) {
            if(!transmitted_initiation_frame) {
                transmitted_initiation_frame = buffer;
                our_slot_offset = i - round_length;

                // grab tx_init from buffer
                tx_init = transmitted_initiation_frame->tx_ts.dwt_ts;
            } else {
                transmitted_finalization_frame = buffer;
                // grab tx_final from buffer
                tx_final = transmitted_finalization_frame->tx_ts.dwt_ts;
                break;
            }
        }
    }

    // -- small sanity check
    if(!transmitted_initiation_frame || !transmitted_finalization_frame) {
        LOG_ERR("Could not find initiation or finalization buffer");
        return;
    }

/* next we retrieve all timestamps that are specific to other nodes and are not shared */
/* between all calculations */

    // first find rx_init, tx_resp and rx_resp
    for(size_t i = 0; i < round_length; i++) {
        if(i != our_slot_offset) {
            uint64_t rx_init = UINT64_MAX, tx_resp = UINT64_MAX, rx_resp = UINT64_MAX, rx_final = UINT64_MAX;

            const struct dwt_ranging_frame_buffer *received_initiation_frame = &frames[round_length + i];
            const struct dwt_ranging_frame_buffer *received_finalization_frame = &frames[2*round_length + i];
            uint8_t other_ranging_id = received_initiation_frame->ranging_id;

            if(received_initiation_frame->ranging_id != received_finalization_frame->ranging_id) {
                LOG_ERR("Initiation and finalization frame do not match %hhu != %hhu", received_initiation_frame->ranging_id, received_finalization_frame->ranging_id);
                return;
            }

            // in any case the received initiation frame will contain rx_init
            for(size_t k = 0; k < received_initiation_frame->rx_ts_count; k++) {
                if(received_initiation_frame->rx_ts[k].ranging_id == node_ranging_id) {
                    rx_init = received_initiation_frame->rx_ts[k].dwt_ts;
                    break;
                }
            }

            // in any case we also find the rx_final timestamp in the received finalization frame
            for(size_t k = 0; k < received_finalization_frame->rx_ts_count; k++) {
                if(received_finalization_frame->rx_ts[k].ranging_id == node_ranging_id) {
                    rx_final = received_finalization_frame->rx_ts[k].dwt_ts;
                    break;
                }
            }

            if(i < our_slot_offset) {
                tx_resp = received_finalization_frame->tx_ts.dwt_ts;

                // in our own finalization frame we will find rx_resp
                for(size_t k = 0; k < transmitted_finalization_frame->rx_ts_count; k++) {
                    if(transmitted_finalization_frame->rx_ts[k].ranging_id == other_ranging_id) {
                        rx_resp = transmitted_finalization_frame->rx_ts[k].dwt_ts;
                        break;
                    }
                }
            } else {
                tx_resp = received_initiation_frame->tx_ts.dwt_ts;

                // in our own initiation frame we will find rx_resp
                for(size_t k = 0; k < transmitted_initiation_frame->rx_ts_count; k++) {
                    if(transmitted_initiation_frame->rx_ts[k].ranging_id == other_ranging_id) {
                        rx_resp = transmitted_initiation_frame->rx_ts[k].dwt_ts;
                        break;
                    }
                }
            }

            // store in responder_timestamps
            for(size_t k = 0; k < round_length-1; k++) {
                if(responder_timestamps[k].ranging_id == other_ranging_id) {
                    responder_timestamps[k].rx_init = rx_init;
                    responder_timestamps[k].tx_resp = tx_resp;
                    responder_timestamps[k].rx_resp = rx_resp;
                    responder_timestamps[k].rx_final = rx_final;
                    break;
                }
            }
        }
    }

    LOG_DBG("tx_init: %llx, tx_final: %llx", tx_init, tx_final);

    k_sem_take(&uart_transfer_finished, K_FOREVER);
    memset(serial_buf, 0, sizeof(serial_buf));

    size_t written = 0;
    written += snprintf(serial_buf, sizeof(serial_buf), "{\"e\":\"r\",\"i\":%hhu,\"t\":[", node_ranging_id);

    for(size_t i = 0; i < round_length-1; i++) {
        struct _ts *ts = &responder_timestamps[i];
        /* int32_t tof = compute_prop_time(ts->rx_resp - tx_init, tx_final - ts->rx_resp, ts->rx_final - ts->tx_resp, ts->tx_resp - ts->rx_init); */
        float tof = calculate_propagation_time_alternative(tx_init, ts->rx_init, ts->tx_resp, ts->rx_resp, tx_final, ts->rx_final);
        float dist = time_to_dist((float)tof);

        // add to return distances
        distances[i].ranging_id = ts->ranging_id;
        distances[i].dist = dist;
        distances[i].quality = 1.0;

        LOG_DBG("Ranging id: %hhu, dist: %dcm, rx_init: %llx, tx_resp: %llx, rx_resp: %llx, rx_final: %llx", ts->ranging_id, (int32_t) (dist * 100), ts->rx_init, ts->tx_resp, ts->rx_resp, ts->rx_final);
        written += snprintf(serial_buf+written, sizeof(serial_buf)-written, "{\"i\":%hhu,\"d\":%d,\"q\":%d}%s", ts->ranging_id, (int32_t) (dist * 10000.0f), 100, i < round_length-2 ? "," : "");
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


/* RANGING_DATA_CALLBACK_DEFINE(DEVICE_DT_GET(DT_CHOSEN(zephyr_ieee802154)), calculate_distances_from_timestamps); */

static struct timeutil_sync_config sync_conf = {
    .ref_Hz = CONFIG_SYS_CLOCK_TICKS_PER_SEC,
    .local_Hz = CONFIG_SYS_CLOCK_TICKS_PER_SEC
};

static struct timeutil_sync_state rtc_clock_sync_state = {
    .cfg = &sync_conf
};

#define SLOT_LENGTH_TIMESTAMP_MS 50
#define SLOT_RX_GUARD_US         100
#define SYS_TICK_RX_GUARD_LENGTH ((CONFIG_SYS_CLOCK_TICKS_PER_SEC * SLOT_RX_GUARD_US) / 1000000)
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
            uint64_t ref_now;
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

#define USE_INITIATION_FRAME 1
void dwt_mtm_handler(struct k_work *work)
{
#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->start(ieee802154_dev);
#endif

    // wait at most 500 us for start
    struct dwt_ranging_frame_buffer *frames;
    if ( !dwt_mtm_ranging(ieee802154_dev, NUM_NODES, node_ranging_id, node_ranging_id, USE_INITIATION_FRAME, 500, &frames) ) {
        calculate_distances_from_timestamps(frames, NUM_NODES, 3);
    }

#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->stop(ieee802154_dev);
#endif

    schedule_network_next_event();
}


#define MTM_ALOHA_SLOTS 2
void dwt_mtm_aloha_handler(struct k_work *work)
{
#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->start(ieee802154_dev);
#endif

    // map nodes onto 2 slots
    uint8_t slot_offset = (rand() % 5);
    /* uint8_t join_round = rand() % 2; */
    uint8_t join_round = 1;

    if(join_round) {
        // for now put every node on slot 1
        struct dwt_ranging_frame_buffer *frames;
        if ( !dwt_mtm_ranging(ieee802154_dev, MTM_ALOHA_SLOTS, slot_offset, node_ranging_id, USE_INITIATION_FRAME, 500, &frames) ) {
            calculate_distances_from_timestamps(frames, MTM_ALOHA_SLOTS, 3);
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
        /* } else (ref_slot_start_ts % (SYS_TICK_ROUND_LENGTH * 10) == 0) { */
        } else {
            /* next_event = RANGING_MTM_ALOHA_SLOT; */
            next_event = RANGING_MTM_SLOT;
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
