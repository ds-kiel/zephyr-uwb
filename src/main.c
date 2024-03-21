#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include <zephyr/devicetree.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/drivers/ieee802154/dw1000.h>
#include <stdio.h>


#include "nodes.h"
#include "log.h"
#include "history.h"

LOG_MODULE_REGISTER(main);

#define NUM_ROUNDS (10000)
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

#define DWT_TS_TO_US(X) (((X)*15650)/1000000000)
#define UUS_TO_DWT_TS(X) (((uint64_t)X)*(uint64_t)65536)

#define EXPERIMENT_NAME "driver mtm ranging"

static uint16_t node_ranging_id;

#define UART_TX_RESULTS 0

static K_SEM_DEFINE(uart_transfer_finished, 1, 1);


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

#define UART_POLLING 0

struct distance_estimate
{
    uint8_t ranging_id;
    float dist;
    float quality; // currently unused but useful in the future
};

// for now this function works on the assumption that all ranging round concluded without any dropped packages
void calculate_distances_from_timestamps(const struct device *dev,  struct dwt_ranging_frame_buffer **buffers, size_t round_length, size_t repetitions)
{
    struct distance_estimate distances[round_length];
    struct _ts {
        uint8_t ranging_id;
        uint64_t rx_init, tx_resp, rx_resp, rx_final;
    } responder_timestamps[round_length];
    uint64_t tx_init, tx_final;

    LOG_DBG("Distance Callback");

    // swipe once through buffers to capture all ranging ids and associate them with a position in distances
    for(size_t i = 0; i < round_length; i++) {
        const struct dwt_ranging_frame_buffer *buffer = buffers[i];
        responder_timestamps[i].ranging_id = buffer->ranging_id;
    }

    // these we will reference more often
    const struct dwt_ranging_frame_buffer *transmitted_initiation_buffer = NULL, *transmitted_finalization_buffer = NULL;
    size_t our_slot_offset;

    // first sweep find our initiation buffer and finalization response
    for(size_t i = round_length; i < repetitions*round_length; i++) {
        const struct dwt_ranging_frame_buffer *buffer = buffers[i];
        if(buffer->ranging_id == node_ranging_id) {
            if(!transmitted_initiation_buffer) {
                transmitted_initiation_buffer = buffer;
                our_slot_offset = i;

                // grab tx_init from buffer
                tx_init = transmitted_initiation_buffer->tx_ts.dwt_ts;
            } else {
                transmitted_finalization_buffer = buffer;
                // grab tx_final from buffer
                tx_final = transmitted_finalization_buffer->tx_ts.dwt_ts;
                break;
            }
        }
    }

    // -- small sanity check
    if(!transmitted_initiation_buffer || !transmitted_finalization_buffer) {
        LOG_ERR("Could not find initiation or finalization buffer");
        return;
    }

/* next we retrieve all timestamps that are specific to other nodes and are not shared */
/* between all calculations */

    // first find rx_init, tx_resp and rx_resp
    for(size_t i = 0; i < round_length; i++) {
        const struct dwt_ranging_frame_buffer *received_initiation_frame = buffers[round_length + i];
        const struct dwt_ranging_frame_buffer *received_finalization_frame = buffers[2*round_length + i];
        uint8_t other_ranging_id = received_initiation_frame->ranging_id;

        if(i != our_slot_offset) {
            uint64_t rx_init = UINT64_MAX, tx_resp = UINT64_MAX, rx_resp = UINT64_MAX, rx_final = UINT64_MAX;

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
                for(size_t k = 0; k < transmitted_finalization_buffer->rx_ts_count; k++) {
                    if(transmitted_finalization_buffer->rx_ts[k].ranging_id == other_ranging_id) {
                        rx_resp = transmitted_finalization_buffer->rx_ts[k].dwt_ts;
                        break;
                    }
                }
            } else {
                tx_resp = received_initiation_frame->tx_ts.dwt_ts;

                // in our own initiation frame we will find rx_resp
                for(size_t k = 0; k < transmitted_initiation_buffer->rx_ts_count; k++) {
                    if(transmitted_initiation_buffer->rx_ts[k].ranging_id == other_ranging_id) {
                        rx_resp = transmitted_initiation_buffer->rx_ts[k].dwt_ts;
                        break;
                    }
                }
            }

            // store in responder_timestamps
            for(size_t k = 0; k < round_length; k++) {
                if(responder_timestamps[k].ranging_id == received_initiation_frame->ranging_id) {
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
    // print vector of distances
    char buf[256];
    size_t written = 0;
    written += snprintf(buf, sizeof(buf), "{\"e\":\"r\",\"i\":%hhu,\"t\":[", node_ranging_id);

    for(size_t i = 0; i < round_length; i++) {
        if(responder_timestamps[i].ranging_id != node_ranging_id) {
            struct _ts *ts = &responder_timestamps[i];
            int32_t tof = compute_prop_time(ts->rx_resp - tx_init, tx_final - ts->rx_resp, ts->rx_final - ts->tx_resp, ts->tx_resp - ts->rx_init);
            float dist = time_to_dist((float)tof);

            // add to return distances
            distances[i].ranging_id = ts->ranging_id;
            distances[i].dist = dist;
            distances[i].quality = 1.0;

            LOG_DBG("Ranging id: %hhu, tof: %d, dist: %dcm, rx_init: %llx, tx_resp: %llx, rx_resp: %llx, rx_final: %llx", ts->ranging_id, tof, (int32_t) (dist * 100), ts->rx_init, ts->tx_resp, ts->rx_resp, ts->rx_final);
            written += snprintf(buf+written, sizeof(buf)-written, "{\"i\":%hhu,\"d\":%d,\"q\":%d}", ts->ranging_id, (int32_t) (dist * 100), 100);
        }
    }
    snprintf(buf+written, sizeof(buf)-written, "]}\r\n");

    LOG_WRN("%s", buf);

    // calculate distances
}

/* static char serial_buf[1024]; */
static atomic_t tx_state;

/* void output_range_measurements(const struct device *dev, const struct dwt_ranging_frame_buffer **buffers) { */
/*     int ret; */
/*     size_t written = 0; */

/*     k_sem_take(&uart_transfer_finished, K_FOREVER); */
/*     memset(serial_buf, 0, sizeof(serial_buf)); */

/* #if !UART_POLLING */
/*     uart_callback_set(uart_dev, uart_finished_callback, NULL); */
/* #endif */

/*     // HEADER */
/*     written += snprintf(serial_buf, sizeof(serial_buf), "{\"e\":\"r\",\"i\":%hhu,\"t\":[", node_ranging_id); */

/*     // TIMESTAMPS */
/*     for (uint8_t i = 0; i < data->len; i++) { */
/*         written += snprintf(serial_buf + written, sizeof(serial_buf) - written, "(%u,%llx)%s", data->timestamps[i].ranging_id, data->timestamps[i].dwt_ts, i == data->len - 1 ? "" : ","); */
/*     } */

/*     written += snprintf(serial_buf + written, sizeof(serial_buf) - written, "]}\r\n"); */

/*     if(written > 255) { */
/*         LOG_ERR("Not supported on NRF52832"); */
/*         goto err; */
/*     } */

/* #if UART_POLLING */
/*     for (size_t i = 0; i < written; i++) { */
/*         uart_poll_out(uart_dev, serial_buf[i]); */
/*     } */
/*     k_sem_give(&uart_transfer_finished); */
/* #else */
/*     ret = uart_tx(uart_dev, serial_buf, written, SYS_FOREVER_US); */

/*     if (ret) { */
/*         LOG_ERR("UART TX failed: %d", ret); */
/*         goto err; */
/*     } */
/* #endif */

/*     return; */

/*   err: */
/*     k_sem_give(&uart_transfer_finished); */
/* } */

/* RANGING_DATA_CALLBACK_DEFINE(DEVICE_DT_GET(DT_CHOSEN(zephyr_ieee802154)), output_range_measurements); */
RANGING_DATA_CALLBACK_DEFINE(DEVICE_DT_GET(DT_CHOSEN(zephyr_ieee802154)), calculate_distances_from_timestamps);

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

    LOG_DBG("Starting main loop");

    uint32_t cur_round = 0;

    while(cur_round < NUM_ROUNDS) {
        LOG_DBG("Starting round %u", cur_round);
        dwt_mtm_ranging(ieee802154_dev, NUM_NODES, node_ranging_id, node_ranging_id);

        /* k_sleep(K_MSEC(500)); */

        cur_round++;
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
