#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include <zephyr/devicetree.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/drivers/ieee802154/dw1000.h>
#include <zephyr/random/random.h>

#include <zephyr/sys/base64.h>

#include <stdio.h>

#include <zephyr/sys/timeutil.h>
#include <zephyr/drivers/sensor.h>

#include "nodes.h"
#include "log.h"
#include "history.h"

#include "debug-gpio.h"

LOG_MODULE_REGISTER(main);

static struct ieee802154_radio_api *radio_api;
static const struct device *ieee802154_dev;

#define DWT_TS_MASK 0xFFFFFFFFFF
#define DWT_TS_TO_US(X) (((X)*15650)/1000000000)
#define UUS_TO_DWT_TS(X) (((uint64_t)X)*(uint64_t)65536)

#define EXPERIMENT_NAME "driver mtm ranging"

static uint16_t node_ranging_id;

#define UART_TX_RESULTS 0
#define UART_POLLING    0

#define SPEED_OF_LIGHT_M_PER_S 299702547.236
#define SPEED_OF_LIGHT_M_PER_UWB_TU                                                                \
	((SPEED_OF_LIGHT_M_PER_S * 1.0E-15) * 15650.0) // around 0.00469175196

#define DIST_OUTPUT_MM 1000
#define DIST_OUTPUT_CM 100
#define DIST_OUTPUT_M  1
#define DIST_OUTPUT_UNIT DIST_OUTPUT_MM

#define FILTER_MEASUREMENTS_THRESHOLD -80
#define MAX_ROUND_LENGTH 15
#define PASSIVE_TDOA 0
#define EXTRACT_ALL_RANGES 0
#define FP_INDEX_VALIDY_RANGE 20
#define EXTRACT_WITH_REJECT           0 // TODO we do this in the ranging directly now

#define SYS_TICK_ROUND_LENGTH    ((CONFIG_SYS_CLOCK_TICKS_PER_SEC * SLOT_LENGTH_TIMESTAMP_MS) / 1000)

#define WITHOUT_INITIATION_FRAME 0
#define WITH_INITIATION_FRAME 1 // TODO maybe create enumerations

// CCA related
#define PAC_SIZE 8
#define PREAMBLE_SYMBOLS 128  //TODO  + 120 // guard times
#define MAX_PAC_TIMEOUT  (PREAMBLE_SYMBOLS / PAC_SIZE)

#define MTM_HASHED_SLOTS MIN(NUM_NODES, MAX_ROUND_LENGTH)
#define MTM_ALOHA_SLOTS  MIN(NUM_NODES, MAX_ROUND_LENGTH)

K_SEM_DEFINE(uart_transfer_finished, 1, 1);

static char serial_buf[1024]; // when TDOA is used has to be quite large, we pretty much use here most spare memory we have left over. Could off course easily be optimized

// fills ids with ranging ids found in the first round of the whole ranging process, return the amount of ids found.
int get_ranging_ids(const struct dwt_ranging_frame_info *frames_info, uint8_t *ranging_ids, int round_length) {
    int id_count = 0;
    for(int i = 0; i < round_length; i++) {
        const struct dwt_ranging_frame_info *frame_info = &frames_info[i];

        if(frame_info->frame != NULL) {
            ranging_ids[id_count] = frame_info->frame->ranging_id;
            id_count++;
        }
    }

    return id_count;
}

/* amount of bytes to write in each batch, encoding using base64 will encode the chunk into a larger
   amount of symbols than bytes which have to be written.  Has to be chosen small enough such that
   each base64 encoded string fits into serial_buf.
 */
// make sure this is a multiple of 4
#define CIR_CHUNK_WRITE_SIZE (4*13)
#define CIR_SERIAL_BUF_ENCODED_OFFSET 500

// See lis2dh sensor example
void ranging_poll_imu_handler(const struct device *sensor) {
	static unsigned int count;
	struct sensor_value accel[3];
	struct sensor_value temperature;
	const char *overrun = "";
	int rc = sensor_sample_fetch(sensor);

	++count;
	if (rc == -EBADMSG) {
		/* Sample overrun.  Ignore in polled mode. */
		if (IS_ENABLED(CONFIG_LIS2DH_TRIGGER)) {
			overrun = "[OVERRUN] ";
		}
		rc = 0;
	}
	if (rc == 0) {
		rc = sensor_channel_get(sensor,
					SENSOR_CHAN_ACCEL_XYZ,
					accel);
	}
	if (rc < 0) {
		printf("ERROR: Update failed: %d\n", rc);
	} else {
		printf("#%u @ %u ms: %sx %f , y %f , z %f",
		       count, k_uptime_get_32(), overrun,
		       sensor_value_to_double(&accel[0]),
		       sensor_value_to_double(&accel[1]),
		       sensor_value_to_double(&accel[2]));
	}

	if (IS_ENABLED(CONFIG_LIS2DH_MEASURE_TEMPERATURE)) {
		if (rc == 0) {
			rc = sensor_channel_get(sensor, SENSOR_CHAN_DIE_TEMP, &temperature);
			if (rc < 0) {
				printf("\nERROR: Unable to read temperature:%d\n", rc);
			} else {
				printf(", t %f\n", sensor_value_to_double(&temperature));
			}
		}

	} else {
		printf("\n");
	}
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
