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

#include "../src/nodes.h"
#include "../src/log.h"
#include "../src/time-synchronization.h"
#include "../src/utils.h"
#include "../src/debug-gpio.h"

LOG_MODULE_REGISTER(main);

struct sstwr_ranging_conf {
    uint8_t slots;
    int asn;
    uint32_t slot_duration_us;
};

static struct ieee802154_radio_api *radio_api;
static const struct device *ieee802154_dev;
static uint8_t node_ranging_id;

static uint8_t serial_buf[1024];

#define CIR_CHUNK_WRITE_SIZE (4 * 13)

static int current_asn;

int output_cir_rtt(int slot, int phase, const uint8_t *cir_memory, size_t size)
{
    // for debug purposes we will measure the timing this takes, which should be quite hefty
    unsigned int start_ts = k_cycle_get_32(), end_ts;

    int cir_bytes_written = 0;
    int ret, length;

    length = snprintf(serial_buf, sizeof(serial_buf),
        "{\"event\": \"cir\", \"i\": %u, \"asn\": %d, \"slot\": %d, "
        "\"phase\": %d, \"length\": %d, \"data\":\"",
        node_ranging_id, current_asn, slot, phase, size);

    printk("%s", serial_buf);
    do {
        ret = base64_encode(serial_buf, sizeof(serial_buf), &length, cir_memory + cir_bytes_written,
			    MIN(CIR_CHUNK_WRITE_SIZE, size - cir_bytes_written));

	if (ret >= 0) {
            serial_buf[length] = '\0';
	    printk("%s,", serial_buf);
            cir_bytes_written += CIR_CHUNK_WRITE_SIZE;
        }
    } while ( ( (int)size - cir_bytes_written ) > 0);

    length = snprintf(serial_buf, sizeof(serial_buf), "\"}\n");

    printk("%s", serial_buf);

    end_ts = k_cycle_get_32();

    // sys_clock_hw_cycles_per_sec()
    LOG_WRN("CIR output took %d ms", ( (end_ts - start_ts) * 1000 ) / sys_clock_hw_cycles_per_sec());

    return 0;
}

static void print_node_information() {
    printk("{\"event\": \"node_info\", \"node_id\": \"0x%04hx\", \"node_ranging_id\": %u}\n", get_own_node_id(), node_ranging_id);
}

void ranging_work_handler(uint64_t rtc_event_time, void *user_data)
{
    struct sstwr_ranging_conf *ranging_conf = (struct sstwr_ranging_conf *) user_data;
    int tx_slot = DWT_NO_TX_SLOT;
    uint8_t slot_node_assignment[NUM_NODES];

    SET_GPIO_HIGH(0);
    SET_GPIO_LOW(0);

#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->start(ieee802154_dev);
#endif

    //memset(slot_node_assignment, 0, sizeof(slot_node_assignment));
    for(int i = 0; i < NUM_NODES; i++){
        slot_node_assignment[i] = i; // we assign slot i to node i for now and then we shuffle them. // TODO is the ranging ID starting from 0 or 1?!
    }

    seeded_hash_permute_until_index(slot_node_assignment,
        sizeof(slot_node_assignment)/sizeof(slot_node_assignment[0]),
        ranging_conf->slots, (uint64_t) rtc_event_time);

    for (int k = 0; k < MIN(ranging_conf->slots, NUM_NODES); k++) {

        if (slot_node_assignment[k] == node_ranging_id) {
            tx_slot = k;
        }
    }

    struct dwt_ranging_frame_info *frame_infos;
    struct mtm_ranging_config conf = {
	    .slots_per_phase = ranging_conf->slots,
	    .ranging_id = node_ranging_id,
	    .phases = 2,
	    .slot_duration_us = ranging_conf->slot_duration_us,
	    .tx_slot_offset = tx_slot,
	    .cfo = 1,
            .cir_handler = output_cir_rtt,
    };

    if (!dwt_mtm_ranging(ieee802154_dev, &conf, &frame_infos)) {
        static char round_info[100];
        snprintf(round_info, sizeof(round_info), "\"slot_dur_us\": %u, \"asn\": %d", conf.slot_duration_us, ranging_conf->asn);
        output_frame_timestamps(frame_infos, &conf, rtc_event_time, round_info);
    }

#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->stop(ieee802154_dev);
#endif
}

// TODO our time synchronization layer does not copy the user supplied argument,
static struct glossy_conf glossy_conf;
static struct sstwr_ranging_conf ranging_conf;

static struct experiment_configuration {
    uint32_t scheduler_slot_duration_ms, dense_ranging_slot_duration_us;
} exp_conf = {
	.scheduler_slot_duration_ms = 2400,
	.dense_ranging_slot_duration_us = 200000, // Outputting ACC MEM data takes a long time
};


void schedule_network_next_event()
{
    int asn = slotted_schedule_get_asn();

    glossy_conf.ieee802154_dev = ieee802154_dev;
    glossy_conf.node_addr = node_ranging_id;

    // we will spent the first 2 minutes syncing
    if (asn >= 0) {
        current_asn = asn;

        if (!(asn % 4)) {
            slotted_schedule_work_next_slot(glossy_handler, &glossy_conf, schedule_network_next_event);
	} else {
	    ranging_conf.slots = 4;
            ranging_conf.asn = asn;
            ranging_conf.slot_duration_us = exp_conf.dense_ranging_slot_duration_us;

            slotted_schedule_work_next_slot(ranging_work_handler, &ranging_conf, schedule_network_next_event);
        }
    } else {
	// directly perform glossy
	glossy_handler(0, &glossy_conf);
        schedule_network_next_event();
    }
}

int main(void) {
    int ret = 0;
    LOG_INF("Starting ...");
    LOG_INF("Getting node id");

    int16_t signed_node_id = get_node_number(get_own_node_id());
    node_ranging_id = signed_node_id;

    setup_debug_gpios();

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

    radio_api = (struct ieee802154_radio_api *) ieee802154_dev->api;

    LOG_INF("GOT node id: %u", node_ranging_id);

    LOG_INF("Start IEEE 802.15.4 device");
    ret = radio_api->start(ieee802154_dev);

    if(ret) {
        LOG_ERR("Could not start ieee 802.15.4 device");
        return false;
    }

    time_sync_init(node_ranging_id == CONFIG_GLOSSY_TX_FLOOD_START_NODE_ID,
		   exp_conf.scheduler_slot_duration_ms);

    schedule_network_next_event();

    while (1) {
        k_msleep(50000);
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
