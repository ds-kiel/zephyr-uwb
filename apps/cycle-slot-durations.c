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

#define SLOT_LENGTH_TIMESTAMP_MS 30

LOG_MODULE_REGISTER(main);

struct sstwr_ranging_conf {
    uint8_t slots;
    uint32_t slot_duration_us;
};

static struct ieee802154_radio_api *radio_api;
static const struct device *ieee802154_dev;
static uint8_t node_ranging_id;

static void print_node_information() {
    printk("{\"event\": \"node_info\", \"node_id\": \"0x%04hx\", \"node_ranging_id\": %u}\n", get_own_node_id(), node_ranging_id);
}

void ranging_mtm_sstwr_cfo_work_handler(uint64_t event_time, void *user_data)
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
        ranging_conf->slots, (uint64_t) event_time);

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
    };

    if (!dwt_mtm_ranging(ieee802154_dev, &conf, &frame_infos)) {
#if CONFIG_DWT_MTM_ON_DEVICE_PROCESSING
        SET_GPIO_HIGH(1);
	int template_cnt = extract_round_templates(frame_infos, &conf, EXTRACT_ALL);
        int freq_cnt = frequencies_from_cfos(frame_infos, conf.slots_per_phase, conf.phases, g_offsets);
        int meas_cnt = calculate_measurements(g_templates, template_cnt, g_offsets, freq_cnt, g_measurements);

        output_measurements_rtt(g_measurements, meas_cnt, next_event_time, "ss-hash"); // OUTPUT_ASYNC_LOOP
        SET_GPIO_LOW(1);
#else
        output_frame_timestamps(frame_infos, &conf, event_time);
#endif
    }

#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->stop(ieee802154_dev);
#endif
}

// TODO our time synchronization layer does not copy the user supplied argument,
static struct glossy_conf glossy_conf;
static struct sstwr_ranging_conf ranging_conf;

static struct experiment_configuration {
    uint32_t initial_warmup_period_ms;
    uint32_t scheduler_slots_per_ranging_slot_duration;
    uint32_t scheduler_slot_duration_ms;
    uint32_t dense_ranging_slot_duration_us_begin, dense_ranging_slot_duration_us_end;
} exp_conf = {
	.initial_warmup_period_ms = 120000,
	.scheduler_slot_duration_ms = 50,
        .scheduler_slots_per_ranging_slot_duration = 1000,
        .dense_ranging_slot_duration_us_begin = 1000,
        .dense_ranging_slot_duration_us_end = 3000,
};


void schedule_network_next_event()
{
    int asn = slotted_schedule_get_asn();

    glossy_conf.ieee802154_dev = ieee802154_dev;
    glossy_conf.node_addr = node_ranging_id;

    // we will spent the first 2 minutes syncing

    int warmup_last_asn = exp_conf.initial_warmup_period_ms / exp_conf.scheduler_slot_duration_ms;

    if (asn >= 0) {
        if (!(asn % 10) || (asn < warmup_last_asn)) {

            slotted_schedule_work_next_slot(glossy_handler, &glossy_conf, schedule_network_next_event);
	} else {
            ranging_conf.slots = 10;
            ranging_conf.slot_duration_us = exp_conf.dense_ranging_slot_duration_us_begin;

            slotted_schedule_work_next_slot(ranging_mtm_sstwr_cfo_work_handler, &ranging_conf, schedule_network_next_event);
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

    time_sync_init(node_ranging_id == CONFIG_GLOSSY_TX_FLOOD_START_NODE_ID, exp_conf.scheduler_slot_duration_ms);

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
