#include <zephyr/logging/log.h>
#include <zephyr/zephyr.h>

#include <zephyr/devicetree.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/drivers/ieee802154/dw1000.h>
#include <stdio.h>


#include "nodes.h"
#include "log.h"
#include "history.h"

LOG_MODULE_REGISTER(main);

#define LOG_FLUSH_DIRECTLY 0

#define NUM_ROUNDS (1200)

#define INITIAL_DELAY_MS 5000
#define SLOT_DUR_UUS 2000

 // only relevant if we are earlier
#define TX_BUFFER_DELAY_UUS 1200
 // TODO default value is 500
#define TX_INVOKE_MIN_DELAY_UUS 800

// This delays has to be below 17/2 s, logging of one slot message takes at least 4ms
#define PRE_ROUND_DELAY_UUS 1000000

#if PRE_ROUND_DELAY_UUS > 16000000/2
#error "PRE_ROUND_DELAY_UUS is too large!"
#endif

// we wait for 20ms per slot after the round is finished to ensure that all logs are flushed before we start the next round
#define FLUSH_MS_PER_SLOT 20

// We need this delay to ensure that late packets do not destroy our schedule
// this might happen if the delay is very long, causing different sleep patters
#define POST_ROUND_DELAY_UUS 2000000
#define DWT_TS_MASK (0xFFFFFFFFFF)

// Debug values
//#define SLOT_DUR_UUS 2000000
//#define TX_BUFFER_DELAY_UUS 500000

#define EXP_TWR 0
#define EXP_NOISE 1
#define EXP_RESP_DELAYS 5
#define EXP_POWER_STATES 10
#define EXP_PING_PONG 20

#define CURRENT_EXPERIMENT EXP_RESP_DELAYS


//#define DB_START(NAME) uint64_t dbts_start_##NAME = dwt_system_ts(ieee802154_dev);
//#define DB_END(NAME) uint64_t dbts_end_##NAME = dwt_system_ts(ieee802154_dev);
//#define DB_LOG(NAME) LOG_INF("TIMING " ## NAME ## "%lld us (start ts %llu, end ts %llu)", (int64_t)(DWT_TS_TO_US((dbts_##END-dbts_##START)&DWT_TS_MASK))-45, dbts_start_##NAME, dbts_end_##NAME); /* -45 because of the inherent 45 ms delay in the dwt_system_ts call */

#define SW_START(NAME) int64_t dbts_start_##NAME = k_ticks_to_us_near64(k_uptime_ticks());
#define SW_END(NAME) int64_t dbts_end_##NAME = k_ticks_to_us_near64(k_uptime_ticks());
#define SW_LOG(NAME) LOG_INF("TIMING " #NAME " %lld us (start ts %lld, end ts %lld)", dbts_end_##NAME - dbts_start_##NAME, dbts_start_##NAME, dbts_end_##NAME);

// Disable stopwatch for now
#define SW_START(NAME)
#define SW_END(NAME)
#define SW_LOG(NAME)

#if CURRENT_EXPERIMENT == EXP_TWR
    #define SLOTS_PER_EXCHANGE 3
    #define NUM_SLOTS (NUM_NODES*(NUM_NODES-1)*SLOTS_PER_EXCHANGE)

#elif CURRENT_EXPERIMENT == EXP_NOISE
    #define SLOTS_PER_NODE 100
    #define NUM_SLOTS (NUM_NODES*SLOTS_PER_NODE)
    // make sure that the history has enough space to hold all of this!!
#elif CURRENT_EXPERIMENT == EXP_RESP_DELAYS
    #define SLOTS_PER_EXCHANGE 3
    #define EXP_RESP_DELAYS_INITIATOR 3
    #define EXP_RESP_DELAYS_RESPONDER 2
    //#define NUM_SLOTS (NUM_NODES*(NUM_NODES-1)*SLOTS_PER_EXCHANGE)
    // we only schedule 3 nodes for now due to long resp delays

    int64_t exp_delays[][2] =  {
        {2, 98},
        {4, 96},
        {6, 94},
        {8, 92},
        {10, 90},
        {12, 88},
        {14, 86},
        {16, 84},
        {18, 82},
        {20, 80},
        {22, 78},
        {24, 76},
        {26, 74},
        {28, 72},
        {30, 70},
        {32, 68},
        {34, 66},
        {36, 64},
        {38, 62},
        {40, 60},
        {42, 58},
        {44, 56},
        {46, 54},
        {48, 52},
        {50, 50},
        {52, 48},
        {54, 46},
        {56, 44},
        {58, 42},
        {60, 40},
        {62, 38},
        {64, 36},
        {66, 34},
        {68, 32},
        {70, 30},
        {72, 28},
        {74, 26},
        {76, 24},
        {78, 22},
        {80, 20},
        {82, 18},
        {84, 16},
        {86, 14},
        {88, 12},
        {90, 10},
        {92, 8},
        {94, 6},
        {96, 4},
        {98, 2},
    };

    #define NUM_SLOTS ((SLOTS_PER_EXCHANGE*(sizeof(exp_delays)/sizeof(exp_delays[0])))+1)
    #define RESP_DELAY_BASE_SLOT_DUR_UUS 400


#elif CURRENT_EXPERIMENT == EXP_POWER_STATES
#define NUM_SLOTS 0

#elif CURRENT_EXPERIMENT == EXP_PING_PONG

#define PING_PONG_INITIATOR 3
#define PING_PONG_SLOTS (200+1)

// MAKE SURE THAT THE HISTORY IS BIG ENOUGH TO HOLD ALL OF THIS ;)
#define NUM_SLOTS (PING_PONG_SLOTS*(NUM_NODES-1))

#endif


#define LOG_SCHEDULING 0

#define ROUND_DUR_US (NUM_SLOTS*SLOT_DUR_US+POST_ROUND_DELAY_US)


#define INITIAL_DELAY_US (5000000)

#if LOG_FLUSH_DIRECTLY
#define log_out uart_out
#else
#define log_out log_out
#endif


/* ieee802.15.4 device */
static struct ieee802154_radio_api *radio_api;
static const struct device *ieee802154_dev;

// We keep track of time in terms of dwt timestamps (which in theory should be pretty good ignoring the relative clock drift)
static uint64_t round_start_dwt_ts = 0;
static uint64_t last_round_start_dwt_ts = 0;

// measurement
// 40 bit measurements
// we save each measurement

// TODO: This is not standard compliant
static uint8_t msg_header[] = {0xDE, 0xCA};

static uint16_t own_number = 0;
static uint32_t cur_round = 0;
static uint32_t upcoming_slot = 0;

// As we are logging everything, we do not need to actually send timestamps here.
struct __attribute__((__packed__)) msg {
    uint8_t number; // the tx sender number
    uint32_t round; // The round is always started by the first node
    uint32_t slot; // The current slot id, there are slots equal to the number of PAIRS in the system, i.e. n squared
};

static struct msg msg_tx_buf;

// Rounds are always started by the first node from which point on all other slots are being synchronized.


#define INITIATOR_ID 0
#define IS_INITIATOR (own_number == INITIATOR_ID)


#define SLOT_IDLE -1
#define SLOT_LOG_FLUSH -2


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

// this function blocks until the wanted ts is almost reached, i.e. it determines the number of us between the wanted and the current ts and blocks for that duration (also adding a correction value)
// Note that it is not precise since we rely on the CPU cycles, yet, we just use it for rough syncs with the dwt clock
// A small correction factor for the whole function execution


// This function execution on its own has an overhead of roughly 83 us. We add a bit of buffer time to be kind of sure to schedule stuff correctly
#define DWT_BUSY_WAIT_DWT_CORRECTION_US (100)

//WARNING: This prevents rx events I think...
static void busy_wait_until_dwt_ts(uint64_t wanted_ts) {
    uint64_t init_ts = dwt_system_ts(ieee802154_dev);
    uint64_t diff = DWT_TS_TO_US(((uint64_t)(wanted_ts-init_ts))&DWT_TS_MASK); // This should wrap around nicely

    // We check that our wait call might not delay us too much
    if (diff > DWT_BUSY_WAIT_DWT_CORRECTION_US && diff < DWT_TS_TO_US(DWT_TS_MASK/2)) {
        k_busy_wait(diff - DWT_BUSY_WAIT_DWT_CORRECTION_US);
    }
}
// This function execution on its own has an overhead of roughly 83 us. We add a bit of buffer time to be kind of sure to schedule stuff correctly
#define DWT_SLEEP_DWT_CORRECTION_US (100+50)
static void sleep_until_dwt_ts(uint64_t wanted_ts) {
    uint64_t init_ts = dwt_system_ts(ieee802154_dev);
    uint64_t diff = DWT_TS_TO_US(((uint64_t)(wanted_ts-init_ts))&DWT_TS_MASK); // This should wrap around nicely

    // We check that our sleep call might not delay us too much
    if (diff > DWT_SLEEP_DWT_CORRECTION_US && diff < DWT_TS_TO_US(DWT_TS_MASK/2)) {
        k_usleep(diff - DWT_SLEEP_DWT_CORRECTION_US);
    }
}

K_SEM_DEFINE(round_start_sem, 0, 1);

#if CURRENT_EXPERIMENT == EXP_TWR
#define EXPERIMENT_NAME "twr"

uint64_t schedule_get_slot_duration_dwt_ts(uint16_t r, uint16_t slot) {
    return UUS_TO_DWT_TS(SLOT_DUR_UUS);
}

int8_t schedule_get_tx_node_number(uint32_t r, uint32_t slot) {

    uint16_t exchange = slot / 3;
    uint8_t m = slot % 3;

    uint16_t init = exchange / (NUM_NODES - 1);
    uint16_t resp = exchange % (NUM_NODES - 1);

    if(init <= resp) {
        resp = (resp+1) % NUM_NODES; // we do not want to execute a ranging with ourselves..., actually modulo should not be necessary here anyway?
    }

    if (m == 0 || m == 2) {
        return init;
    } else if(m == 1) {
        return resp;
    }

    return -1;
}

#elif CURRENT_EXPERIMENT == EXP_NOISE
#define EXPERIMENT_NAME "noise"

uint64_t schedule_get_slot_duration_dwt_ts(uint16_t r, uint16_t slot) {
    return UUS_TO_DWT_TS(SLOT_DUR_UUS);
}
int8_t schedule_get_tx_node_number(uint32_t r, uint32_t slot) {
    return slot/SLOTS_PER_NODE;
}

#elif CURRENT_EXPERIMENT == EXP_RESP_DELAYS
#define EXPERIMENT_NAME "resp_delays"
uint64_t schedule_get_slot_duration_dwt_ts(uint16_t r, uint16_t slot) {

    if (slot == 0) {
        return UUS_TO_DWT_TS(SLOT_DUR_UUS*2);
    }

    slot -= 1;

    uint8_t exp = (slot/3) % (sizeof(exp_delays)/sizeof(exp_delays[0]));

    int64_t delay_b_multiplier = exp_delays[exp][0];
    int64_t delay_a_multiplier = exp_delays[exp][1];

    uint8_t m = slot % 3;
    if (m == 0) {
        return UUS_TO_DWT_TS(RESP_DELAY_BASE_SLOT_DUR_UUS*delay_b_multiplier);
    } else if(m == 1) {
        return UUS_TO_DWT_TS(RESP_DELAY_BASE_SLOT_DUR_UUS*delay_a_multiplier);
    } else {
        return UUS_TO_DWT_TS(SLOT_DUR_UUS); // we use a bit more delay to compensate for clock drifts in the last slot
    }
}

int8_t schedule_get_tx_node_number(uint32_t r, uint32_t slot) {
    if (slot == 0) {
        return EXP_RESP_DELAYS_INITIATOR;
    }
    slot -= 1;

    uint8_t m = slot % 3;

    if (m == 0 || m == 2) {
        return EXP_RESP_DELAYS_INITIATOR;
    } else if(m == 1) {
        return EXP_RESP_DELAYS_RESPONDER;
    }

    return -1;
}

#elif CURRENT_EXPERIMENT == EXP_PING_PONG
#define EXPERIMENT_NAME "ping_pong"

uint64_t schedule_get_slot_duration_dwt_ts(uint16_t r, uint16_t slot) {
    return UUS_TO_DWT_TS(SLOT_DUR_UUS); // we use the normal slot duration
}

int8_t schedule_get_tx_node_number(uint32_t r, uint32_t slot) {

    uint16_t init = PING_PONG_INITIATOR;
    uint16_t resp = slot / PING_PONG_SLOTS;

    if(init <= resp) {
        resp = (resp+1) % NUM_NODES; // we do not want to execute a ranging with ourselves..., actually modulo should not be necessary here anyway?
    }

    if (slot % 2 == 0) {
        return init;
    } else {
        return resp;
    }
}

#endif


int main(void) {

    //LOG_INF("Testing ...");
    //matrix_test();
    //return;

    int ret = 0;
    LOG_INF("Starting ...");


    LOG_INF("Getting node id");
    int16_t signed_node_id = get_node_number(get_own_node_id());

    if (signed_node_id < 0) {
        LOG_WRN("Node number NOT FOUND! Shutting down :( I am: 0x%04hx", get_own_node_id());
        return 0;
    }

    own_number = signed_node_id;

    LOG_INF("Initialize ieee802.15.4");
    ieee802154_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_ieee802154));

    if (!ieee802154_dev) {
        LOG_ERR("Cannot get ieee 802.15.4 device");
        return false;
    }

    // prepare msg buffer
    {
        (void)memset(&msg_tx_buf, 0, sizeof(msg_tx_buf));
        msg_tx_buf.number = own_number&0xFF;
        msg_tx_buf.round = 0;
    }
    /* Setup antenna delay values to 0 to get raw tx values */
    uint32_t opt_delay_both = dwt_otp_antenna_delay(ieee802154_dev);
    uint16_t rx_delay = opt_delay_both&0xFFFF; // we fully split the delay
    uint16_t tx_delay = opt_delay_both&0xFFFF; // we fully split the delay

    {
       char buf[512];
       snprintf(buf, sizeof(buf), "{\"event\": \"init\", \"own_number\": %hhu, \"rx_delay\": %hu, \"tx_delay\": %hu, \"experiment\": \"%s\"}\n", own_number, rx_delay, tx_delay, EXPERIMENT_NAME);
       log_out(buf); // this will be flushed later!
       log_flush();
    }


    //dwt_set_antenna_delay_rx(ieee802154_dev, 16450);
    //dwt_set_antenna_delay_tx(ieee802154_dev, 16450);

    // we disable the frame filter, otherwise the packets are not received!
    dwt_set_frame_filter(ieee802154_dev, 0, 0);
    radio_api = (struct ieee802154_radio_api *)ieee802154_dev->api;


    #if CURRENT_EXPERIMENT == EXP_POWER_STATES
        while(1) {

            k_sleep(K_MSEC(1000));
            LOG_INF("Start IEEE 802.15.4 device");
            ret = radio_api->start(ieee802154_dev);
            if(ret) {
                LOG_ERR("Could not start ieee 802.15.4 device");
            }
            k_sleep(K_MSEC(1000));
//            for(int i = 0; i < 1000; i++) {
//                k_busy_wait(1000);
//                k_yield();
//            }
            LOG_INF("Stop IEEE 802.15.4 device");
            radio_api->stop(ieee802154_dev);
        };
        return;
    #endif

    LOG_INF("GOT node id: %hhu", own_number);

    LOG_INF("Start IEEE 802.15.4 device");
    ret = radio_api->start(ieee802154_dev);

    if(ret) {
        LOG_ERR("Could not start ieee 802.15.4 device");
        return false;
    }

    // Sleep in DWT time to have enough time before the round starts.
    // TODO: do not sleep right now as we have enough pre round delay atm!
    //sleep_until_dwt_ts(dwt_system_ts(ieee802154_dev)+UUS_TO_DWT_TS(INITIAL_DELAY_MS*1000) & DWT_TS_MASK);

     // Timing test
     {
        for (int i = 0; i < 10; i++) {
            uint64_t ts1 = dwt_system_ts(ieee802154_dev);
            uint64_t ts2 = dwt_system_ts(ieee802154_dev);
            int64_t diff_us = DWT_TS_TO_US((ts2-ts1)&DWT_TS_MASK);
            LOG_INF("dwt_system_ts calls ts1  %llu, ts2: %llu, diff us %lld", ts1, ts2, diff_us);
        }
    }

//    {
//        uint64_t init_ts = dwt_system_ts(ieee802154_dev);
//        uint64_t wanted_ts = init_ts + UUS_TO_DWT_TS(100);
//        sleep_until_dwt_ts(wanted_ts);
//        uint64_t other_ts = dwt_system_ts(ieee802154_dev);
//
//        int64_t diff = (int64_t)other_ts - (int64_t)wanted_ts;
//        int64_t diff_us = DWT_TS_TO_US(diff);
//        LOG_INF("Blocking DWT TS initial  %llu, wanted: %llu, actual: %llu, diff %lld, diff us %lld", init_ts, wanted_ts, other_ts, diff, diff_us);
//
//        init_ts = dwt_system_ts(ieee802154_dev);
//        wanted_ts = init_ts + UUS_TO_DWT_TS(TX_BUFFER_DELAY_UUS);
//        sleep_until_dwt_ts(wanted_ts);
//        other_ts = dwt_system_ts(ieee802154_dev);
//
//        diff = (int64_t)other_ts - (int64_t)wanted_ts;
//        diff_us = DWT_TS_TO_US(diff);
//        LOG_INF("SLEEPING DWT TS initial  %llu, wanted: %llu, actual: %llu, diff %lld, diff us %lld", init_ts, wanted_ts, other_ts, diff, diff_us);
//    }


    uint16_t antenna_delay = dwt_antenna_delay_tx(ieee802154_dev);

    while(cur_round < NUM_ROUNDS) { // ----- Round Logic

        uint64_t actual_round_start = dwt_system_ts(ieee802154_dev);

        // calculate which node is responsible for the next slot
        if (own_number == schedule_get_tx_node_number(cur_round, upcoming_slot)) {
            round_start_dwt_ts = (dwt_system_ts(ieee802154_dev) + MAX(UUS_TO_DWT_TS((uint64_t)TX_BUFFER_DELAY_UUS), UUS_TO_DWT_TS(PRE_ROUND_DELAY_UUS))) & DWT_TS_MASK; // we are the first to transmit in this round!
            k_sem_give(&round_start_sem);
        }

        k_sem_take(&round_start_sem, K_FOREVER);

        uint64_t upcoming_slot_tx_ts = round_start_dwt_ts;

        if (upcoming_slot > 0) { // next slot is set in the rx handler!
            // seems like we start not on the first slot (happens when we are not initializing the round!)
            upcoming_slot_tx_ts = (upcoming_slot_tx_ts + schedule_get_slot_duration_dwt_ts(cur_round, upcoming_slot-1)) & DWT_TS_MASK;
        }

        while(upcoming_slot < NUM_SLOTS) { // ----- Slot Logic
            uint64_t upcoming_slot_dur_ts = schedule_get_slot_duration_dwt_ts(cur_round, upcoming_slot);
            int16_t  slot_tx_id           = schedule_get_tx_node_number(cur_round, upcoming_slot);

            // sleep until slot start
            sleep_until_dwt_ts(((uint64_t)upcoming_slot_tx_ts-(uint64_t)UUS_TO_DWT_TS(TX_BUFFER_DELAY_UUS))& DWT_TS_MASK);

            if (own_number == slot_tx_id) { // check whether this slot is for us
                k_thread_priority_set(k_current_get(), K_HIGHEST_THREAD_PRIO); // we are a bit time sensitive from here on now ;)

                struct net_pkt *pkt = NULL;
                struct net_buf *buf = NULL;
                size_t len = sizeof (msg_header)+sizeof(msg_tx_buf);

                /* Maximum 2 bytes are added to the len */
                while(pkt == NULL) {
                    pkt = net_pkt_alloc_with_buffer(NULL, len, AF_UNSPEC, 0, K_MSEC(100));//K_NO_WAIT);
                    if (!pkt) {
                        LOG_WRN("COULD NOT ALLOCATE MEMORY FOR PACKET!");
                    }
                }

                buf = net_buf_frag_last(pkt->buffer);
                len = net_pkt_get_len(pkt);

                struct net_ptp_time ts;
                ts.second = 0;
                ts.nanosecond = 0;

                net_pkt_set_timestamp(pkt, &ts);

                net_pkt_write(pkt, msg_header, sizeof(msg_header));

                // update current values
                msg_tx_buf.round = cur_round;
                msg_tx_buf.slot = upcoming_slot;

                // all other entries are updated in the rx event!
                net_pkt_write(pkt, &msg_tx_buf, sizeof(msg_tx_buf));

                uint32_t planned_tx_short_ts = upcoming_slot_tx_ts >> 8;
                dwt_set_delayed_tx_short_ts(ieee802154_dev, planned_tx_short_ts);

                uint64_t tx_invoke_ts = dwt_system_ts(ieee802154_dev);

                // Check that we are not overflowing, i.e. that we are not too late with the tx
                // invocation here! (otherwise we will get a warning which destroys all of our
                // timing...)
                if ((uint64_t)(upcoming_slot_tx_ts-(tx_invoke_ts+DWT_TS_TO_US(TX_INVOKE_MIN_DELAY_UUS))) < DWT_TS_MASK/2) {
                    ret = radio_api->tx(ieee802154_dev, IEEE802154_TX_MODE_TXTIME, pkt, buf);
                }

                // WE NEED COOP PRIORITY otherwise we are verryb likely to miss our tx window
                k_thread_priority_set(k_current_get(), K_HIGHEST_APPLICATION_THREAD_PRIO); // we are less time sensitive from here on now ;)
                net_pkt_unref(pkt);

                uint64_t actual_tx_ts = (((uint64_t)(planned_tx_short_ts & 0xFFFFFFFEUL)) << 8) + antenna_delay;

                // Note that the actual tx time is not the planned one for the slot!
                if (history_save_tx(own_number, cur_round, upcoming_slot, actual_tx_ts)) {
                    LOG_WRN("Could not save TX to history");
                }
            }

            // we are already in the next slot, set the next slot tx timestamp accordingly
            upcoming_slot_tx_ts = (upcoming_slot_tx_ts + upcoming_slot_dur_ts) & DWT_TS_MASK;
            upcoming_slot++;
        } // ----- Current round stops here

        // we sleep here to the end of the last slot, so that we do not receive a message that overrides anything, especially not our round_started sem!!
        {
            uint64_t cur_ts = dwt_system_ts(ieee802154_dev);
            uint64_t wanted_ts = cur_ts + UUS_TO_DWT_TS(POST_ROUND_DELAY_UUS);
            sleep_until_dwt_ts(wanted_ts & DWT_TS_MASK);
        }
        if (LOG_SCHEDULING) {
            char buf[512];
            // TODO: it is quite possible that this logging breaks the start of the round already!!!
            snprintf(buf, sizeof(buf), "{\"event\": \"round_end\", \"own_number\": %hhu, \"round\": %u, \"round_start_us\": %llu, \"cur_us\": %llu, \"actual_round_start_us\": %llu}\n", own_number, cur_round, DWT_TS_TO_US(round_start_dwt_ts), DWT_TS_TO_US(dwt_system_ts(ieee802154_dev)), DWT_TS_TO_US(actual_round_start));
            log_out(buf);
        }

        last_round_start_dwt_ts = round_start_dwt_ts;
        upcoming_slot = 0; // we restart the round
        round_start_dwt_ts = 0;
        cur_round++;

        // After every round, we flush all of our logs
        //uint64_t before_flush_ts = dwt_system_ts(ieee802154_dev);

        int64_t before_flush_ms = k_uptime_get();

        size_t log_count = history_count();
        history_print();
        history_reset();

        int64_t after_flush_ms = k_uptime_get();
        int64_t wanted_dur_ms = FLUSH_MS_PER_SLOT * NUM_SLOTS;
        int64_t actual_dur_ms = k_uptime_get() - before_flush_ms;

        LOG_INF("Flushing count %d, wanted ms %lld, actual ms %lld", log_count, wanted_dur_ms, actual_dur_ms);
        if (actual_dur_ms < wanted_dur_ms) {
            k_sleep(K_MSEC(wanted_dur_ms - actual_dur_ms));
        } else {
            LOG_WRN("WARNING: Flushing took too long!");
        }
        //sleep_until_dwt_ts(before_flush_us + UUS_TO_DWT_TS(POST_ROUND_DELAY_UUS));
    }

    return 0;
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
    size_t len = net_pkt_get_len(pkt);
    struct net_buf *buf = pkt->buffer;
    int ret = 0;
    //LOG_WRN("Got data of length %d", len);

    if (len > sizeof(msg_header) + 2 && !memcmp(msg_header, net_buf_pull_mem(buf, sizeof(msg_header)), sizeof(msg_header))) {
        len -= sizeof(msg_header) + 2; // 2 bytes crc?
        struct msg *rx_msg = net_buf_pull_mem(buf, len);

        // TODO: Use these?
        net_buf_pull_u8(buf);
        net_buf_pull_u8(buf);

        if (len >  0 && len % sizeof (struct msg)  == 0) {
            //size_t num_msg = len / sizeof (struct msg_ts);

            // TODO: Check that rx_number is actually valid... -> buffer overflow!
            // TODO: CHECK IF WE would need to ignore this msg
            uint8_t rx_number = rx_msg->number;
            uint16_t rx_round = rx_msg->round;
            uint16_t rx_slot = rx_msg->slot;

            //LOG_DBG("Received (n: %hhu, r: %hu)", rx_number, rx_round);

            uint64_t rx_ts = dwt_rx_ts(ieee802154_dev);
            int carrierintegrator = dwt_readcarrierintegrator(ieee802154_dev);
            int8_t rssi = (int8_t)net_pkt_ieee802154_rssi(pkt);
            int8_t bias_correction = get_range_bias_by_rssi(rssi);
            uint64_t bias_corrected_rx_ts = rx_ts - bias_correction;
            uint8_t rx_ttcko_rc_phase = dwt_rx_ttcko_rc_phase(ieee802154_dev);

            // Log the message!
            {
                if (history_save_rx(own_number, rx_number, rx_round, rx_slot, rx_ts, carrierintegrator, rssi, bias_correction, bias_corrected_rx_ts, rx_ttcko_rc_phase)) {
                    LOG_WRN("Could not save RX to history");
                }
            }

            if (upcoming_slot == 0 && rx_round >= cur_round) {
                cur_round = rx_round; // we might have missed a round! (or this is just the start of something new?)
                upcoming_slot = rx_slot+1; // this should always be > 0 so we do not execute this thing twice...
                round_start_dwt_ts = rx_ts; // TODO: we neglect any airtime and other delays at this point but it should be "good enough"
                k_sem_give(&round_start_sem);
            }

            //LOG_INF("RX Event");

        } else {
            LOG_WRN("Got weird data of length %d", len);
        }
    } else {
        LOG_WRN("Got WRONG data, pkt %p, len %d", pkt, len);
    }

    net_pkt_unref(pkt);

    return ret;
}