#include "time-synchronization.h"
#include <zephyr/sys/timeutil.h>
#include <zephyr/drivers/ieee802154/dw1000.h>

#include "nodes.h"

struct  time_sync_work_item {
    struct k_work work;
    time_sync_event_handler_t event_handler;
    time_sync_event_finished_callback_t event_finished_callback;
    void *user_data;
};

struct timeutil_sync_config rtc_sync_conf = {
    .ref_Hz = CONFIG_SYS_CLOCK_TICKS_PER_SEC,
    .local_Hz = CONFIG_SYS_CLOCK_TICKS_PER_SEC
};

static struct time_sync_state {
    uint64_t next_event_time;
    uint32_t timesync_skew_update_count;
    uint8_t is_timesync_root;
    uint32_t systick_round_length;
    struct timeutil_sync_state rtc_clock_sync_state;

    struct time_sync_work_item queued_work_item;

} state = {
	.next_event_time = 0,
	.timesync_skew_update_count = 0,
	.is_timesync_root = 0,
	.systick_round_length = 0,
	.rtc_clock_sync_state = {.cfg = &rtc_sync_conf },
};

void time_sync_event_handler()
{
    k_work_submit(&state.queued_work_item.work);
}

K_TIMER_DEFINE(event_timer, time_sync_event_handler, NULL);

int get_current_reference_time(uint64_t *local_now, uint64_t *ref_now)
{
    uint64_t _local_now = k_uptime_ticks();

    if(local_now != NULL) {
        *local_now = _local_now;
    }

    if (state.is_timesync_root) {
        *ref_now = _local_now;
        return 0;
    }

    if (timeutil_sync_ref_from_local(&state.rtc_clock_sync_state, _local_now, ref_now) < 0) {
        return -ENOTIMEBASE;
    }

    return 0;
}

void time_sync_init(uint8_t is_timesync_root, uint32_t round_length_ms)
{
    state.is_timesync_root = is_timesync_root;
    state.timesync_skew_update_count = 0;

    // convert round_length_ms to ticks
    state.systick_round_length = round_length_ms * CONFIG_SYS_CLOCK_TICKS_PER_SEC / 1000;

    // reset base and latest data of time_util_sync_state
    state.rtc_clock_sync_state.base.local = 0;
    state.rtc_clock_sync_state.base.ref = 0;
    state.rtc_clock_sync_state.latest.local = 0;
    state.rtc_clock_sync_state.latest.ref = 0;
}

int slotted_schedule_work_next_slot(time_sync_event_handler_t event_handler,
    void *user_data, time_sync_event_finished_callback_t event_finished_callback)
{
    int ret;
    uint64_t ref_now, ref_slot_start_ts;

    if( (ret = get_current_reference_time(NULL, &ref_now)) < 0 ) {
        return ret;
    }

    ref_slot_start_ts = ref_now + (state.systick_round_length - ref_now % state.systick_round_length);

    return schedule_work_at(ref_slot_start_ts, event_handler, user_data, event_finished_callback);
}

void event_worker(struct k_work *work_item)
{
    struct time_sync_work_item *item = CONTAINER_OF(work_item, struct time_sync_work_item, work);

    item->event_handler(state.next_event_time, item->user_data);
    item->event_finished_callback();
}

// returns the current asn
int slotted_schedule_get_asn()
{
    int ret;
    uint64_t ref_now;
    if ((ret = get_current_reference_time(NULL, &ref_now)) < 0) {
        return ret;
    }

    return (ref_now / state.systick_round_length);
}

int schedule_work_at(uint64_t ref_event_start_ts, time_sync_event_handler_t event_handler,
    void *user_data, time_sync_event_finished_callback_t event_finished_callback)
{
    int ret;
    uint64_t ref_now, local_now, local_event_start_ts = 0;
    k_timeout_t next_event_delay;

    if( (ret = get_current_reference_time(&local_now, &ref_now)) < 0 ) {
	return ret;
    }

    state.next_event_time = ref_event_start_ts;

    // convert back into local time frame
    if (state.is_timesync_root) {
        next_event_delay = K_TIMEOUT_ABS_TICKS(ref_event_start_ts);
    } else {
        timeutil_sync_local_from_ref(&state.rtc_clock_sync_state, ref_event_start_ts, &local_event_start_ts);
        next_event_delay = K_TIMEOUT_ABS_TICKS(local_event_start_ts);
    }

    // print event telling us the event delay in MS
    printk("{\"event\": \"scheduler, \"delay[ms]\": %d}\n",
        (int)((local_event_start_ts - local_now) * 1000) / CONFIG_SYS_CLOCK_TICKS_PER_SEC);

    // create a work item for the passed event_handler, user_data and event_finished_callback
    state.queued_work_item.event_handler = event_handler;
    state.queued_work_item.event_finished_callback = event_finished_callback;
    state.queued_work_item.user_data = user_data;
    k_work_init(&state.queued_work_item.work, event_worker);

    k_timer_start(&event_timer, next_event_delay, K_NO_WAIT);

    return 0;
}

void glossy_handler(uint64_t event_time, void *user_data)
{
    struct dwt_glossy_tx_result sync_result;

    const struct glossy_conf *conf = (struct glossy_conf *) user_data;
    const struct device *ieee802154_dev = conf->ieee802154_dev;

#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->start(ieee802154_dev);
#endif
    printk("{\"event\": \"glossy\", \"node_id\": \"0x%04hx\"", get_own_node_id());

    // wait at at most 10 ms for glossy packet
    if( dwt_glossy_tx_timesync(ieee802154_dev, state.is_timesync_root, conf->node_addr, 0, &sync_result) >= 0 ) {
        if( timeutil_sync_state_update(&state.rtc_clock_sync_state, &sync_result.clock_sync_instant) ) {
            float skew;

            skew = timeutil_sync_estimate_skew(&state.rtc_clock_sync_state);
            timeutil_sync_state_set_skew(&state.rtc_clock_sync_state, skew, NULL);

	    printk(", \"rtc_offset\": %d, \"hops\": %d", timeutil_sync_skew_to_ppb(skew), sync_result.dist_to_root);

            state.timesync_skew_update_count++;
	}
    }

    printk("}\n");

#if CONFIG_RANGING_RADIO_SLEEP
    radio_api->stop(ieee802154_dev);
#endif
}
