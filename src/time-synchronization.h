#ifndef TIME_SYNCHRONIZATION_H
#define TIME_SYNCHRONIZATION_H

#include <stdint.h>
#include <zephyr/kernel.h>

#define ENOTIMEBASE 150

// create type alias for function for passing to schedule work at taking a user supplied pointer for additional arguments
typedef void (*time_sync_event_handler_t)(uint64_t event_time, void *user_data);
// add also type for callback after work has finished
typedef void (*time_sync_event_finished_callback_t)();

void time_sync_init(uint8_t is_timesync_root, uint32_t round_length_ms);

// big TODO i am not happy at all yet with how the schedule next events
int slotted_schedule_work_next_slot(time_sync_event_handler_t event_handler,
    void *user_data, time_sync_event_finished_callback_t work_callback);
int schedule_work_at(uint64_t ref_event_start_ts, time_sync_event_handler_t event_handler,
		     void *user_data, time_sync_event_finished_callback_t work_callback);

int slotted_schedule_get_asn();

struct glossy_conf {
    const struct device *ieee802154_dev;
    uint8_t node_addr;
};

void glossy_handler(uint64_t event_time, void *glossy_conf);

#endif
