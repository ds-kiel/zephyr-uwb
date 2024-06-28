void timesync_check_gpio_handler(struct k_work *work) {
    SET_GPIO_HIGH(0);
    SET_GPIO_LOW(0);

    /* schedule_network_next_event(); */
}
    // Convert back into local time frame
    if (state.is_timesync_root) {
        next_event_delay = K_TIMEOUT_ABS_TICKS(ref_slot_start_ts);
    } else {
        timeutil_sync_local_from_ref(&state.rtc_clock_sync_state, ref_slot_start_ts, &local_slot_start_ts);
        next_event_delay = K_TIMEOUT_ABS_TICKS(local_slot_start_ts);
    }
