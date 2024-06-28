#ifndef LOG_H
#define LOG_H

#include <stdint.h>
#include <zephyr/drivers/ieee802154/dw1000.h>

void uart_out(char* msg);
void uart_disabled(char *msg);
void output_measurements_rtt(const struct measurement *g_measurements, int tof_count,
			     uint64_t rtc_slot_ts, const char *type);
void output_frame_timestamps(const struct dwt_ranging_frame_info *frame_infos,
    const struct mtm_ranging_config *conf, uint64_t rtc_slot_ts, const char *additional);

void log_out(char* msg);
void log_flush();
#endif
