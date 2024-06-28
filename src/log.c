#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "log.h"
#include "nodes.h"

#define LOG_BUF_SIZE (2048)

LOG_MODULE_REGISTER(logging);

#if LOG_FLUSH_DIRECTLY
#define log_out uart_out
#else
#define log_out log_out
#endif

static char serial_buf[1024]; // when TDOA is used has to be quite large, we pretty much use here most spare memory we have left over. Could off course easily be optimized

#define UART0_NODE DT_NODELABEL(uart0)
static const struct device *const uart_dev = DEVICE_DT_GET(UART0_NODE);

static const struct device* uart_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

K_SEM_DEFINE(log_buf_sem, 1, 1);

void uart_out(char* msg) {
    k_sem_take(&log_buf_sem, K_FOREVER);
    while (*msg != '\0') {
        uart_poll_out(uart_device, *msg);
        msg++;
    }
    k_sem_give(&log_buf_sem);
}

void uart_disabled(char* msg) {
    //NOOP
}


static char log_buf[LOG_BUF_SIZE] = {0};
static size_t log_buf_count = 0;


void log_out(char* msg) {
    k_sem_take(&log_buf_sem, K_FOREVER);

    while (*msg != '\0' && log_buf_count < LOG_BUF_SIZE) {
        log_buf[log_buf_count] = *msg;
        log_buf_count++;
        msg++;
    }

    k_sem_give(&log_buf_sem);

    if (log_buf_count == LOG_BUF_SIZE-1) {
        LOG_WRN("LOG IS FULL!!!");
    }
}

void log_flush() {
    k_sem_take(&log_buf_sem, K_FOREVER);

    for (size_t i = 0; i < log_buf_count; i++) {
        uart_poll_out(uart_device, log_buf[i]);
    }

    log_buf_count = 0;
    (void)memset(&log_buf, 0, sizeof(log_buf));

    k_sem_give(&log_buf_sem);
}


size_t log_get_count() {
    return log_buf_count;
}

/* static void uart_poll_serial_output_msg(char* msg) { */
/*     while (*msg != '\0') { */
/*         uart_poll_out(uart_dev, *msg); */
/*         msg++; */
/*     } */
/* } */

/* void uart_finished_callback(const struct device *dev, struct uart_event *evt, void *user_data) */
/* { */
/*     if(evt->type == UART_TX_DONE) { */
/*         /\* LOG_WRN("UART TX done"); *\/ */
/*     } else if(evt->type == UART_TX_ABORTED) { */
/*         LOG_WRN("UART TX aborted"); */
/*     } */

/*     k_sem_give(&uart_transfer_finished); */
/* } */

/* void output_measurements_uart(const struct measurement *g_measurements, */
/*     int tof_count, uint64_t rtc_slot_ts) { */
/*     int written = 0; */
/*     uint8_t insert_comma; */

/*     // Create output */
/*     written += snprintf(serial_buf, sizeof(serial_buf), "{\"event\":\"twr\",\"i\":%u,\"rtc_round_ts\": %llu,\"t\":[", */
/*         node_ranging_id, rtc_slot_ts); */

/*     insert_comma = 0; */
/*     for (int i = 0; i < tof_count; i++) { */
/*         if(g_measurements[i].ranging_initiator_id == node_ranging_id) { */
/*             uint8_t id = g_measurements[i].ranging_responder_id; */
/*             float dist = time_to_dist(g_measurements[i].tof) * DIST_OUTPUT_UNIT; */

/*             if (insert_comma) { */
/*                 written += snprintf(serial_buf + written, sizeof(serial_buf) - written, ","); */
/*             } */

/*             written += snprintf(serial_buf + written, sizeof(serial_buf) - written, */
/*                 "{\"i\":%u,\"d\":%d,\"q\":%d}", id, (int32_t)(dist), 100); */

/*             insert_comma = 1; */
/*         } */
/*     } */

/*     written += snprintf(serial_buf + written, sizeof(serial_buf) - written, "]}\r\n"); */

/*     uart_poll_serial_output_msg(serial_buf); */

/*     written = 0; */
/*     written += */
/*         snprintf(serial_buf, sizeof(serial_buf), */
/*             "{\"event\":\"td\",\"i\":%u,\"rtc_round_ts\": %llu,\"t\":[", node_ranging_id, rtc_slot_ts); */
/*     insert_comma = 0; */

/*     for (int i = 0; i < tof_count; i++) { */
/*         uint8_t initiator = g_measurements[i].ranging_responder_id, responder = g_measurements[i].ranging_initiator_id; */

/*         if(initiator != node_ranging_id && responder != node_ranging_id && g_measurements[i].tdoa != NAN) { */
/*             float dist = time_to_dist(g_measurements[i].tdoa) * DIST_OUTPUT_UNIT; */

/*             if (insert_comma) { */
/*                 written += snprintf(serial_buf + written, sizeof(serial_buf) - written, ","); */
/*             } */

/*             written += */
/*                 snprintf(serial_buf + written, sizeof(serial_buf) - written, */
/*                     "{\"i\":[%u,%u],\"d\":%d,\"q\":%d}", */
/*                     initiator, responder, (int32_t)(dist), 100); */
/*             insert_comma = 1; */
/*         } */
/*     } */

/*     written += snprintf(serial_buf + written, sizeof(serial_buf) - written, "]}\r\n"); */

/*     uart_poll_serial_output_msg(serial_buf); */
/* } */

#if CONFIG_DWT_MTM_ON_DEVICE_PROCESSING
void output_measurements_rtt(const struct measurement *g_measurements,
    int tof_count, uint64_t rtc_slot_ts, const char *type) {
    uint8_t insert_comma;

    printk("{\"event\":\"twr\", \"type\": \"%s\", \"i\":%u,\"rtc_round_ts\": %llu,\"t\":[",
        type, node_ranging_id, rtc_slot_ts);

    insert_comma = 0;
    for (int i = 0; i < tof_count; i++) {
        if(g_measurements[i].ranging_initiator_id == node_ranging_id) {
            uint8_t id = g_measurements[i].ranging_responder_id;
            float dist = time_to_dist(g_measurements[i].tof) * DIST_OUTPUT_UNIT;

            if (insert_comma) {
                printk(",");
            }

            printk("{\"i\":%u,\"d\":%d,\"q\":%d}", id, (int32_t)(dist), 100);

            insert_comma = 1;
        }
    }
    printk("]}\r\n");

    printk("{\"event\":\"td\", \"type\": \"%s\",\"i\":%u,\"rtc_round_ts\": %llu,\"t\":[", type, node_ranging_id, rtc_slot_ts);
    insert_comma = 0;

    for (int i = 0; i < tof_count; i++) {
        uint8_t initiator = g_measurements[i].ranging_responder_id, responder = g_measurements[i].ranging_initiator_id;

        if(initiator != node_ranging_id && responder != node_ranging_id && g_measurements[i].tdoa != NAN) {
            float dist = time_to_dist(g_measurements[i].tdoa) * DIST_OUTPUT_UNIT;

            if (insert_comma) {
                printk(",");
            }

            printk("{\"i\":[%u,%u],\"d\":%d,\"q\":%d}", initiator, responder, (int32_t)(dist), 100);
            insert_comma = 1;
        }
    }
    printk("]}\r\n");
}
#endif

void output_frame_timestamps(const struct dwt_ranging_frame_info *frame_infos,
    const struct mtm_ranging_config *conf, uint64_t rtc_slot_ts, const char *additional)
{
    for (int i = 0; i < conf->phases-1; i++) { // the last phase/phase is not of importance for us
        for(int j = 0; j < conf->slots_per_phase; j++) {
            const struct dwt_ranging_frame_info *curr_frame_info = &frame_infos[i*conf->slots_per_phase + j];

	    if (curr_frame_info->frame != NULL) {
                const struct dwt_ranging_frame_buffer *curr_frame = curr_frame_info->frame;

		if (curr_frame_info->type == DWT_RANGING_TRANSMITTED_FRAME) {
		    printk("{\"event\": \"tx\", \"own_id\": %u, \"rtc_round_ts\": %llu, \"phase\": "
			   "%u, \"slot\": %u, \"ts\": %llu",
                        conf->ranging_id, rtc_slot_ts, i, j, curr_frame_info->timestamp);
		} else {
		    printk("{\"event\": \"rx\", \"own_id\": %u, \"other_id\": %u,"
			   "\"rtc_round_ts\": %llu, \"phase\": %u, \"slot\": %u,"
			   "\"fp_ampl1\": %u, \"fp_ampl2\": %u, \"fp_ampl3\": %u,"
			   "\"fp_index\": %u, \"cir_pwr\": %u, \"cfo\": %d, \"rx_pacc\": %u,"
			   "\"ts\": %llu",
			   conf->ranging_id, curr_frame->ranging_id, rtc_slot_ts, i, j,
			   curr_frame_info->fp_ampl1, curr_frame_info->fp_ampl2,
			   curr_frame_info->fp_ampl3, curr_frame_info->fp_index >> 6,
			   curr_frame_info->cir_pwr, (int)(curr_frame_info->cfo_ppm * 1e6),
                        curr_frame_info->rx_pacc, curr_frame_info->timestamp);
		}

                if (additional != NULL) {
                    printk(", %s", additional);
                }

                printk("}\n");
            }
        }
    }
}



/* int output_cir_uart(int slot, int phase, const uint8_t *cir_memory, size_t size) */
/* { */

/*     // for debug purposes we will measure the timing this takes, which should be quite hefty */
/*     unsigned int start_ts = k_cycle_get_32(), end_ts; */

/*     int cir_bytes_written = 0; */
/*     int ret, length; */
/*     uart_callback_set(uart_dev, uart_finished_callback, NULL); */

/*     k_sem_take(&uart_transfer_finished, K_FOREVER); */
/*     length = snprintf(serial_buf, sizeof(serial_buf), */
/* 				       "{\"event\": \"cir\", \"rtc\": %llu, \"slot\": %d, " */
/* 				       "\"phase\": %d, \"length\": %d, \"data\":\"", */
/* 				       next_event_time, slot, phase, size); */

/*     ret = uart_tx(uart_dev, serial_buf, length, SYS_FOREVER_US); */

/*     if (ret) { */
/*         LOG_WRN("UART TX failed: %d", ret); */
/* 	k_sem_give(&uart_transfer_finished); */

/*         return -1; */
/*     } */

/*     // Base64 encode the data */
/*     do { */
/*         k_sem_take(&uart_transfer_finished, K_FOREVER); // always wait for previous uart transfer to finish */

/* 	ret = base64_encode(serial_buf, sizeof(serial_buf), &length, */
/* 				 cir_memory + cir_bytes_written, */
/* 				 MIN(CIR_CHUNK_WRITE_SIZE, size - cir_bytes_written)); */

/* 	if (ret >= 0) { */
/* 	    ret = uart_tx(uart_dev, serial_buf, length, SYS_FOREVER_US); */

/*             if (ret) { */
/*                 LOG_ERR("UART TX failed: %d", ret); */
/*                 k_sem_give(&uart_transfer_finished); */
/* 	    } */

/*             cir_bytes_written += CIR_CHUNK_WRITE_SIZE; */
/* 	} else { */
/* 	    LOG_ERR("Base64 encoding failed"); */
/*             k_sem_give(&uart_transfer_finished); */
/*             return -1; */
/*         } */
/*     } while ((int)size - cir_bytes_written > 0); */

/*     // wait for encoded part to finish */
/*     k_sem_take(&uart_transfer_finished, K_FOREVER); */

/*     length = snprintf(serial_buf, sizeof(serial_buf), "\"}\n"); */
/*     ret = uart_tx(uart_dev, serial_buf, length, SYS_FOREVER_US); */

/*     if (ret) { */
/*         LOG_ERR("UART TX failed: %d", ret); */
/*         k_sem_give(&uart_transfer_finished); */
/*     } */

/*     end_ts = k_cycle_get_32(); */

/*     // sys_clock_hw_cycles_per_sec() */
/*     LOG_WRN("CIR output took %d ms", ( (end_ts - start_ts) * 1000 ) / sys_clock_hw_cycles_per_sec()); */

/*     return 0; */
/* } */


