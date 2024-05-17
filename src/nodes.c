#include <zephyr/kernel.h>
#include <zephyr/drivers/hwinfo.h>
#include "nodes.h"

// testbed defined in testbed.c

inline size_t pair_index(uint16_t a, uint16_t b) {
       return PAIR_INDEX(a,b);
}


uint16_t get_own_node_id() {
    uint8_t id_buf[2] = {0};
    ssize_t copied_bytes = hwinfo_get_device_id(id_buf, sizeof(id_buf));
    return id_buf[0] << 8 | id_buf[1];
}

int8_t get_node_number(uint16_t node_id) {
    for (int i = 0; i < NUM_NODES; i++) {
        if (node_id == node_ids[i]) {
            return i;
        }
    }

    return -1;  //node number not found!
}
