#include "nodes.h"
// ----------------------
// Definition of Testbed lille

#if CONFIG_TESTBED_KIEL

#if NUM_NODES>6
#error Testbed Kiel only has 6 nodes
#elif NUM_NODES<6
#warning Testbed Kiel has more nodes available (6 in total)
#endif

uint16_t node_ids[NUM_NODES] = {
    0x2b75, // profiled-gateway-2:
    0x0320, // profiled-gateway-1:
    0xe680, // gateway-503-1:
    0xf39f, // gateway-503-0:
    0xde5d, // gateway-503-2:
    0xacc3, // gateway-503-4:
    0xa96b, // gateway-503-6:
    0x9d21, // mobile-gateway:
    0xbdea, // gateway-503-9:
    0xba5e, // gateway-503-8:
};

int16_t node_factory_antenna_delay_offsets[NUM_NODES] = {
    0x00
};


float32_t node_distances[NUM_PAIRS] = {
    0x00
};

#endif