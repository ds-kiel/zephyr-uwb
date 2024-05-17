#include "nodes.h"
// ----------------------
// Definition of Testbed lille

#if CONFIG_TESTBED_KIEL

#if NUM_NODES>12
#error Testbed Kiel only has 12 nodes
#elif NUM_NODES<12
#warning Testbed Kiel has more nodes available (12 in total)
#endif

uint16_t node_ids[NUM_NODES] = {
    0x2b75, // profiled-gateway-2-dwm1001-000760119483
    0x0320, // profiled-gateway-1
    0x67a4, // profiled-gateway-2-dwm1001-000760120271
    0xe680, // gateway-503-1
    0x9d21, // mobile-gateway
    0xf39f, // gateway-503-0
    0xc503, // gateway-503-2
    0xacc3, // gateway-503-4
    0xa96b, // gateway-503-6
    0xbdea, // gateway-503-9-dwm1001-000760120369
    0xde5d, // gateway-503-9-dwm1001-000760120245 Note: bad OTP values
    0x5321, // gateway-503-8
};

int16_t node_factory_antenna_delay_offsets[NUM_NODES] = {
    0x00
};


float32_t node_distances[NUM_PAIRS] = {
    0x00
};

#endif
