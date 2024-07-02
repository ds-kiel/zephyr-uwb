#include "../nodes.h"
// ----------------------
// Definition of Testbed lille

#if CONFIG_TESTBED_KIEL

#if NUM_NODES>15
#error Testbed Kiel only has 14 nodes
#elif NUM_NODES<15
#warning Testbed Kiel has more nodes available (14 in total)
#endif

uint16_t node_ids[NUM_NODES] = {
	0x4b51, // gateway-stativ-2-dwm1001-000760118589
	0xc7f6, // gateway-stativ-1-dwm1001-000760120221
	0x4d0f, // gateway-stativ-3-dwm1001-000760119517
	0xbe83, // gateway-stativ-4-dwm1001-000760119517
	0xbdea, // gateway-503-9-dwm1001-000760120369		//
	0x2b75, // profiled-gateway-1
	0x0320, // profiled-gateway-1-dwm1001-000760119483		//
	0xe680, // gateway-503-1-dwm1001-000760120328
	0x9d21, // mobile-gateway
	0xf39f, // gateway-503-0-dwm1001-000760120293
	0xc503, // gateway-503-2-dwm1001-000760120263
        0x67a4, // gateway-503-2-dwm1001-000760120271
	0xde5d, // gateway-503-9-dwm1001-000760120245 Note: bad OTP values
	0xa96b, // gateway-503-6-dwm1001-000760120318
	0x5321, // gateway-503-8-dwm1001-000760120244
};



uint16_t neighbor_graph_degree[NUM_NODES] = {0x00};

uint16_t aloha_access_prob[NUM_NODES] = {
	800,
	800,
	800,
	800,
	800,
	800,
	800,
	800,
	800,
	800,
	800,
	800,
	800
};

int16_t node_factory_antenna_delay_offsets[NUM_NODES] = {
    0x00
};


float32_t node_distances[NUM_PAIRS] = {
    0x00
};

#endif
