#include "../nodes.h"
// ----------------------
// Definition of Testbed Drone

#if CONFIG_TESTBED_DRONE

#if NUM_NODES>4
#error Testbed Kiel only has 3 nodes
#elif NUM_NODES<4
#warning Testbed Kiel has more nodes available (3 in total)
#endif

uint16_t node_ids[NUM_NODES] = {
    0x4d0f, // drone-1
    0xbe83, // drone-2
    0xc7f6, // drone-2
    0x4b51, // drone-4 without-case
};

int16_t node_factory_antenna_delay_offsets[NUM_NODES] = {0x00};

uint16_t neighbor_graph_degree[NUM_NODES] = {0x00};

uint16_t aloha_access_prob[NUM_NODES] = {400, 400, 400, 400};



float32_t node_distances[NUM_PAIRS] = {
    0x00
};

#endif
