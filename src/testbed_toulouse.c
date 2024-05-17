#include "nodes.h"
// ----------------------
// Definition of Testbed lille

#if CONFIG_TESTBED_TOULOUSE

#if NUM_NODES>42
#error Testbed toulouse only has 42 nodes
#elif NUM_NODES<42
#warning Testbed toulouse has more nodes available (42 in total)
#endif

uint16_t node_ids[NUM_NODES] = {
	0x42ac, // dwm1001-35,
	0x6a65, // dwm1001-37,
	0xd24f, // dwm1001-17,
	0xa847, // dwm1001-77,
	0xbb06, // dwm1001-95,
	0xb393, // dwm1001-1,
	0x03b5, // dwm1001-30,
	0xc3df, // dwm1001-33,
	0x69bb, // dwm1001-42,
	0x114d, // dwm1001-82,
	0x40c2, // dwm1001-86,
	0x6924, // dwm1001-41,
	0xc077, // dwm1001-38,
	0x13aa, // dwm1001-9,
	0x1af1, // dwm1001-65,
	0x8961, // dwm1001-81,
	0x5263, // dwm1001-5,
	0x03eb, // dwm1001-85,
	0x83b6, // dwm1001-29,
	0x34ea, // dwm1001-6,
	0xf237, // dwm1001-43,
	0x1bf6, // dwm1001-70,
	0xc2c8, // dwm1001-2,
	0xe91d, // dwm1001-74,
	0x83fd, // dwm1001-94,
	0xa4fa, // dwm1001-34,
	0xc729, // dwm1001-19,
	0x157f, // dwm1001-14,
	0xf6a8, // dwm1001-18,
	0x4739, // dwm1001-10,
	0xdcbc, // dwm1001-45,
	0xdab5, // dwm1001-63,
	0x2177, // dwm1001-21,
	0xbff4, // dwm1001-25,
	0x330f, // dwm1001-39,
	0x05ee, // dwm1001-89,
	0xb2a8, // dwm1001-26,
	0x7888, // dwm1001-62,
	0xf4cf, // dwm1001-73,
	0x8c51, // dwm1001-13,
	0xce09, // dwm1001-69,
	0x7c37  // dwm1001-93,
};

// maps ranging id to degree
uint16_t neighbor_graph_degree[NUM_NODES] = {
	28, // Node: dwm1001-35 (0)
	29, // Node: dwm1001-37 (1)
	25, // Node: dwm1001-17 (2)
	15, // Node: dwm1001-77 (3)
	12, // Node: dwm1001-95 (4)
	25, // Node: dwm1001-1 (5)
	28, // Node: dwm1001-30 (6)
	28, // Node: dwm1001-33 (7)
	29, // Node: dwm1001-42 (8)
	9,  // Node: dwm1001-82 (9)
	8,  // Node: dwm1001-86 (10)
	26, // Node: dwm1001-41 (11)
	29, // Node: dwm1001-38 (12)
	25, // Node: dwm1001-9 (13)
	12, // Node: dwm1001-65 (14)
	14, // Node: dwm1001-81 (15)
	25, // Node: dwm1001-5 (16)
	15, // Node: dwm1001-85 (17)
	30, // Node: dwm1001-29 (18)
	25, // Node: dwm1001-6 (19)
	29, // Node: dwm1001-43 (20)
	24, // Node: dwm1001-70 (21)
	25, // Node: dwm1001-2 (22)
	20, // Node: dwm1001-74 (23)
	9,  // Node: dwm1001-94 (24)
	28, // Node: dwm1001-34 (25)
	25, // Node: dwm1001-19 (26)
	26, // Node: dwm1001-14 (27)
	25, // Node: dwm1001-18 (28)
	25, // Node: dwm1001-10 (29)
	28, // Node: dwm1001-45 (30)
	14, // Node: dwm1001-63 (31)
	26, // Node: dwm1001-21 (32)
	28, // Node: dwm1001-25 (33)
	28, // Node: dwm1001-39 (34)
	14, // Node: dwm1001-89 (35)
	28, // Node: dwm1001-26 (36)
	19, // Node: dwm1001-62 (37)
	25, // Node: dwm1001-73 (38)
	26, // Node: dwm1001-13 (39)
	21, // Node: dwm1001-69 (40)
	12, // Node: dwm1001-93 (41)
};

uint16_t aloha_access_prob[NUM_NODES] = {
	400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400,
	400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400,
	400, 400, 400, 400, 400, 400,
};

int16_t node_factory_antenna_delay_offsets[NUM_NODES] = {
    0x00
};


float32_t node_distances[NUM_PAIRS] = {
    0x00
};

#endif
