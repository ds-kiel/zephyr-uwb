#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <zephyr/kernel.h>

void seeded_hash_permute_until_index(uint8_t permutation[], size_t overall_size,
				     size_t permutation_size, uint64_t seed); // TODO: use PRNG!

#endif
