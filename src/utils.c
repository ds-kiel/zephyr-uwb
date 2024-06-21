#include "utils.h"

#include <zephyr/sys/hash_function.h>

void seeded_hash_permute_until_index(uint8_t permutation[], size_t overall_size, size_t permutation_size, uint64_t seed)
{
        // we only swap until we reach at least permutation_size items
        int limit = permutation_size > 0 ? MIN(overall_size-2, permutation_size) : overall_size-2;

        for (int i = 0; i <= limit; i++) {
                uint64_t hash_input = seed + i; // TODO: Use PRNG which should be more efficient than this
                uint32_t hash_output = sys_hash32((void *)&hash_input, sizeof(hash_input));

                int j = i + hash_output % (overall_size-i); /* A random integer such that i ≤ j < n */

                uint8_t val = permutation[i]; /* Swap the randomly picked element with permutation[i] */
                permutation[i] = permutation[j];
                permutation[j] = val;
        }
}
