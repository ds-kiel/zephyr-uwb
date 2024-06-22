import matplotlib.pyplot as plt
import numpy as np

# import binomial from numpy
from numpy.random import binomial

import math


# plot binomial
def calc_collision_prob(n, s, p):
    agg = 0
    for k in range(2, n+1): # we can start with 2 devices as otherwise there wont be collisions
        p_k_active_devices =  math.comb(n, k) * p**k * (1 - p)**(n-k)  # the probability that k out of n devices try to send in the same round
        p_no_collision =  (math.factorial(k)*math.comb(s, k)) / s**k  # the probability that there is no collision of k active devices with s slots
        agg += p_k_active_devices*(1-p_no_collision)
    return agg



params = [
    (28, 10),
    (29, 10),
    (25, 10),
    (15, 10),
    (12, 10),
    (25, 10),
    (28, 10),
    (28, 10),
    (29, 10),
   (9, 10),
   (8, 10),
    (26, 10),
    (29, 10),
    (25, 10),
    (12, 10),
    (14, 10),
    (25, 10),
    (15, 10),
    (30, 10),
    (25, 10),
    (29, 10),
    (24, 10),
    (25, 10),
    (20, 10),
   (9, 10),
    (28, 10),
    (25, 10),
    (26, 10),
    (25, 10),
    (25, 10),
    (28, 10),
    (14, 10),
    (26, 10),
    (28, 10),
    (28, 10),
    (14, 10),
    (28, 10),
    (19, 10),
    (25, 10),
    (26, 10),
    (21, 10),
    (12, 10),
]



def calculate_optimal_access_prob(threshold, params, plot=True):
    ps = np.linspace(0, 1, 100)

    for N, S in params:
        probs = [calc_collision_prob(N, S, p) for p in ps]
        if plot:
            plt.plot(ps, probs, label="n={}, s={}".format(N, S))
        best_i = max([i for (i,x) in enumerate(probs) if x <= threshold])
        #print("n={}, s={}, p={}".format(N, S, ps[best_i]))
        #print("{},".format(round(ps[best_i]*1000)))
        yield probs[best_i]
if __name__ == '__main__':


    list(calculate_optimal_access_prob(0.2, params))


    plt.legend()
    plt.show()


    # exit()

    # def calc_expected_transmissions_without_collisions(n, s, p):
    #     agg = 0
    #     for k in range(2, n+1): # we can start with 2 devices as otherwise there wont be collisions
    #         p_k_active_devices =  math.comb(n, k) * p**k * (1 - p)**(n-k)  # the probability that k out of n devices try to send in the same round
    #         p_no_collision =  (math.factorial(k)*math.comb(s, k)) / s**k  # the probability that there is no collision of k active devices with s slots
    #         agg += k*p_k_active_devices*p_no_collision
    #     return agg

    # params = [
    #     (10, 10),
    #     (14, 10),
    #     (20, 10),
    # ]

    # ps = np.linspace(0, 1, 100)

    # THRESHOLD = 0.1

    # for N, S in params:
    #     probs = [calc_expected_transmissions_without_collisions(N, S, p) for p in ps]
    #     plt.plot(ps, probs, label="n={}, s={}".format(N, S))
    #     best_i = max([i for (i,x) in enumerate(probs) if x <= THRESHOLD])
    #     print("n={}, s={}, p={}".format(N, S, ps[best_i]))




    # plt.legend()
    # plt.show()
