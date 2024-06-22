import os
import numpy as np
import json

import scipy.optimize

import logs

import matplotlib.pyplot as plt
from utility import slugify, cached_legacy, init_cache, load_env_config, set_global_cache_prefix_by_config

from export import load_plot_defaults, save_and_crop, CONFIDENCE_FILL_COLOR, PERCENTILES_FILL_COLOR, COLOR_MAP, PROTOCOL_NAME

from channel_usage import calculate_optimal_access_prob

import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import math, itertools

import pandas as pd
import random


def two_hop_graph(G):
    """
    Generate the 2-hop graph of the input graph G.

    :param G: Input graph.
    :return: 2-hop graph of G.
    """
    H = nx.Graph()

    for u in G.nodes():
        # Consider all pairs of neighbors of u
        for v, w in itertools.combinations(G.neighbors(u), 2):
            # If v and w are not connected, then they are two hops away via u
            # if not G.has_edge(v, w):
            H.add_edge(v, w)

    # Set position attributes if they exist in G
    if nx.get_node_attributes(G, 'pos'):
        pos = nx.get_node_attributes(G, 'pos')
        for node in H.nodes():
            H.nodes[node]['pos'] = pos[node]

    return H




def generate_random_graph(n, target_edges=None):
    # Create a fully connected graph with n nodes
    G = nx.complete_graph(n)

    if target_edges is None:
        target_edges = random.randint(n - 1, n * (n - 1) // 2)

    assert target_edges >= n - 1

    # While the number of edges is greater than target_edges, remove edges
    while G.number_of_edges() > target_edges:
        edge = random.choice(list(G.edges()))

        # Temporarily remove the edge
        G.remove_edge(*edge)

        # Check if the graph is still connected
        if nx.is_connected(G):
            continue  # Keep the edge removed if the graph is still connected
        else:
            G.add_edge(*edge)  # Restore the edge if the graph became disconnected

    return G

# Example usage
# n = 10  # Number of nodes
# random_graph = generate_random_graph(n)
#
# print("Nodes:", random_graph.nodes())
# print("Edges:", random_graph.edges())
#
# nx.draw(random_graph, node_color='b', edge_color='b', with_labels=True)
# nx.draw(two_hop_graph(random_graph), node_color='r', edge_color='r', with_labels=True)
# plt.show()


# schedule: list of sets of nodes that are transmitting in each slot
def analyze_schedule(g, schedule, assert_collision_free=True):
    node_res = []

    for node in range(g.number_of_nodes()):

        neighbors = set(g.neighbors(node))
        res = {
            "success": 0,
            "collision": 0,
            "silence": 0,
        }

        for transmitting_nodes in schedule:
            transmitting_neighbors = neighbors.intersection(set(transmitting_nodes))

            if len(transmitting_neighbors) > 1:
                res["collision"] += 1
            elif len(transmitting_neighbors) == 1:
                res["success"] += 1
            else:
                res["silence"] += 1

        res['measurements'] = res['success'] * (res['success']-1)
        node_res.append(res)

    aggr_res = {
        "success_avg": sum([res["success"] for res in node_res]) / g.number_of_nodes(),
        "success_min": min([res["success"] for res in node_res]),
        "success_max": max([res["success"] for res in node_res]),
        "collision_avg": sum([res["collision"] for res in node_res]) / g.number_of_nodes(),
        "collision_min": min([res["collision"] for res in node_res]),
        "collision_max": max([res["collision"] for res in node_res]),
        "silence_avg": sum([res["silence"] for res in node_res]) / g.number_of_nodes(),
        "silence_min": min([res["silence"] for res in node_res]),
        "silence_max": max([res["silence"] for res in node_res]),
        "measurements_avg": sum([res["measurements"] for res in node_res]) / g.number_of_nodes(),
        "measurements_min": min([res["measurements"] for res in node_res]),
        "measurements_max": max([res["measurements"] for res in node_res]),
    }

    if assert_collision_free:
        assert aggr_res["collision_max"] == 0, "Collision rate is not zero"

    return aggr_res, node_res


def create_hashed_schedule(g, num_slots):
    if g.number_of_nodes() > num_slots:
        s = random.sample(list(range(g.number_of_nodes())), num_slots)
    else:
        s = list(range(g.number_of_nodes()))
    return [{x} for x in s]


def create_slotted_aloha_schedule(g, num_slots, p=1.0):

    chosen_slots = [random.randint(0, num_slots - 1) for _ in range(g.number_of_nodes())]
    if isinstance(p, list):
        chosen_transmission = [random.random() < p[i] for i in range(g.number_of_nodes())]
    else:
        chosen_transmission = [random.random() < p for _ in range(g.number_of_nodes())]

    return [ set([n for n in range(g.number_of_nodes()) if chosen_transmission[i] and chosen_slots[n] == i]) for i in range(num_slots)]


def create_dynamic_slotted_aloha_schedule(g, num_slots, collision_prob_threshold=0.1):

    params = [
        (len(list(g.neighbors(n))), num_slots) for n in range(g.number_of_nodes())
    ]

    access_probabilities = list(
        calculate_optimal_access_prob(collision_prob_threshold, params, plot=False)
    )

    return create_slotted_aloha_schedule(g, num_slots, p=access_probabilities)

def create_coloring_schedule(g, num_slots, strategy="largest_first"):

    two_hop_interference_graph = two_hop_graph(g)
    d = nx.coloring.greedy_color(two_hop_interference_graph, strategy=strategy)

    colors = list(set([c for (n, c) in d.items()]))

    if len(colors) > num_slots:
        colors = random.sample(colors, num_slots)

    return [ [n for n in d if d[n] == c] for c in colors]


def export_by_density(export_dir):
    num_nodes = 50
    num_slots = 15
    num_graphs = 1000

    graphs = []
    two_hop_graphs = []
    densities = []
    avg_degrees = []

    SAVE_GRAPHS = True

    NUM_SIMULATED_ROUNDS = 10

    print("Generating graphs...")
    for i in range(num_graphs):
        g = generate_random_graph(num_nodes, random.randint(num_nodes - 1, math.floor((num_nodes * (num_nodes - 1) / 2) * 0.5)))
        graphs.append(g)
        two_hop_graphs.append(two_hop_graph(g))
        density = nx.density(g)
        densities.append(density)

        avg_degree = sum([d for n, d in nx.degree(g)]) / g.number_of_nodes()
        avg_degrees.append(avg_degree)

        path = os.path.join(export_dir, "graph_{}.png".format(round(density * 100)))
        if SAVE_GRAPHS and not os.path.exists(path):
            plt.clf()
            nx.draw(g)
            plt.savefig(path)

    print("Done generating")

    for aggr in ["avg", "min", "max"]:

        color_aggs = []
        hash_aggs = []
        slotted_aloha_aggs = []
        slotted_aloha_collisions_aggs = []

        dyn_slotted_aloha_aggs = []
        dyn_slotted_aloha_collisions_aggs = []

        slotted_aloha_p = 0.5
        dynamic_slotted_aloha_collision_threshold = 0.1


        metric = 'measurements'
        
        for r in range(NUM_SIMULATED_ROUNDS):
            for g in graphs:
                color_sched = create_coloring_schedule(g, num_slots)
                hash_sched = create_hashed_schedule(g, num_slots)
                slotted_aloha_sched = create_slotted_aloha_schedule(g, num_slots, p=slotted_aloha_p)
                dynaic_slotted_aloha_sched = create_dynamic_slotted_aloha_schedule(g, num_slots, collision_prob_threshold=dynamic_slotted_aloha_collision_threshold)

                color_aggs.append(analyze_schedule(g, color_sched)[0][metric + '_'+ aggr])
                hash_aggs.append(analyze_schedule(g, hash_sched)[0][metric + '_'+ aggr])


                slotted_aloha_aggr_res = analyze_schedule(g, slotted_aloha_sched, assert_collision_free=False)[0]
                slotted_aloha_aggs.append(slotted_aloha_aggr_res[metric + '_'+ aggr])
                slotted_aloha_collisions_aggs.append(slotted_aloha_aggr_res['collision_'+ aggr])

                dyn_slotted_aloha_aggr_res = analyze_schedule(g, dynaic_slotted_aloha_sched, assert_collision_free=False)[0]
                dyn_slotted_aloha_aggs.append(dyn_slotted_aloha_aggr_res[metric + '_'+ aggr])
                dyn_slotted_aloha_collisions_aggs.append(dyn_slotted_aloha_aggr_res['collision_'+ aggr])

                #slotted_aloha_with_collisions_aggs.append(slotted_aloha_aggr_res[metric + '_'+ aggr] + slotted_aloha_aggr_res['collision_'+ aggr])


                #print("Graph", i, "Density:", density, "Average neighbor degree:", avg_degree)

        plt.clf()
        plt.scatter(densities * NUM_SIMULATED_ROUNDS, color_aggs, label="Coloring", alpha=0.8, edgecolors='none')
        plt.scatter(densities * NUM_SIMULATED_ROUNDS, hash_aggs, label="Hashed", alpha=0.8, edgecolors='none')
        plt.scatter(densities * NUM_SIMULATED_ROUNDS, slotted_aloha_aggs, label="Slotted Aloha {}".format(slotted_aloha_p), alpha=0.8,  edgecolors='none')
        #plt.scatter(densities, dyn_slotted_aloha_aggs, label="Slotted Aloha (Dynamic)", alpha=0.8,  edgecolors='none')
        plt.ylabel("{} #measurements".format(aggr))
        plt.xlabel("densities")
        plt.legend()

        #plt.show()
        plt.savefig(os.path.join(export_dir, "scheduling_efficiency_by_density_{}_{}_{}_nodes_{}_slots_{}_graphs_{}_rounds_.png".format(metric, aggr, num_nodes, num_slots, num_graphs, NUM_SIMULATED_ROUNDS)))


        plt.clf()
        plt.scatter(densities * NUM_SIMULATED_ROUNDS, slotted_aloha_collisions_aggs, label="Slotted Aloha {}".format(slotted_aloha_p), alpha=0.8,
                    edgecolors='none')
        # plt.scatter(densities, dyn_slotted_aloha_collisions_aggs, label="Slotted Aloha Dynamic",
        #             alpha=0.8,
        #             edgecolors='none')
        plt.ylabel("{} #Collisions".format(aggr))
        plt.xlabel("densities")
        plt.legend()
        #plt.show()
        plt.savefig(os.path.join(export_dir, "scheduling_efficiency_collisions_{}_{}_{}_nodes_{}_slots_{}_graphs_{}_rounds_.png".format(metric, aggr, num_nodes, num_slots, num_graphs, NUM_SIMULATED_ROUNDS)))


def export_by_nodes(export_dir):
    MIN_NODES = 50
    MAX_NODES = 150

    xs = list(np.linspace(MIN_NODES, MAX_NODES, 50, dtype=int))
    num_slots = 15

    num_graphs_per_count = 20

    NUM_SIMULATED_ROUNDS = 1 # TODO: Not sure if this works atm

    metric = 'measurements'
    aggr = 'avg'

    plt.clf()

    for density in [0.2]:
        color_aggs = []
        hash_aggs = []
        slotted_aloha_aggs = []
        slotted_aloha_collisions_aggs = []

        dyn_slotted_aloha_aggs = []
        dyn_slotted_aloha_collisions_aggs = []

        slotted_aloha_p = 0.5
        dynamic_slotted_aloha_collision_threshold = 0.1

        for r in range(NUM_SIMULATED_ROUNDS):
            for num_nodes in xs:
                for gc in range(num_graphs_per_count):

                    g = nx.gnp_random_graph(num_nodes, density)

                    color_sched = create_coloring_schedule(g, num_slots)
                    hash_sched = create_hashed_schedule(g, num_slots)
                    slotted_aloha_sched = create_slotted_aloha_schedule(g, num_slots, p=slotted_aloha_p)
                    dynaic_slotted_aloha_sched = create_dynamic_slotted_aloha_schedule(g, num_slots,
                                                                                       collision_prob_threshold=dynamic_slotted_aloha_collision_threshold)

                    color_aggs.append(analyze_schedule(g, color_sched)[0][metric + '_' + aggr])
                    hash_aggs.append(analyze_schedule(g, hash_sched)[0][metric + '_' + aggr])

                    slotted_aloha_aggr_res = \
                    analyze_schedule(g, slotted_aloha_sched, assert_collision_free=False)[0]
                    slotted_aloha_aggs.append(slotted_aloha_aggr_res[metric + '_' + aggr])
                    slotted_aloha_collisions_aggs.append(slotted_aloha_aggr_res['collision_' + aggr])

                    dyn_slotted_aloha_aggr_res = \
                        analyze_schedule(g, dynaic_slotted_aloha_sched, assert_collision_free=False)[0]
                    dyn_slotted_aloha_aggs.append(dyn_slotted_aloha_aggr_res[metric + '_' + aggr])
                    dyn_slotted_aloha_collisions_aggs.append(dyn_slotted_aloha_aggr_res['collision_' + aggr])


        plt.scatter(xs * NUM_SIMULATED_ROUNDS * num_graphs_per_count, color_aggs, label="Coloring @ {}".format(density), alpha=0.5,
                    edgecolors='none')
        plt.scatter(xs * NUM_SIMULATED_ROUNDS * num_graphs_per_count, hash_aggs, label="Hashed @ {}".format(density), alpha=0.5,
                    edgecolors='none')
        plt.scatter(xs * NUM_SIMULATED_ROUNDS * num_graphs_per_count, slotted_aloha_aggs, label="Slotted Aloha {} @ {}".format(slotted_aloha_p, density), alpha=0.5, edgecolors='none')

    plt.ylabel("{} #measurements".format(aggr))
    plt.xlabel("#nodes")
    plt.legend()

    #plt.show()
    plt.savefig(os.path.join(export_dir,
                             "scheduling_efficiency_by_nodes_{}_{}_{}_nodes_{}_slots_{}_graphs_{}_rounds_.png".format(
                                 metric, aggr, MAX_NODES, num_slots, num_graphs_per_count, NUM_SIMULATED_ROUNDS)))









if __name__ == '__main__':
    config = load_env_config()
    load_plot_defaults()
    assert 'EXPORT_DIR' in config and config['EXPORT_DIR']
    if 'CACHE_DIR' in config and config['CACHE_DIR']:
        init_cache(config['CACHE_DIR'])

    export_by_density(config['EXPORT_DIR'])
    export_by_nodes(config['EXPORT_DIR'])