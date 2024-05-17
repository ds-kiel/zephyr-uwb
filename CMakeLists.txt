cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr)
project(uwb_swarm_ranging)

target_sources(app PRIVATE src/main.c src/nodes.c src/log.c src/history.c src/testbed_kiel.c src/testbed_toulouse.c src/testbed_drone.c src/testbed_lille.c src/testbed_trento_a.c src/testbed_trento_b.c)
