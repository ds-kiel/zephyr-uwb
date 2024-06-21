cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr)
project(uwb_swarm_ranging)

target_sources(app PRIVATE
  # src/main.c
  src/nodes.c
  src/log.c
  src/utils.c
  src/debug-gpio.c
  src/history.c
  src/ranging-engine.c
  # src/rangings-slots.c
  src/time-synchronization.c
  src/testbeds/testbed_kiel.c
  src/testbeds/testbed_toulouse.c
  src/testbeds/testbed_drone.c
  src/testbeds/testbed_lille.c
  src/testbeds/testbed_trento_a.c
  src/testbeds/testbed_trento_b.c


  # apps/cir-evaluation.c
  apps/cycle-slot-durations.c
)
