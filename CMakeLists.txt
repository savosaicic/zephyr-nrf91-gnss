cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(zephyr_nrf91_gnss)

target_include_directories(app PRIVATE include)
target_sources(app PRIVATE src/main.c src/fake_agps_data.c)
