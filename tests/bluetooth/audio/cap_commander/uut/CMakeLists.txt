#
# Copyright (c) 2023 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: Apache-2.0
#
# CMakeLists.txt file for creating of uut library.
#

add_library(uut STATIC
  ${ZEPHYR_BASE}/subsys/bluetooth/audio/audio.c
  ${ZEPHYR_BASE}/subsys/bluetooth/audio/cap_commander.c
  ${ZEPHYR_BASE}/subsys/bluetooth/audio/cap_common.c
  ${ZEPHYR_BASE}/subsys/logging/log_minimal.c
  ${ZEPHYR_BASE}/lib/net_buf/buf_simple.c
  bap_broadcast_assistant.c
  aics.c
  cap_commander.c
  csip.c
  micp.c
  vcp.c
  vocs.c
)

add_subdirectory(${ZEPHYR_BASE}/tests/bluetooth/audio/mocks mocks)

target_link_libraries(uut PUBLIC test_interface mocks)
target_include_directories(uut PRIVATE ${ZEPHYR_BASE}/tests/bluetooth/audio/cap_commander/include)

target_compile_options(uut PRIVATE -std=c11 -include ztest.h)
