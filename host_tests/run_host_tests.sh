#!/usr/bin/env bash
set -euo pipefail
cc -std=c11 -Wall -Wextra -Werror -Isrc \
  host_tests/test_protocol.c \
  src/protocol/protocol.c src/protocol/crc16.c \
  src/control/ball_controller.c \
  src/motor/emm_v5_protocol.c \
  -o /tmp/bbc_host_tests
/tmp/bbc_host_tests
