#!/usr/bin/env bash
# Builds and runs the host tests for the pure logic (no Arduino headers).
set -euo pipefail
cd "$(dirname "$0")/.."
LOGIC=firmware/ImuBench/src/logic
mkdir -p build/native
c++ -std=c++11 -Wall -Wextra -Werror -I "$LOGIC" \
  "$LOGIC"/*.cpp tests/native/*.cpp -o build/native/mouthmod_tests
./build/native/mouthmod_tests
