#!/usr/bin/env bash
# Standalone C++ algorithm tests — no ROS required, just g++.
# Extracts the algorithm blocks VERBATIM from the shipped nodes so the
# tests always exercise the code that actually runs.
set -euo pipefail
cd "$(dirname "$0")"

awk '/^namespace$/{f=1} f{print} /^}  \/\/ namespace$/{if(f)exit}' \
    ../src/path_planning_node.cpp > algo_extract.hpp
awk '/^namespace$/{f=1} f{print} /^}  \/\/ namespace$/{if(f)exit}' \
    ../src/cone_mapping_node.cpp > mapping_extract.hpp

CXXFLAGS="-std=c++17 -Wall -Wextra -O2"

g++ $CXXFLAGS -o test_cpp_algo test_cpp_algo.cpp
./test_cpp_algo

g++ $CXXFLAGS -o test_cpp_mapping test_cpp_mapping.cpp
./test_cpp_mapping

echo
echo "--- planning benchmark (informational; 10 Hz budget = 100 ms) ---"
g++ $CXXFLAGS -o bench_planning bench_planning.cpp
./bench_planning

echo
echo "ALL fsd_cpp STANDALONE TESTS PASSED"
