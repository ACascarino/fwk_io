#!/bin/bash
set -e

FRAMEWORK_IO_ROOT=`git rev-parse --show-toplevel`
BOARD="XCORE-AI-EXPLORER"
TOOLCHAIN="xmos_cmake_toolchain/xs3a.cmake"

source ${FRAMEWORK_IO_ROOT}/tools/ci/helper_functions.sh

# row format is: "make_target BOARD toolchain"
declare -a applications=(
    "test_hil_i2s_tdm_tx16_slave_test_0"
    "test_hil_i2s_tdm_tx16_slave_test_1"
    "test_hil_i2s_tdm_tx16_slave_test_2"
)

# perform builds
path="${FRAMEWORK_IO_ROOT}"
echo '**************************'
echo '* Building lib_i2s tests *'
echo '**************************'

(cd ${path}; rm -rf build_ci_${BOARD})
(cd ${path}; mkdir -p build_ci_${BOARD})
(cd ${path}/build_ci_${BOARD}; log_errors cmake ../ -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN} -DBOARD=${BOARD} -DFWK_IO_TESTS=ON; log_errors make ${applications[@]} -j$(nproc))
