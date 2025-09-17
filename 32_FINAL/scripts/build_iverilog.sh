#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
TOP=../rtl
TB=../sim/multicore_full_sim2.sv
iverilog -g2012 -o ../simv \
  $TOP/mc_common_pkg.sv \
  $TOP/btb_dm.sv $TOP/bht_2bit.sv $TOP/interlock_unit.sv \
  $TOP/instruction_memory.sv $TOP/unified_l1_llsc.sv $TOP/rr_arbiter.sv \
  $TOP/l2_cache_simple.sv $TOP/core7.sv $TOP/multicore_top2.sv \
  $TB
echo "[OK] Built simv"
