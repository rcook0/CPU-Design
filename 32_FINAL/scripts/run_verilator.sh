#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
verilator --cc --exe --build -Wall -O3 --timing \
  ../rtl/mc_common_pkg.sv \
  ../rtl/btb_dm.sv ../rtl/bht_2bit.sv ../rtl/interlock_unit.sv \
  ../rtl/instruction_memory.sv ../rtl/unified_l1_llsc.sv ../rtl/rr_arbiter.sv \
  ../rtl/l2_cache_simple.sv ../rtl/core7.sv ../rtl/multicore_top2.sv \
  ../sim/multicore_full_sim2.sv \
  --top-module multicore_full_sim2
../obj_dir/Vmulticore_full_sim2
