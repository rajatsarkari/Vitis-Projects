############################################################
## This file is generated automatically by Vitis HLS.
## Please DO NOT edit it.
## Copyright (C) 1986-2020 Xilinx, Inc. All Rights Reserved.
############################################################
open_project Sched_Bind
set_top foo
add_files Sched_Bind/.settings/foo.cpp
open_solution "solution1"
set_part {xcu200-fsgd2104-2-e}
create_clock -period 10 -name default
#source "./Sched_Bind/solution1/directives.tcl"
#csim_design
csynth_design
#cosim_design
export_design -format ip_catalog
