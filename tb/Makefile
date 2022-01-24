# //////////////////////////////////////////////////////////////////////////////
# // SPDX-FileCopyrightText: 2021, Dinesh Annayya
# // 
# // Licensed under the Apache License, Version 2.0 (the "License");
# // you may not use this file except in compliance with the License.
# // You may obtain a copy of the License at
# //
# //      http://www.apache.org/licenses/LICENSE-2.0
# //
# // Unless required by applicable law or agreed to in writing, software
# // distributed under the License is distributed on an "AS IS" BASIS,
# // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# // See the License for the specific language governing permissions and
# // limitations under the License.
# // SPDX-License-Identifier: Apache-2.0
# // SPDX-FileContributor: Dinesh Annayya <dinesha@opencores.org>
# // //////////////////////////////////////////////////////////////////////////
#------------------------------------------------------------------------------
# Makefile for Synthesis
#------------------------------------------------------------------------------

# Paths
export ROOT_DIR := $(shell pwd)

## Simulation mode: RTL/GL
SIM?=RTL
DUMP?=OFF


# Targets
.PHONY: clean dcache icache gate synth help

PATTERNS = dcache icache 

default: clean dcache

$(PATTERNS): clean 
	@echo "Generating Random Dump file"
	gcc hex_gen.c -o hex_gen
	./hex_gen > dumpfile.hex
ifeq ($(SIM),RTL)
   ifeq ($(DUMP),OFF)
	@echo "Running $@ Verilog Simulation"
	iverilog -g2012  -D SPEEDSIM  -DFUNCTIONAL -DSIM -I./ -I ../src/core -I ../src/model -I $(PDK_ROOT)/sky130A  $@_tb_top.sv  -o $@_tb_top.vvp
	vvp $@_tb_top.vvp | tee sim_result.log
    else
	@echo "Running $@ Verilog Simulation with waveform "
	iverilog -g2012  -D SPEEDSIM  -DFUNCTIONAL -DSIM -DWFDUMP -I./ -I ../src/core -I ../src/model -I $(PDK_ROOT)/sky130A  $@_tb_top.sv  -o $@_tb_top.vvp
	vvp $@_tb_top.vvp | tee sim_result.log
    endif
else
	@echo "Running $@ Gate level Simulation"
	iverilog -g2012  -D GL -D SPEEDSIM  -DFUNCTIONAL -DSIM  -I./ -I ../synth/netlist/  -I ../src/model -I $(PDK_ROOT)/sky130A  $@_tb_top.sv  -o $@_tb_top.vvp
	vvp $@_tb_top.vvp | tee sim_result.log
endif


help:
	@echo "To run RTL  simulation: make dcache SIM=RTL"
	@echo "To run RTL  simulation: make icache SIM=RTL"
	@echo "To run RTL  simulation: make dcache SIM=RTL DUMP=ON"
	@echo "To run RTL  simulation: make icache SIM=RTL DUMP=ON"
	@echo "To run RTL  simulation: make dcache SIM=GL"
	@echo "To run RTL  simulation: make icache SIM=GL"
	@echo "Note: Before running gate sim make sure that you have completed synthesis"



clean:
	$(RM) -R hex_gen
	$(RM) -R *.hex
	$(RM) -R *.vvp
	$(RM) -R *.vcd
	$(RM) -R *.fst
	$(RM) -R *.log
