//////////////////////////////////////////////////////////////////////////////
// SPDX-FileCopyrightText: 2021 , Dinesh Annayya                          
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0
// SPDX-FileContributor: Created by Dinesh Annayya <dinesha@opencores.org>
//
///////////////////////////////////////////////////////////////////
//                                                              
//  Test Bench Top
////////////////////////////////////////////////////////////////////

`include "icache_tag_fifo.sv"
`include "icache_app_fsm.sv"
`include "icache_top.sv"
`include "memory_tb_wb.sv"
`include "sky130_sram_2kbyte_1rw1r_32x512_8.v"

module tb_top;
parameter CLK1_PERIOD = 10;


logic			   clk         ; //Clock input 
logic			   rst_n       ; //Active Low Asynchronous Reset Signal Input

// Wishbone CPU I/F
logic                      wb_cpu_stb_i; // strobe/request
logic                      wb_cpu_cyc_i; // strobe/request
logic   [31:0]             wb_cpu_adr_i; // address
logic                      wb_cpu_we_i;  // write
logic   [31:0]             wb_cpu_dat_i; // data output
logic   [3:0]              wb_cpu_sel_i; // byte enable

logic   [31:0]             wb_cpu_dat_o; // data input
logic                      wb_cpu_ack_o; // acknowlegement
logic                      wb_cpu_err_o;  // error

// Wishbone CPU I/F
logic                      wb_app_stb_o; // strobe/request
logic   [31:0]             wb_app_adr_o; // address
logic                      wb_app_we_o;  // write
logic   [31:0]             wb_app_dat_o; // data output
logic   [3:0]              wb_app_sel_o; // byte enable
logic   [9:0]              wb_app_bl_o;  // Burst Length

logic   [31:0]             wb_app_dat_i; // data input
logic                      wb_app_ack_i; // acknowlegement
logic                      wb_app_lack_i;// last acknowlegement
logic                      wb_app_err_i ;// error

logic [31:0]               address;
logic [3:0]                be;
logic [31:0]               cmp_data;
logic                      test_fail;
logic  [31:0]              read_data     ;
logic [7:0]                local_memory [0:8095];
integer cnt;

always #(CLK1_PERIOD/2) clk  <= (clk === 1'b0);


initial begin
	clk = 0;
	rst_n = 0;
        wb_cpu_cyc_i ='h0;  // strobe/request
        wb_cpu_stb_i ='h0;  // strobe/request
        wb_cpu_adr_i ='h0;  // address
        wb_cpu_we_i  ='h0;  // write
        wb_cpu_dat_i ='h0;  // data output
        wb_cpu_sel_i ='h0;  // byte enable
	#100;
	rst_n <= 1'b1;	    // Release reset
	$readmemh("dumpfile.hex",u_tb_mem.memory);
	//for(cnt = 0; cnt < 32'h200_0000; cnt = cnt+4) begin
	//  $display("Memory content Address: %x : %x", cnt,{u_tb_mem.memory[{7'b0,cnt[24:2],2'b0}+3],
	//   	       u_tb_mem.memory[{7'b0,cnt[24:2],2'b0}+2],
	//   	       u_tb_mem.memory[{7'b0,cnt[24:2],2'b0}+1],
	//   	       u_tb_mem.memory[{7'b0,cnt[24:2],2'b0}+0]});
	//end
        repeat (10) @(posedge clk);
        test_fail = 0;
	$display("#####################################");
	$display("       Testing Sequential Address ...");
	$display("#####################################");
	for(cnt = 0; cnt < 1024; cnt = cnt+1) begin
       	   address= cnt*4; // $random;
	   cmp_data = {u_tb_mem.memory[{7'b0,address[24:2],2'b0}+3],
	   	       u_tb_mem.memory[{7'b0,address[24:2],2'b0}+2],
	   	       u_tb_mem.memory[{7'b0,address[24:2],2'b0}+1],
	   	       u_tb_mem.memory[{7'b0,address[24:2],2'b0}+0]};
	   wb_user_core_read_check({7'b0,address[24:2],2'b0},read_data,cmp_data);
           repeat (1) @(posedge clk);
	end
	
	$display("#####################################");
	$display("        Testing Random Address ...");
	$display("#####################################");
	for(cnt = 0; cnt < 1024; cnt = cnt+1) begin
       	   address= $random;
	   cmp_data = {u_tb_mem.memory[{7'b0,address[24:2],2'b0}+3],
	   	       u_tb_mem.memory[{7'b0,address[24:2],2'b0}+2],
	   	       u_tb_mem.memory[{7'b0,address[24:2],2'b0}+1],
	   	       u_tb_mem.memory[{7'b0,address[24:2],2'b0}+0]};
	   wb_user_core_read_check({7'b0,address[24:2],2'b0},read_data,cmp_data);
           repeat (1) @(posedge clk);
	end
	$display("#####################################");
	$display("        Testing Random Address[10:5] ...");
	$display("#####################################");
	for(cnt = 0; cnt < 1024; cnt = cnt+1) begin
       	   address[10:5]= $random;
	   cmp_data = {u_tb_mem.memory[{7'b0,address[24:2],2'b0}+3],
	   	       u_tb_mem.memory[{7'b0,address[24:2],2'b0}+2],
	   	       u_tb_mem.memory[{7'b0,address[24:2],2'b0}+1],
	   	       u_tb_mem.memory[{7'b0,address[24:2],2'b0}+0]};
	   wb_user_core_read_check({7'b0,address[24:2],2'b0},read_data,cmp_data);
           repeat (1) @(posedge clk);
	end
      $display("###################################################");
      if(test_fail == 0) begin
         `ifdef GL
             $display("Monitor: Standalone (GL) Passed");
         `else
             $display("Monitor: Standalone (RTL) Passed");
         `endif
      end else begin
          `ifdef GL
              $display("Monitor: Standalone (GL) Failed");
          `else
              $display("Monitor: Standalone (RTL) Failed");
          `endif
       end
      $display("###################################################");
      #100
      $finish;
end


`ifdef WFDUMP
   initial begin
   	$dumpfile("simx.vcd");
   	$dumpvars(0, tb_top);
   	//$dumpvars(3, tb_top.u_core);
   end
`endif


icache_top #(
	 .WB_AW      (32),
	 .WB_DW      (32),
	 .TAG_MEM_WD (22),
	 .TAG_MEM_DP (16),
         .CACHELINES (16), // 16 Cache Line
         .CACHESIZE  (32) // Each cache is 32 Word
        ) u_core (
	.mclk                (clk         )   , //Clock input 
	.rst_n               (rst_n       )   , //Active Low Asynchronous Reset Signal Input

	.cfg_pfet_dis        (1'b0        ),
	.cfg_ntag_pfet_dis   (1'b0        ),

        .wb_cpu_stb_i        (wb_cpu_stb_i)   , // strobe/request
        .wb_cpu_adr_i        (wb_cpu_adr_i)   , // address
        .wb_cpu_we_i         (wb_cpu_we_i )   , // write
        .wb_cpu_dat_i        (wb_cpu_dat_i)   , // data output
        .wb_cpu_sel_i        (wb_cpu_sel_i)   , // byte enable
                                          
        .wb_cpu_dat_o        (wb_cpu_dat_o)   , // data input
        .wb_cpu_ack_o        (wb_cpu_ack_o)   , // acknowlegement
        .wb_cpu_err_o        (wb_cpu_err_o)   ,  // error

	// Wishbone CPU I/F
        .wb_app_stb_o        (wb_app_stb_o)   , // strobe/request
        .wb_app_adr_o        (wb_app_adr_o)   , // address
        .wb_app_we_o         (wb_app_we_o )   , // write
        .wb_app_dat_o        (wb_app_dat_o)   , // data output
        .wb_app_sel_o        (wb_app_sel_o)   , // byte enable
        .wb_app_bl_o         (wb_app_bl_o )   , // Burst Length
                                          
        .wb_app_dat_i        (wb_app_dat_i )  , // data input
        .wb_app_ack_i        (wb_app_ack_i )  , // acknowlegement
        .wb_app_lack_i       (wb_app_lack_i)  , // last acknowlegement
        .wb_app_err_i        (wb_app_err_i )    // error


);







ycr1_memory_tb_wb #(
    .YCR1_WB_WIDTH(32),
    .YCR1_MEM_POWER_SIZE(25) // Memory sized increased for non TCM Mode
)  u_tb_mem (
    // Control
    .rst_n                   (rst_n),
    .clk                     (clk),
    .mem_req_ack_stall_in    ('0),

    // Memory Interface
    .wbd_mem_stb_i         (wb_app_stb_o ) ,
    .wbd_mem_adr_i         (wb_app_adr_o ) ,
    .wbd_mem_we_i          (wb_app_we_o  ) ,
    .wbd_mem_dat_i         (wb_app_dat_o ) ,
    .wbd_mem_sel_i         (wb_app_sel_o ) ,
    .wbd_mem_bl_i          (wb_app_bl_o  ) ,
    .wbd_mem_dat_o         (wb_app_dat_i ) ,
    .wbd_mem_ack_o         (wb_app_ack_i ) ,
    .wbd_mem_lack_o        (wb_app_lack_i) ,
    .wbd_mem_err_o         (wb_app_err_i ) 

);








task wb_user_core_write;
input [31:0] address;
input [3:0]  be;
input [31:0] data;
begin
  repeat (1) @(posedge clk);
  #1;
  wb_cpu_adr_i =address;  // address
  wb_cpu_we_i  ='h1;  // write
  wb_cpu_dat_i =data;  // data output
  wb_cpu_sel_i =be;  // byte enable
  wb_cpu_cyc_i ='h1;  // strobe/request
  wb_cpu_stb_i ='h1;  // strobe/request
  wait(wb_cpu_ack_o == 1);
  repeat (1) @(posedge clk);
  #1;
  wb_cpu_cyc_i ='h0;  // strobe/request
  wb_cpu_stb_i ='h0;  // strobe/request
  wb_cpu_adr_i ='h0;  // address
  wb_cpu_we_i  ='h0;  // write
  wb_cpu_dat_i ='h0;  // data output
  wb_cpu_sel_i ='h0;  // byte enable
  $display("DEBUG WB USER ACCESS WRITE Address : %x, Mask: %x Data : %x",address,be,data);
  repeat (2) @(posedge clk);
end
endtask

task  wb_user_core_read;
input [31:0] address;
output [31:0] data;
reg    [31:0] data;
begin
  repeat (1) @(posedge clk);
  #1;
  wb_cpu_adr_i =address;  // address
  wb_cpu_we_i  ='h0;  // write
  wb_cpu_dat_i ='0;  // data output
  wb_cpu_sel_i ='hF;  // byte enable
  wb_cpu_cyc_i ='h1;  // strobe/request
  wb_cpu_stb_i ='h1;  // strobe/request
  wait(wb_cpu_ack_o == 1);
  data  = wb_cpu_dat_o;  
  repeat (1) @(posedge clk);
  #1;
  wb_cpu_cyc_i ='h0;  // strobe/request
  wb_cpu_stb_i ='h0;  // strobe/request
  wb_cpu_adr_i ='h0;  // address
  wb_cpu_we_i  ='h0;  // write
  wb_cpu_dat_i ='h0;  // data output
  wb_cpu_sel_i ='h0;  // byte enable
  $display("DEBUG WB USER ACCESS READ Address : %x, Data : %x",address,data);
  repeat (2) @(posedge clk);
end
endtask

task  wb_user_core_read_check;
input [31:0] address;
output [31:0] data;
input [31:0] cmp_data;
reg    [31:0] data;
begin
  repeat (1) @(posedge clk);
  #1;
  wb_cpu_adr_i =address;  // address
  wb_cpu_we_i  ='h0;  // write
  wb_cpu_dat_i ='0;  // data output
  wb_cpu_sel_i ='hF;  // byte enable
  wb_cpu_cyc_i ='h1;  // strobe/request
  wb_cpu_stb_i ='h1;  // strobe/request
  wait(wb_cpu_ack_o == 1);
  data  = wb_cpu_dat_o;  
  repeat (1) @(posedge clk);
  #1;
  wb_cpu_cyc_i ='h0;  // strobe/request
  wb_cpu_stb_i ='h0;  // strobe/request
  wb_cpu_adr_i ='h0;  // address
  wb_cpu_we_i  ='h0;  // write
  wb_cpu_dat_i ='h0;  // data output
  wb_cpu_sel_i ='h0;  // byte enable
  if(data !== cmp_data) begin
     $display("ERROR : WB USER ACCESS READ  Address : 0x%x, Exd: 0x%x Rxd: 0x%x ",address,cmp_data,data);
     test_fail = 1;
  end else begin
     $display("STATUS: WB USER ACCESS READ  Address : 0x%x, Data : 0x%x",address,data);
  end
  repeat (2) @(posedge clk);
end
endtask



endmodule


