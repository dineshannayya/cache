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
////                                                              
////  instruction cache top                                                   
////                                                              
////  This file is part of the riscduino cores project            
////  https://github.com/dineshannayya/riscduino.git              
////  
////                                                              
////  Description                                                 
////
////    This a 16-way set associative cache module. 
//// 	The cache can be used between a CPU Instruction Memory and Main Memory or
//// 	Secondary Cache. 
//// 	The cache uses following policies:
//// 	1. Module is configured to be used in Look-through Architecture
//// 	2. The module implements Fist In First Out (FIFO) policy for eviction of valid 
////       i.e if the cache fill order is 0,1,2,3,4,5,5,6,7,8 ...  During the
////       cache refill will be over-written in same order 0,1,2,3 ...
////
//// 	   The module inferres 2KB cache Memory and 16 Location TAG Memory
//// 	   The TAG RAM stores tag data as well as Valid, Dirty Bits.
////
//// 	   The Most significant bit is Valid bit in tag data
//// 	   block, and all other are TAG data bits.
//// 		----------------------------------------------------------------------
//// 		|Valid bit |        Tag Data
//// 		----------------------------------------------------------------------
////
//// 	States of Finite State Machine implemented for Cache
//// 	Controller and its basic explanation:
////    1. PREFILL: During power up, with prefill command, complete
////                2KB Cache memory will be refreshed with applicaton
////                memory and corresponding tag memory is updated

////
//// 	2. IDLE: This is a reset/default state of FSM. It waits for
//// 	         read or write command from CPU and when receives any,
//// 	         it latches the address and move to TAG_COMPARE state
////
//// 	3. TAG_COMPARE: 
////             This is a decision state for tag hit or miss in cache. 
////             If hit, 
////                  B1. As this interface only support read access.
////                      A2. Read the cache memory offset location.
////                      B2. Move to IDLE
////             If No hit, It moved to refill state.
////    4. CACHE_REFILL:
//// 		In this state, data will be filled from Main memory
////            to cache memory. 
////                  A1. As current cpu access is read, cache memory will be
////                      loaded with main memory data and once cpu request
////                      address is available data will be fed back to cpu
////                       with ack 
////   5.NEXT_CACHE_REFILL_REQ : If the next tag data is not available in
////            cache & application fsm is free, then put request for next tag
////            location prefetech. This can be disabled by cfg_ntag_pfet_dis
////
//// Assumptions:
////            1. Wishbone Support Burst Read access. 
////               To support is additional two signal added to wishbone i/f
////                  *_bl  - 8 Bit - Indicate Burst Word, 1 Indicate 1 word
////                          FF - 255 word
////                  *_lack- Indicate last ack of the busrt ack
////                  *_ack - Ack will be asserted for completion of each
////                  valid access
//// Memory organization
////       TAG Memory:
////              A1. Each location hold  20 bit of [26:7] cpu address
////              A2. Each location also has Valid bit 
////              A3. Dirtly bit indicate cache memory is locally modified. need
////                  to write back data during cache location flush.
////              A4. There are 16 Tag location corresponds to 16 cache line
////       Cache Memory: 2KB SRAM or 512 Word SRAM
////             16 Cache Line * 32 Cache Word = 512 Word = 2048 Byte
////             Cache Address : <tag offset[3:0]> <cache ptr[4:0]> 
////
//// CPU address decoding:
////      [1:0]   -  32 Bit Word
////      [6:2]   -  32 Cache Word
////      [26:7]  -  Tag comparsion
////      [31:27] - Unused
////      
////      With [26:0] access, cache can address up to 128MB Memory Space.
////
////  Note: Skywater SRAM has two port
////     port-0: Support both Write and Read - This port used for cache write
////             back and Refill purpose
////     port-1: Support Read access only - This port used to read tag hit
////             cache location data
////  To Do:                                                      
////    nothing
////                                                              
////  Author(s):                                                  
////      - Dinesh Annayya, dinesha@opencores.org                 
////                                                              
////  Revision :                                                  
////    0.1 - 19th Jan 2022, Dinesh A                             
////           Working initial version
////
//// ******************************************************************************************************

`include "cache_defs.svh"

module icache_top #(
	 parameter WB_AW      = 32,
	 parameter WB_DW      = 32,
	 parameter TAG_MEM_WD = 21, // Valid + Tag
	 parameter TAG_MEM_DP = 16,
         parameter CACHELINES = 16, // 16 Cache Line
         parameter CACHESIZE  = 32 // Each cacheline has  32 Word
        ) (
	input logic			   mclk,	  //Clock input 
	input logic			   rst_n,	  //Active Low Asynchronous Reset Signal Input

	input logic                        cfg_pfet_dis,      // To disable Next Pre data Pre fetch, default = 0
	input logic                        cfg_ntag_pfet_dis, // To disable next Tag refill, default = 0

	// Wishbone CPU I/F
        input logic                        wb_cpu_stb_i, // strobe/request
        input logic   [WB_AW-1:0]          wb_cpu_adr_i, // address
        input logic                        wb_cpu_we_i,  // write
        input logic   [WB_DW-1:0]          wb_cpu_dat_i, // data output
        input logic   [3:0]                wb_cpu_sel_i, // byte enable

        output logic   [WB_DW-1:0]         wb_cpu_dat_o, // data input
        output logic                       wb_cpu_ack_o, // acknowlegement
        output logic                       wb_cpu_err_o,  // error

	// Wishbone CPU I/F
        output logic                       wb_app_stb_o, // strobe/request
        output logic   [WB_AW-1:0]         wb_app_adr_o, // address
        output logic                       wb_app_we_o,  // write
        output logic   [WB_DW-1:0]         wb_app_dat_o, // data output
        output logic   [3:0]               wb_app_sel_o, // byte enable
        output logic   [9:0]               wb_app_bl_o,  // Burst Length

        input logic   [WB_DW-1:0]          wb_app_dat_i, // data input
        input logic                        wb_app_ack_i, // acknowlegement
        input logic                        wb_app_lack_i,// last acknowlegement
        input logic                        wb_app_err_i  // error


);

// Parameters

// Total cache memory = 16 * 32 * 4 = 2048 (2KB)

// State Machine Parameters

localparam	IDLE		         = 4'd0,	//Please read Description for explanation of States and their operation
		TAG_COMPARE	         = 4'd1,
		CACHE_RDATA_FETCH1       = 4'd2,
		CACHE_RDATA_FETCH2       = 4'd3,
		CACHE_RDATA_FETCH3       = 4'd4,
		PREFETCH_WAIT            = 4'd5,
		CACHE_REFILL_WAIT        = 4'd6,
		CACHE_REFILL_DONE        = 4'd7,
		CACHE_PREFILL_WAIT       = 4'd8,
		CACHE_PREFILL_DONE       = 4'd9,
		NEXT_CACHE_REFILL_REQ    = 4'd10;

// CACHE SRAM Memory I/F
logic                             cache_mem_clk0           ; // CLK
logic                             cache_mem_csb0           ; // CS#
logic                             cache_mem_web0           ; // WE#
logic   [8:0]                     cache_mem_addr0          ; // Address
logic   [3:0]                     cache_mem_wmask0         ; // WMASK#
logic   [31:0]                    cache_mem_din0           ; // Write Data
logic   [31:0]                    cache_mem_dout0          ; // Read Data

// SRAM-0 PORT-1, IMEM I/F
logic                             cache_mem_clk1           ; // CLK
logic                             cache_mem_csb1           ; // CS#
logic  [8:0]                      cache_mem_addr1          ; // Address
logic  [31:0]                     cache_mem_dout1          ; // Read Data

// Tag Memory Wire decleration
logic 	                          tag_wr                   ; // Tag Write Indication
logic 	                          tag_uwr                  ; // Tag Update
logic [$clog2(TAG_MEM_DP)-1:0]	  tag_uptr                 ; // Tag Memory Write Update Location
type_icache_tag_mem_s 	          tag_wdata                ; 
logic [$clog2(TAG_MEM_DP)-1:0]    tag_cur_loc              ; // Tag Memory Write Current Location
logic                             tag_hdirty               ; // Hit location Dirty indication
logic                             tag_full                 ;


logic  [`TAG_XLEN-1:0]            tag_cmp_data             ; // Tag Compare Data
logic [TAG_MEM_DP-1:0]            tag_hit                  ; // Tag Compare Hit 
logic [TAG_MEM_DP-1:0]            tag_next_hit             ; // Next Tag Compare Hit 
logic [$clog2(TAG_MEM_DP)-1:0]    tag_hindex               ; // Tag Hit Index
logic                             tag_cdirty               ; // Current location Dirty indication
logic  [`TAG_XLEN-1:0]            tag_ctag                 ; // Tag Compare Data

logic [$clog2(CACHESIZE)-1:0]     cache_mem_ptr            ; // Cache Memory Pointer


// Internal Signals derived from respective data or address buses
logic	                          cache_hit                ;
logic	                          cache_next_hit           ;


logic [WB_AW-1:0]                cpu_addr                  ;
logic [WB_AW-1:0]                cache_refill_addr         ;

logic   [WB_DW-1:0]               prefetch_data            ; // Additional Prefetch on next location of current location
logic [$clog2(CACHESIZE)-1:0]     prefetch_ptr             ; // Prefetch Ptr
logic [$clog2(TAG_MEM_DP)-1:0]    prefetch_index           ; // Prefetch Index
logic                             prefetch_val             ;

logic   [WB_DW-1:0]               cache_mem_hdata          ; // Additional cache hold data
logic                             cache_mem_hval           ; // Holding Additional cache data valid

logic   [WB_DW-1:0]               wb_cpu_dat1_o            ; // data input
logic                             wb_cpu_ack1_o            ; // acknowlegement

logic                             cache_refill_req         ; // Request for Refill of 32 location
logic                             cache_prefill_req        ; // Request for complete prefill 32 x 16
logic                             cache_busy               ; // Request for complete prefill 32 x 16


// State Variables
reg [3:0] state;

// Combinational Logic

assign tag_cmp_data = cpu_addr[26:7];
assign cache_hit = |tag_hit;
assign cache_next_hit = |tag_next_hit;

wire [$clog2(CACHESIZE)-1:0]  next_prefetch_ptr = prefetch_ptr[4:0] + 1;

// Cache Controller State Machine and Logic

always@(posedge mclk or negedge rst_n)
begin
   if(!rst_n)
   begin
      wb_cpu_dat_o      <= '0;
      wb_cpu_ack_o      <= 1'b0;

      cache_mem_addr1   <= '0;
      cache_mem_csb1    <= 1'b1;

      cache_mem_ptr     <= '0;

      cpu_addr          <= '0;

      prefetch_data     <= '0;
      prefetch_ptr      <= '0;
      prefetch_index    <= '0;
      prefetch_val      <= '0;
      cache_mem_hval    <= 1'b0;
      cache_mem_hdata   <= '0;
      cache_refill_req  <= '0;
      cache_prefill_req <= '1;
      cache_refill_addr <= '0;

      state             <= CACHE_PREFILL_WAIT;

   end else begin
      case(state)
      IDLE	:begin

	 cache_mem_ptr     <= '0;


	// Check if the current address is next location of same cache offset
	// if yes, pick the data from prefetch content
	 if(!cfg_pfet_dis && wb_cpu_stb_i && !wb_cpu_we_i && prefetch_val && 
	     (wb_cpu_adr_i[31:2] == {cpu_addr[31:7], prefetch_ptr[4:0]})) begin
	     // Ack with Prefect data
              wb_cpu_dat_o     <= prefetch_data;
	      wb_cpu_ack_o     <= 1'b1;

	      // Goahead for next data prefetech in same cache index
	      cache_mem_addr1  <= {prefetch_index,next_prefetch_ptr[4:0]}; // Address for additional prefetch;
	      cache_mem_csb1   <= 1'b0;
	      prefetch_ptr     <= next_prefetch_ptr;
	      state            <= PREFETCH_WAIT;

         end else begin
	    wb_cpu_ack_o      <= 1'b0;
	    cache_mem_addr1   <= '0;
	    cache_mem_csb1    <= 1'b1;

	    if(wb_cpu_stb_i && !wb_cpu_ack_o) begin
	        cpu_addr         <= wb_cpu_adr_i;
		prefetch_val     <= 1'b0;
	        state            <= TAG_COMPARE;
	    end else if(!cfg_ntag_pfet_dis && !cache_next_hit && !cache_busy) begin 
	    // If there is no Next Tag Hit and cache fsm is free, the give
            // additional Next Tag Pre-fetech request
	       cache_refill_req         <= 1;
	       cache_refill_addr[31:27] <= cpu_addr[31:27];
	       cache_refill_addr[26:7]  <= tag_cmp_data+1;
	       cache_refill_addr[6:0]   <= '0;
	       state                    <= NEXT_CACHE_REFILL_REQ;
	     end
	 end
      end

      TAG_COMPARE	:begin
         case(cache_hit)
	 1'd0:begin // If there is no Tag Hit
	      if(cache_busy == 0) begin
	        cache_refill_req  <= 1;
		cache_refill_addr <= cpu_addr;
	        state             <= CACHE_REFILL_WAIT;
	     end
         end

	 1'd1:	begin
	      cache_mem_addr1  <= {tag_hindex,cpu_addr[6:2]};
	      prefetch_index   <= tag_hindex;
	      prefetch_ptr     <=  cpu_addr[6:2]+1;
	      cache_mem_csb1    <= 1'b0;
	      state             <= CACHE_RDATA_FETCH1; // Read Cache
	  end
	  endcase
       end
       CACHE_RDATA_FETCH1: begin
	  cache_mem_addr1  <= {prefetch_index,prefetch_ptr}; // Address for additional prefetch;
	  state            <= CACHE_RDATA_FETCH2;
       end

       CACHE_RDATA_FETCH2: begin
	  cache_mem_csb1   <= 1'b1;
          wb_cpu_dat_o     <= cache_mem_dout1;
	  wb_cpu_ack_o     <= 1'b1;
	  state            <= CACHE_RDATA_FETCH3;
       end
       // Do Additial prefetech for next location
       CACHE_RDATA_FETCH3: begin
          prefetch_data    <= cache_mem_dout1;
          prefetch_val     <= 1'b1;
	  wb_cpu_ack_o     <= 1'b0;
	  state            <= IDLE;
       end

       // Additional Prefetch delay do to RAM access is take effectly two
       // cycle
       PREFETCH_WAIT: begin
	  wb_cpu_ack_o     <= 1'b0;
	  cache_mem_csb1   <= 1'b1;
	  state            <= CACHE_RDATA_FETCH3;
      end

      CACHE_REFILL_WAIT: begin
	  if(cache_busy == 1) begin
	     cache_refill_req <= 0;
	     state       <= CACHE_REFILL_DONE;
	  end
      end

     CACHE_REFILL_DONE: begin
	  // Copy the snoop details from cache fsm
	  wb_cpu_ack_o     <= wb_cpu_ack1_o;
	  wb_cpu_dat_o     <= wb_cpu_dat1_o;
	  if(cache_busy == 0) begin
	     state       <= IDLE;
	  end
      end

      CACHE_PREFILL_WAIT: begin
	  if(cache_busy == 1) begin
	     cache_prefill_req <= 0;
	     state       <= CACHE_PREFILL_DONE;
	  end
      end

     CACHE_PREFILL_DONE: begin
	  // Copy the snoop details from cache fsm
	  if(cache_busy == 0) begin
	     state       <= IDLE;
	  end
      end

      // Wait for Req accepted by application fsm
      NEXT_CACHE_REFILL_REQ: begin
	  if(cache_busy == 1) begin
	     cache_refill_req <= 0;
	     state            <= IDLE;
	  end
      end


       default:begin
          wb_cpu_dat_o      <= '0;
          wb_cpu_ack_o      <= 1'b0;
       
          cache_mem_addr1   <= '0;
          cache_mem_csb1    <= 1'b1;
       
          cache_mem_ptr     <= '0;
       
          cpu_addr          <= '0;
       
          state             <= IDLE;
       end
       endcase
   end
end


icache_app_fsm  #(
	 .WB_AW      (WB_AW     ) ,
	 .WB_DW      (WB_DW     ) ,
	 .TAG_MEM_WD (TAG_MEM_WD) ,
	 .TAG_MEM_DP (TAG_MEM_DP) ,
         .CACHELINES (CACHELINES) , // 16 Cache Line
         .CACHESIZE  (CACHESIZE )  // Each cacheline has  32 Word

        ) u_app_fsm (
	.mclk                         (mclk                ),	  //Clock input 
	.rst_n                        (rst_n               ),	  //Active Low Asynchronous Reset Signal Input

	// Wishbone CPU I/F
        .cpu_addr                     (cache_refill_addr   ),     // address
                                                   
        .wb_cpu_dat_o                 (wb_cpu_dat1_o       ), // data input
        .wb_cpu_ack_o                 (wb_cpu_ack1_o       ), // acknowlegement

	// Wishbone Application I/F
        .wb_app_stb_o                 (wb_app_stb_o        ), // strobe/request
        .wb_app_adr_o                 (wb_app_adr_o        ), // address
        .wb_app_we_o                  (wb_app_we_o         ), // write
        .wb_app_dat_o                 (wb_app_dat_o        ), // data output
        .wb_app_sel_o                 (wb_app_sel_o        ), // byte enable
        .wb_app_bl_o                  (wb_app_bl_o         ), // Burst Length
                                                    
        .wb_app_dat_i                 (wb_app_dat_i        ), // data input
        .wb_app_ack_i                 (wb_app_ack_i        ), // acknowlegement
        .wb_app_lack_i                (wb_app_lack_i       ), // last acknowlegement
                                                    
        .tag_cur_loc                  (tag_cur_loc         ), // Tag Memory Write Current Location
        .tag_wr                       (tag_wr              ), // Tag Write Indication
        .tag_uwr                      (tag_uwr             ), // Tag Update
        .tag_uptr                     (tag_uptr            ), // Tag Memory Write Update Location
        .tag_wdata                    (tag_wdata           ), 
	
	// CACHE SRAM Memory I/F
        .cache_mem_clk0               (cache_mem_clk0      ), // CLK
        .cache_mem_csb0               (cache_mem_csb0      ), // CS#
        .cache_mem_web0               (cache_mem_web0      ), // WE#
        .cache_mem_addr0              (cache_mem_addr0     ), // Address
        .cache_mem_wmask0             (cache_mem_wmask0    ), // WMASK#
        .cache_mem_din0               (cache_mem_din0      ), // Write Data
                                                           
        .cache_refill_req             (cache_refill_req    ),
        .cache_prefill_req            (cache_prefill_req   ),
	.cache_busy                   (cache_busy          )


     );



// Tag Memory
icache_tag_fifo #(.WD(TAG_MEM_WD), .DP(TAG_MEM_DP)) u_tag_fifo ( 
	.clk                 (mclk),
	.reset_n             (rst_n),
	.flush               (1'b0),

        .tag_wr              (tag_wr),
	.tag_uwr             (tag_uwr),
	.tag_uptr            (tag_uptr),
	.tag_wdata           (tag_wdata),
	.tag_wptr            (tag_cur_loc),


	.tag_cmp_data        (tag_cmp_data),
        .tag_hit             (tag_hit),
        .tag_next_hit        (tag_next_hit),
	.tag_hindex          (tag_hindex),
	.tag_ctag            (tag_ctag),


	.full                (tag_full),
	.empty               ()
	  );

// 2KB SRAM Cache memory
sky130_sram_2kbyte_1rw1r_32x512_8 u_cmem_2kb(
`ifdef USE_POWER_PINS
    .vccd1 (vccd1),// User area 1 1.8V supply
    .vssd1 (vssd1),// User area 1 digital ground
`endif
// Port 0: RW
    .clk0     (mclk),
    .csb0     (cache_mem_csb0),
    .web0     (cache_mem_web0),
    .wmask0   (cache_mem_wmask0),
    .addr0    (cache_mem_addr0),
    .din0     (cache_mem_din0),
    .dout0    (cache_mem_dout0),
// Port 1: R
    .clk1     (mclk),
    .csb1     (cache_mem_csb1),
    .addr1    (cache_mem_addr1),
    .dout1    (cache_mem_dout1)
  );

// END OF MODULE
endmodule

