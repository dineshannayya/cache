

assign tag_cmp_data = imem_addr[26:7];
assign cache_hit    = |tag_hit;


always@(posedge mclk or negedge rst_n)
begin
   if(!rst_n)
   begin
      wb_imem_dat_o      <= '0;
      wb_imem_ack_o      <= 1'b0;

      cache_mem_addr1   <= '0;
      cache_mem_csb1    <= 1'b1;

      imem_addr          <= '0;

      state             <= IDLE;

   end else begin
      case(state)
      IDLE	:begin
	 wb_imem_dat_o    <= '0;
	 wb_imem_ack_o    <= 1'b0;

	 cache_mem_addr1  <= '0;
	 cache_mem_csb1   <= 1'b1;

	 imem_addr         <= wb_imem_adr_i;

	 if(wb_imem_stb)
	     state            <= TAG_COMPARE;
      end

      TAG_COMPARE	:begin
         case(cache_hit)
	 1'd0:begin // If there is no Tag Hit
	    cache_refill_req <= 1;
	    state            <= CACHE_REFILL_DONE;
	    end
         end

	 1'd1:	begin
	      cache_mem_addr1  <= {tag_hindex,imem_addr[6:2]};
	      cache_mem_csb1   <= 1'b0;
	      state            <= CACHE_DATA_FETCH;
	  end
	  endcase
       end
       CACHE_DATA_FETCH: begin
          wb_cpu_dat_o <= cache_mem_dout1;
	  wb_cpu_ack_o <= 1'b1;
	  state        <= IDLE;
       end
       CACHE_REFILL_DONE: begin
	   if(cache_refill_ack) begin
	      cache_mem_addr1  <= {tag_cindex,imem_addr[6:2]};
	      cache_mem_csb1   <= 1'b0;
	      state            <= IDLE;
	   end
      end
      endcase
   end
end

