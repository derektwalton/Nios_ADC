module adc_ctrl
  #(
   parameter SRAM_SIZE = 1*1024
   )
  ( 
    // Avalon-MM agent interface
    input logic 	clk_i,
    input logic 	rst_ni,
    input logic [7:0] 	address_i,
    input logic [3:0] 	byteenable_i,
    input logic 	read_i,
    output logic [31:0] readdata_o,
    input logic 	write_i,
    input logic [31:0] 	writedata_i,

    // ADC interface
    output logic 	ad9226_clk_o,
    input logic 	ad9226_otr_i,
    input logic [11:0] 	ad9226_d_i
    );
   

   //
   // Base clock for ADC sampling.  For now just use clk_i but we may wan't to create a 2nd independent clock for this
   // from a PLL.  clk_i appears to be setup to run at 100 MHz.
   //
   
   wire adc_clk_i;
   assign adc_clk_i = clk_i;
   
   
   //
   // For now, upon initiation of a capture, store samples from ADC in an on-chip SRAM.  Software can then read samples
   // from this SRAM.
   //
   // The ad9226_clk_o output is currently setup to toggle every rising edge of adc_clk_i (= clk_i).  Thus, we will be
   // running the ADC at 50 MHz sample rate.
   //
   // Speed of sound in water is around 1481 m/s.  A sample rate of 50 MHz cooresponds to a sample period of 20 nSec,
   // and a travel distance of 0.029 mm = 1481 m/s * 1000 mm/m * 20e-9 sec.  With a buffer size of 1K samples, we can
   // store samples associated with a travel distance of 29 mm = 1.14".
   //

   typedef struct packed {
      logic 		otr;
      logic [11:0] 	d;
   } ADC_SAMPLE_t;
   
   ADC_SAMPLE_t sram[SRAM_SIZE-1:0];
   
   logic [$clog2(SRAM_SIZE)-1:0] sram_rd_ptr_q, sram_wr_ptr_q;
   

   //
   // CSRs (control / state registers)
   //

   typedef logic [7:0] RA_t;
   localparam RA_t RA_control = 0;
   localparam RA_t RA_status = 1;
   localparam RA_t RA_delay = 2;
   localparam RA_t RA_data = 3;
   
   typedef struct packed {
      logic [1:0] sample_timing;
      logic capture;                 // 0 to 1 transition initiates a capture
   } CSR_CONTROL_t;
   
   typedef struct packed {
      logic capture_done;
   } CSR_STATUS_t;

   typedef logic [15:0] CSR_DELAY_t; // number of ADC cycles to delay before beginning capture

   CSR_CONTROL_t csr_control_q;
   CSR_STATUS_t csr_status_q;   
   CSR_DELAY_t csr_delay_q;  
   ADC_SAMPLE_t csr_data_q;
      
   
   // create csr write data from current register contents and Avalon bus write data and write enables
   function automatic logic [31:0] csr_write_data(input logic [31:0] d_reg, input logic [31:0] d_ava, input logic [3:0] be);
      logic [31:0] q;
      q[0+:8]  = be[0] ? d_ava[0+:8]  : d_reg[0+:8];
      q[8+:8]  = be[1] ? d_ava[8+:8]  : d_reg[8+:8];
      q[16+:8] = be[2] ? d_ava[16+:8] : d_reg[16+:8];
      q[24+:8] = be[3] ? d_ava[24+:8] : d_reg[24+:8];
      return q;
   endfunction

   // csr write
   always @(posedge clk_i) begin
      if (!rst_ni) begin
	 csr_control_q <= 0;
	 csr_delay_q <= 0;	 
      end
      else if (write_i) begin
	 case (address_i)
	   RA_control: csr_control_q <= csr_write_data(csr_control_q, writedata_i, byteenable_i);
	   RA_delay:   csr_delay_q   <= csr_write_data(csr_delay_q,   writedata_i, byteenable_i);
	 endcase
      end
   end

   // csr read
   always_comb begin
      readdata_o = '0;
      case (address_i)
	 RA_control: readdata_o = csr_control_q;
	 RA_status:  readdata_o = csr_status_q;
	 RA_delay:   readdata_o = csr_delay_q;
	 RA_data:    readdata_o = csr_data_q;
      endcase
   end

   // detect 0->1 transition on csr_control.capture to know when to start a capture
   logic start;
   CSR_CONTROL_t csr_control_p1q;
   always @(posedge clk_i) begin
      csr_control_p1q <= csr_control_q;
   end
   assign start = csr_control_q.capture && !csr_control_p1q.capture;

   // reset SRAM read pointer on start, and increment each time host reads CSR_DATA register
   always @(posedge clk_i) begin
      if (!rst_ni || start) begin
	 sram_rd_ptr_q <= '0;
      end
      else if (read_i && address_i==RA_data) begin
	 sram_rd_ptr_q <= sram_rd_ptr_q + 1;      
      end
   end

   // Synchronous read of SRAM on clk_i
   always_ff @(posedge clk_i) begin
      csr_data_q <= sram[sram_rd_ptr_q];
   end


   //
   // Synchronize reset to adc_clk_i clock domain.
   //

   logic rst_adc_p1q;
   logic rst_adc_p2q;
   logic rst_adc_n;
   always_ff @(posedge adc_clk_i) begin
      rst_adc_p1q <= rst_ni;
      rst_adc_p2q <= rst_adc_p1q;
   end
   assign rst_adc_n = rst_adc_p2q;

   
   //
   // Synchronize csr_control.capture to adc_clk_i clock domain and again perform 0->1 edge detect to determine 
   // when to start ADC capture
   //

   logic capture_adc_p1q;
   logic capture_adc_p2q;
   logic capture_adc_p3q;
   always_ff @(posedge adc_clk_i) begin
      capture_adc_p1q <= csr_control_q.capture;
      capture_adc_p2q <= capture_adc_p1q;
      capture_adc_p3q <= capture_adc_p2q;
   end
   
   logic start_adc;
   assign start_adc = capture_adc_p2q && !capture_adc_p3q;
   

   //
   // ADC interface statemachine ... operates asynchronously on adc_clk_i.
   //

   typedef enum logic [1:0] {
     ADC_STATE_active0,
     ADC_STATE_active1,
     ADC_STATE_done
   } ADC_STATE_e;
   
   ADC_STATE_e adc_state_q;
   logic [15:0] delay_q;
   
   always_ff @(posedge adc_clk_i) begin
      if (!rst_adc_n) begin
	 ad9226_clk_o <= 0;	 
	 delay_q <= '0;
	 sram_wr_ptr_q <= '0;
	 adc_state_q <= ADC_STATE_done;
      end
      else if (start_adc) begin
	 ad9226_clk_o <= 0;	 
	 delay_q <= csr_delay_q;
	 sram_wr_ptr_q <= '0;
	 adc_state_q <= ADC_STATE_active0;
      end
      else begin
	 case (adc_state_q)
	   ADC_STATE_active0: begin
	      ad9226_clk_o <= 1;
	      adc_state_q <= ADC_STATE_active1;
	   end
	   ADC_STATE_active1: begin
	      ad9226_clk_o <= 0;
	      if (delay_q==0) begin
		 sram_wr_ptr_q <= sram_wr_ptr_q + 1;
	      end
	      else begin
		 delay_q <= delay_q - 1;
	      end
	      adc_state_q <= sram_wr_ptr_q=={$clog2(SRAM_SIZE){1'b1}} ? ADC_STATE_done : ADC_STATE_active0;
	   end
	   ADC_STATE_done: begin	      
	   end
	 endcase
      end
   end

   // sample ADC outputs at various times relative to ad9226_clk_o.  we will experimentally determine
   // which yields clean sampling of the ADC data (ie. when the signals are valid and stable).
   ADC_SAMPLE_t sample_r0_q, sample_r1_q;   
   always_ff @(posedge adc_clk_i) begin
      sample_r0_q <= {ad9226_otr_i, ad9226_d_i};
      sample_r1_q <= sample_r0_q;
   end
   ADC_SAMPLE_t sample_f0_q, sample_f1_q;   
   always_ff @(negedge adc_clk_i) begin
      sample_f0_q <= {ad9226_otr_i, ad9226_d_i};
      sample_f1_q <= sample_f0_q;
   end
   ADC_SAMPLE_t sample_q;   
   always_ff @(posedge adc_clk_i) begin
      case(csr_control_q.sample_timing) 
	0: sample_q <= sample_r0_q;
	1: sample_q <= sample_r1_q;
	2: sample_q <= sample_f0_q;
	3: sample_q <= sample_f1_q;
      endcase
   end
   
   // write sample to the SRAM
   always_ff @(posedge adc_clk_i) begin
      if ( adc_state_q==ADC_STATE_active1 && delay_q==0 ) begin
	 sram[sram_wr_ptr_q] <= sample_q;
      end
   end

   
   //
   // Synchronize done indication back to the Avalon bus logic.
   //

   logic done_p1q, done_p2q;
   always_ff @(posedge clk_i) begin
      done_p1q <= adc_state_q==ADC_STATE_done;
      done_p2q <= done_p1q;
      csr_status_q.capture_done <= done_p2q;
   end

endmodule
