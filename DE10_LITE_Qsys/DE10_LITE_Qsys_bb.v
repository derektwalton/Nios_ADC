
module DE10_LITE_Qsys (
	ad9226_writeresponsevalid_n,
	ad9226_writebyteenable_n,
	ad9226_beginbursttransfer,
	altpll_0_areset_conduit_export,
	altpll_0_locked_conduit_export,
	clk_clk,
	clk_sdram_clk,
	key_external_connection_export,
	reset_reset_n,
	sdram_wire_addr,
	sdram_wire_ba,
	sdram_wire_cas_n,
	sdram_wire_cke,
	sdram_wire_cs_n,
	sdram_wire_dq,
	sdram_wire_dqm,
	sdram_wire_ras_n,
	sdram_wire_we_n);	

	output		ad9226_writeresponsevalid_n;
	input	[11:0]	ad9226_writebyteenable_n;
	input		ad9226_beginbursttransfer;
	input		altpll_0_areset_conduit_export;
	output		altpll_0_locked_conduit_export;
	input		clk_clk;
	output		clk_sdram_clk;
	input	[3:0]	key_external_connection_export;
	input		reset_reset_n;
	output	[12:0]	sdram_wire_addr;
	output	[1:0]	sdram_wire_ba;
	output		sdram_wire_cas_n;
	output		sdram_wire_cke;
	output		sdram_wire_cs_n;
	inout	[15:0]	sdram_wire_dq;
	output	[1:0]	sdram_wire_dqm;
	output		sdram_wire_ras_n;
	output		sdram_wire_we_n;
endmodule
