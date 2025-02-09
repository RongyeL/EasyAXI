// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_slv.v
// Author        : Rongye
// Created On    : 2025-02-06 06:52
// Last Modified : 2025-02-09 00:01
// ---------------------------------------------------------------------------------
// Description   : 
//
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_SLV (
// Global
    input  wire                              clk,
    input  wire                              rst_n, 
    input  wire                              enable,

// AXI AR Channel
    input  wire                              axi_slv_arvalid,
    output wire                              axi_slv_arready,
    input  wire  [`AXI_ADDR_WIDTH-1:0]       axi_slv_araddr
);
localparam DLY = 0.1;
localparam CLR_CNT_W = 4;
//--------------------------------------------------------------------------------
// Inner Signal
//--------------------------------------------------------------------------------
reg                         valid_buff_r;
wire                        buff_full;
reg  [`AXI_ADDR_WIDTH-1:0]  addr_buff_r;

reg  [CLR_CNT_W      -1:0]  clr_cnt_r;
wire                        clr;
//--------------------------------------------------------------------------------
// Main Ctrl
//--------------------------------------------------------------------------------

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        valid_buff_r <= #DLY 1'b0;
    end
    else if (clr) begin
        valid_buff_r <= #DLY 1'b0;
    end
    else if (axi_slv_arvalid & axi_slv_arready) begin
        valid_buff_r <= #DLY 1'b1;
    end
end
assign buff_full = &valid_buff_r;

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        addr_buff_r <= #DLY {`AXI_ADDR_WIDTH{1'b0}};
    end
    else if (clr) begin
        addr_buff_r <= #DLY {`AXI_ADDR_WIDTH{1'b0}};
    end
    else if (axi_slv_arvalid & axi_slv_arready) begin
        addr_buff_r <= #DLY axi_slv_araddr;
    end
end

assign clr = clr_cnt_r == {CLR_CNT_W{1'b1}};
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        clr_cnt_r <= #DLY {CLR_CNT_W{1'b0}};
    end
    else if (clr) begin
        clr_cnt_r <= #DLY {CLR_CNT_W{1'b0}};
    end
    else if (axi_slv_arvalid & axi_slv_arready) begin
        clr_cnt_r <= #DLY clr_cnt_r + 1;
    end
    else if (clr_cnt_r != {CLR_CNT_W{1'b0}}) begin
        clr_cnt_r <= #DLY clr_cnt_r + 1;
    end
end


//--------------------------------------------------------------------------------
// Output Signal
//--------------------------------------------------------------------------------

assign axi_slv_arready = ~buff_full;

endmodule
