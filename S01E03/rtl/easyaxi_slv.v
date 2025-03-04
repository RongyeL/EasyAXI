// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_slv.v
// Author        : Rongye
// Created On    : 2025-02-06 06:52
// Last Modified : 2025-03-02 07:40
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
    input  wire  [`AXI_ID_WIDTH  -1:0]       axi_slv_arid,
    input  wire  [`AXI_ADDR_WIDTH-1:0]       axi_slv_araddr
);
localparam DLY       = 0.1;
localparam CLR_CNT_W = 4;     // Clear counter width
localparam REG_ADDR  = 16'h0000;  // Default register address
//--------------------------------------------------------------------------------
// Inner Signal
//--------------------------------------------------------------------------------
reg                          valid_buff_r;
reg  [`AXI_ID_WIDTH   -1:0]  id_buff_r;   
reg  [`AXI_ADDR_WIDTH -1:0]  addr_buff_r; 

wire                         buff_full;   

reg  [CLR_CNT_W       -1:0]  clr_cnt_r;   
wire                         clr;         
//--------------------------------------------------------------------------------
// Main Ctrl
//--------------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        valid_buff_r <= #DLY 1'b0;
    end
    else if (clr) begin
        valid_buff_r <= #DLY 1'b0;        // Clear buffer
    end
    else if (axi_slv_arvalid & axi_slv_arready) begin
        valid_buff_r <= #DLY 1'b1;        // Latch valid on handshake
    end
end
assign buff_full = &valid_buff_r;          // All bits set = full

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        id_buff_r   <= #DLY {`AXI_ID_WIDTH{1'b0}};
        addr_buff_r <= #DLY {`AXI_ADDR_WIDTH{1'b0}};
    end
    else if (clr) begin
        id_buff_r   <= #DLY {`AXI_ID_WIDTH{1'b0}};  // Clear stored ID
        addr_buff_r <= #DLY {`AXI_ADDR_WIDTH{1'b0}};// Clear stored address
    end
    else if (axi_slv_arvalid & axi_slv_arready) begin
        id_buff_r   <= #DLY axi_slv_arid;    // Capture incoming ID
        addr_buff_r <= #DLY axi_slv_araddr;  // Capture incoming address
    end
end
assign dec_miss = (axi_slv_araddr != REG_ADDR);  // Address decode miss

assign clr = clr_cnt_r == {CLR_CNT_W{1'b1}};  // Clear when counter max
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        clr_cnt_r <= #DLY {CLR_CNT_W{1'b0}};
    end
    else if (clr) begin
        clr_cnt_r <= #DLY {CLR_CNT_W{1'b0}};  // Reset counter
    end
    else if (axi_slv_arvalid & axi_slv_arready) begin
        clr_cnt_r <= #DLY clr_cnt_r + 1;      // Increment on handshake
    end
    else if (clr_cnt_r != {CLR_CNT_W{1'b0}}) begin
        clr_cnt_r <= #DLY clr_cnt_r + 1;      // Auto-increment if non-zero
    end
end

//--------------------------------------------------------------------------------
// Output Signal
//--------------------------------------------------------------------------------
assign axi_slv_arready = (~buff_full) & (~dec_miss);  // Ready when buffer available & address match

endmodule
