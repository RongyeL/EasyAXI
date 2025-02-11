// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_mst.v
// Author        : Rongye
// Created On    : 2025-02-06 06:45
// Last Modified : 2025-02-08 23:55
// ---------------------------------------------------------------------------------
// Description   : When the enable signal is valid, it triggers the valid signal to be set.  
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_MST (
// Global
    input  wire                              clk,
    input  wire                              rst_n, 
    input  wire                              enable,

// AXI AR Channel
    output wire                              axi_mst_arvalid,
    input  wire                              axi_mst_arready,
    output wire  [`AXI_ADDR_WIDTH-1:0]       axi_mst_araddr
);
localparam DLY                 = 0.1;
//--------------------------------------------------------------------------------
// Inner Signal
//--------------------------------------------------------------------------------
reg                       axi_mst_arvalid_r;
reg [`AXI_ADDR_WIDTH-1:0] axi_mst_araddr_r;

//--------------------------------------------------------------------------------
// Main Ctrl
//--------------------------------------------------------------------------------

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        axi_mst_arvalid_r <= #DLY 1'b0;
    end
    else if (~axi_mst_arvalid_r & enable) begin
        axi_mst_arvalid_r <= #DLY 1'b1;
    end
    else if (axi_mst_arvalid & axi_mst_arready) begin
        axi_mst_arvalid_r <= #DLY 1'b0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        axi_mst_araddr_r <= #DLY {`AXI_ADDR_WIDTH{1'b0}};
    end
    else if (axi_mst_arvalid & axi_mst_arready) begin
        axi_mst_araddr_r <= #DLY axi_mst_araddr_r + 1;
    end
end



assign axi_mst_arvalid = axi_mst_arvalid_r;
assign axi_mst_araddr  = axi_mst_araddr_r;

endmodule
