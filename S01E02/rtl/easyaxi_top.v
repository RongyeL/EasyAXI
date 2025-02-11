// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi.v
// Author        : Rongye
// Created On    : 2025-02-05 05:04
// Last Modified : 2025-02-09 00:07
// ---------------------------------------------------------------------------------
// Description   : 
//
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_TOP (
    input wire clk,
    input wire rst_n,
    input wire enable 
);
wire                       axi_mst_arvalid;
wire                       axi_mst_arready;
wire [`AXI_ADDR_WIDTH-1:0] axi_mst_araddr;

EASYAXI_MST U_EASYAXI_MST (
    .clk             (clk             ), // i
    .rst_n           (rst_n           ), // i
    .enable          (enable          ), // i
    .axi_mst_arvalid (axi_mst_arvalid ), // o
    .axi_mst_arready (axi_mst_arready ), // i
    .axi_mst_araddr  (axi_mst_araddr  )  // o
);
wire                       axi_slv_arvalid;
wire                       axi_slv_arready;
wire [`AXI_ADDR_WIDTH-1:0] axi_slv_araddr;

EASYAXI_SLV U_EASYAXI_SLV (
    .clk             (clk             ), // i
    .rst_n           (rst_n           ), // i
    .enable          (enable          ), // i
    .axi_slv_arvalid (axi_slv_arvalid ), // i
    .axi_slv_arready (axi_slv_arready ), // o
    .axi_slv_araddr  (axi_slv_araddr  )  // i
);

assign axi_slv_arvalid = axi_mst_arvalid;
assign axi_mst_arready = axi_slv_arready;
assign axi_slv_araddr  = axi_mst_araddr ;

endmodule
