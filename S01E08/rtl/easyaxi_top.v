// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi.v
// Author        : Rongye
// Created On    : 2025-02-05 05:04
// Last Modified : 2025-12-03 13:04
// ---------------------------------------------------------------------------------
// Description   : 
//
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_TOP (
    input  wire clk,
    input  wire rst_n,
    input  wire rd_en, 
    output wire rd_done,
    input  wire wr_en, 
    output wire wr_done 
);
//--------------------------------------------------------------------------------
// Inst Master
//--------------------------------------------------------------------------------
wire                        axi_mst_arvalid;
wire                        axi_mst_arready;
wire  [`AXI_ID_W    -1:0]   axi_mst_arid;
wire  [`AXI_ADDR_W  -1:0]   axi_mst_araddr;
wire  [`AXI_LEN_W   -1:0]   axi_mst_arlen;
wire  [`AXI_SIZE_W  -1:0]   axi_mst_arsize;
wire  [`AXI_BURST_W -1:0]   axi_mst_arburst;
wire  [`AXI_USER_W  -1:0]   axi_mst_aruser;

wire                        axi_mst_rvalid;
wire                        axi_mst_rready;
wire  [`AXI_ID_W    -1:0]   axi_mst_rid;
wire  [`AXI_DATA_W  -1:0]   axi_mst_rdata;
wire  [`AXI_RESP_W  -1:0]   axi_mst_rresp;
wire                        axi_mst_rlast;
wire  [`AXI_USER_W  -1:0]   axi_mst_ruser;
EASYAXI_MST_RD_CTRL U_EASYAXI_MST_RD_CTRL (
    .clk             (clk             ), // i
    .rst_n           (rst_n           ), // i
    .rd_en           (rd_en           ), // i
    .rd_done         (rd_done         ), // o

    .axi_mst_arvalid (axi_mst_arvalid ), // o
    .axi_mst_arready (axi_mst_arready ), // i
    .axi_mst_arid    (axi_mst_arid    ), // o
    .axi_mst_araddr  (axi_mst_araddr  ), // o
    .axi_mst_arlen   (axi_mst_arlen   ), // o
    .axi_mst_arsize  (axi_mst_arsize  ), // o
    .axi_mst_arburst (axi_mst_arburst ), // o
    .axi_mst_aruser  (axi_mst_aruser  ), // o

    .axi_mst_rvalid  (axi_mst_rvalid  ), // i
    .axi_mst_rready  (axi_mst_rready  ), // o
    .axi_mst_rid     (axi_mst_rid     ), // i
    .axi_mst_rdata   (axi_mst_rdata   ), // i
    .axi_mst_rresp   (axi_mst_rresp   ), // i
    .axi_mst_rlast   (axi_mst_rlast   ), // i
    .axi_mst_ruser   (axi_mst_ruser   )  // i
);

wire                        axi_mst_awvalid;
wire                        axi_mst_awready;
wire  [`AXI_ID_W    -1:0]   axi_mst_awid;
wire  [`AXI_ADDR_W  -1:0]   axi_mst_awaddr;
wire  [`AXI_LEN_W   -1:0]   axi_mst_awlen;
wire  [`AXI_SIZE_W  -1:0]   axi_mst_awsize;
wire  [`AXI_BURST_W -1:0]   axi_mst_awburst;
wire  [`AXI_USER_W  -1:0]   axi_mst_awuser;

wire                        axi_mst_wvalid;
wire                        axi_mst_wready;
wire  [`AXI_DATA_W  -1:0]   axi_mst_wdata;
wire  [`AXI_DATA_W/8-1:0]   axi_mst_wstrb;
wire                        axi_mst_wlast;
wire  [`AXI_USER_W  -1:0]   axi_mst_wuser;

wire                        axi_mst_bvalid;
wire                        axi_mst_bready;
wire  [`AXI_ID_W    -1:0]   axi_mst_bid;
wire  [`AXI_RESP_W  -1:0]   axi_mst_bresp;
wire  [`AXI_USER_W  -1:0]   axi_mst_buser;
EASYAXI_MST_WR_CTRL U_EASYAXI_MST_WR_CTRL (
    .clk             (clk             ), // i
    .rst_n           (rst_n           ), // i
    .wr_en           (wr_en           ), // i
    .wr_done         (wr_done         ), // o

    .axi_mst_awvalid (axi_mst_awvalid ), // o
    .axi_mst_awready (axi_mst_awready ), // i
    .axi_mst_awid    (axi_mst_awid    ), // o
    .axi_mst_awaddr  (axi_mst_awaddr  ), // o
    .axi_mst_awlen   (axi_mst_awlen   ), // o
    .axi_mst_awsize  (axi_mst_awsize  ), // o
    .axi_mst_awburst (axi_mst_awburst ), // o
    .axi_mst_awuser  (axi_mst_awuser  ), // o

    .axi_mst_wvalid  (axi_mst_wvalid  ), // o
    .axi_mst_wready  (axi_mst_wready  ), // i
    .axi_mst_wdata   (axi_mst_wdata   ), // o
    .axi_mst_wstrb   (axi_mst_wstrb   ), // o
    .axi_mst_wlast   (axi_mst_wlast   ), // o
    .axi_mst_wuser   (axi_mst_wuser   ), // o

    .axi_mst_bvalid  (axi_mst_bvalid  ), // i
    .axi_mst_bready  (axi_mst_bready  ), // o
    .axi_mst_bid     (axi_mst_bid     ), // i
    .axi_mst_bresp   (axi_mst_bresp   ), // i
    .axi_mst_buser   (axi_mst_buser   )  // i
);
//--------------------------------------------------------------------------------
// Inst Slave
//--------------------------------------------------------------------------------
wire                        axi_slv_arvalid;
wire                        axi_slv_arready;
wire  [`AXI_ID_W    -1:0]   axi_slv_arid;
wire  [`AXI_ADDR_W  -1:0]   axi_slv_araddr;
wire  [`AXI_LEN_W   -1:0]   axi_slv_arlen;
wire  [`AXI_SIZE_W  -1:0]   axi_slv_arsize;
wire  [`AXI_BURST_W -1:0]   axi_slv_arburst;
wire  [`AXI_USER_W  -1:0]   axi_slv_aruser;

wire                        axi_slv_rvalid;
wire                        axi_slv_rready;
wire  [`AXI_ID_W    -1:0]   axi_slv_rid;
wire  [`AXI_DATA_W  -1:0]   axi_slv_rdata;
wire  [`AXI_RESP_W  -1:0]   axi_slv_rresp;
wire                        axi_slv_rlast;
wire  [`AXI_USER_W  -1:0]   axi_slv_ruser;

EASYAXI_SLV_RD_CTRL U_EASYAXI_SLV_RD_CTRL (
    .clk             (clk             ), // i
    .rst_n           (rst_n           ), // i
    .axi_slv_arvalid (axi_slv_arvalid ), // i
    .axi_slv_arready (axi_slv_arready ), // o
    .axi_slv_arid    (axi_slv_arid    ), // i
    .axi_slv_araddr  (axi_slv_araddr  ), // i
    .axi_slv_arlen   (axi_slv_arlen   ), // i
    .axi_slv_arsize  (axi_slv_arsize  ), // i
    .axi_slv_arburst (axi_slv_arburst ), // i
    .axi_slv_aruser  (axi_slv_aruser  ), // i

    .axi_slv_rvalid  (axi_slv_rvalid  ), // o
    .axi_slv_rready  (axi_slv_rready  ), // i
    .axi_slv_rid     (axi_slv_rid     ), // o
    .axi_slv_rdata   (axi_slv_rdata   ), // o
    .axi_slv_rresp   (axi_slv_rresp   ), // o
    .axi_slv_rlast   (axi_slv_rlast   ), // o
    .axi_slv_ruser   (axi_slv_ruser   )  // o
);
//--------------------------------------------------------------------------------
// Link Wire
//--------------------------------------------------------------------------------
assign axi_slv_arvalid = axi_mst_arvalid;
assign axi_mst_arready = axi_slv_arready;
assign axi_slv_arid    = axi_mst_arid ;
assign axi_slv_araddr  = axi_mst_araddr;
assign axi_slv_arlen   = axi_mst_arlen;
assign axi_slv_arsize  = axi_mst_arsize;
assign axi_slv_arburst = axi_mst_arburst;
assign axi_slv_aruser  = axi_mst_aruser;

assign axi_mst_rvalid = axi_slv_rvalid;
assign axi_slv_rready = axi_mst_rready;
assign axi_mst_rid    = axi_slv_rid;
assign axi_mst_rdata  = axi_slv_rdata;
assign axi_mst_rresp  = axi_slv_rresp;
assign axi_mst_rlast  = axi_slv_rlast;
assign axi_mst_ruser  = axi_slv_ruser;

endmodule
