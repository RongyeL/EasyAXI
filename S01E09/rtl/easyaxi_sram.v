// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_sram.v
// Author        : Rongye
// Created On    : 2026-05-05
// Last Modified : 2026-05-05
// ---------------------------------------------------------------------------------
// Description   : Wrapper module that combines slave read/write controllers
//   and memory into a single EASYAXI_SRAM module.
//   - AXI slave interface (AR, R, AW, W, B channels)
//   - Internal memory for data storage
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_SRAM #(
    parameter OST_DEPTH = 16  // Outstanding depth, must be power of 2
)(
    input  wire                      clk,
    input  wire                      rst_n,

// AXI AR Channel
    input  wire                      axi_slv_arvalid,
    output wire                      axi_slv_arready,
    input  wire  [`AXI_ID_W    -1:0] axi_slv_arid,
    input  wire  [`AXI_ADDR_W  -1:0] axi_slv_araddr,
    input  wire  [`AXI_LEN_W   -1:0] axi_slv_arlen,
    input  wire  [`AXI_SIZE_W  -1:0] axi_slv_arsize,
    input  wire  [`AXI_BURST_W -1:0] axi_slv_arburst,
    input  wire  [`AXI_USER_W  -1:0] axi_slv_aruser,

// AXI R Channel
    output wire                      axi_slv_rvalid,
    input  wire                      axi_slv_rready,
    output wire  [`AXI_ID_W    -1:0] axi_slv_rid,
    output wire  [`AXI_DATA_W  -1:0] axi_slv_rdata,
    output wire  [`AXI_RESP_W  -1:0] axi_slv_rresp,
    output wire                      axi_slv_rlast,
    output wire  [`AXI_USER_W  -1:0] axi_slv_ruser,

// AXI AW Channel
    input  wire                      axi_slv_awvalid,
    output wire                      axi_slv_awready,
    input  wire  [`AXI_ID_W    -1:0] axi_slv_awid,
    input  wire  [`AXI_ADDR_W  -1:0] axi_slv_awaddr,
    input  wire  [`AXI_LEN_W   -1:0] axi_slv_awlen,
    input  wire  [`AXI_SIZE_W  -1:0] axi_slv_awsize,
    input  wire  [`AXI_BURST_W -1:0] axi_slv_awburst,
    input  wire  [`AXI_USER_W  -1:0] axi_slv_awuser,

// AXI W Channel
    input  wire                      axi_slv_wvalid,
    output wire                      axi_slv_wready,
    input  wire  [`AXI_DATA_W  -1:0] axi_slv_wdata,
    input  wire  [`AXI_DATA_W/8-1:0] axi_slv_wstrb,
    input  wire                      axi_slv_wlast,
    input  wire  [`AXI_USER_W  -1:0] axi_slv_wuser,

// AXI B Channel
    output wire                      axi_slv_bvalid,
    input  wire                      axi_slv_bready,
    output wire  [`AXI_ID_W    -1:0] axi_slv_bid,
    output wire  [`AXI_RESP_W  -1:0] axi_slv_bresp,
    output wire  [`AXI_USER_W  -1:0] axi_slv_buser
);

//--------------------------------------------------------------------------------
// Internal Interface Signals
//--------------------------------------------------------------------------------
// Slave Read Internal Interface
wire                        s_rd_req_en;
wire  [`AXI_ID_W    -1:0]   s_rd_req_id;
wire  [`AXI_ADDR_W  -1:0]   s_rd_req_addr;
wire                        s_rd_resp_en;
wire  [`AXI_ID_W    -1:0]   s_rd_resp_id;
wire  [127:0]               s_rd_resp_data;   // 128-bit data for internal interface
wire  [`AXI_RESP_W  -1:0]   s_rd_resp_resp;

// Slave Write Internal Interface
wire                        s_wr_data_en;
wire  [`AXI_ID_W    -1:0]   s_wr_data_id;
wire  [`AXI_ADDR_W  -1:0]   s_wr_data_addr;
wire  [127:0]               s_wr_data_data;   // 128-bit data for internal interface
wire  [15:0]                s_wr_data_strb;   // 16-byte strobe

//--------------------------------------------------------------------------------
// Slave Read Controller
//--------------------------------------------------------------------------------
EASYAXI_SLV_RD_CTRL #(
    .OST_DEPTH  (OST_DEPTH  )
) U_EASYAXI_SLV_RD_CTRL(
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
    .axi_slv_ruser   (axi_slv_ruser   ), // o

    // Slave Read Internal Interface
    .s_rd_req_en     (s_rd_req_en     ), // o
    .s_rd_req_id     (s_rd_req_id     ), // o
    .s_rd_req_addr   (s_rd_req_addr   ), // o
    .s_rd_resp_en    (s_rd_resp_en    ), // i
    .s_rd_resp_id    (s_rd_resp_id    ), // i
    .s_rd_resp_data  (s_rd_resp_data  ), // i
    .s_rd_resp_resp  (s_rd_resp_resp  )  // i
);

//--------------------------------------------------------------------------------
// Slave Write Controller
//--------------------------------------------------------------------------------
EASYAXI_SLV_WR_CTRL #(
    .OST_DEPTH  (OST_DEPTH  )
) U_EASYAXI_SLV_WR_CTRL(
    .clk             (clk             ), // i
    .rst_n           (rst_n           ), // i

    .axi_slv_awvalid (axi_slv_awvalid ), // i
    .axi_slv_awready (axi_slv_awready ), // o
    .axi_slv_awid    (axi_slv_awid    ), // i
    .axi_slv_awaddr  (axi_slv_awaddr  ), // i
    .axi_slv_awlen   (axi_slv_awlen   ), // i
    .axi_slv_awsize  (axi_slv_awsize  ), // i
    .axi_slv_awburst (axi_slv_awburst ), // i
    .axi_slv_awuser  (axi_slv_awuser  ), // i

    .axi_slv_wvalid  (axi_slv_wvalid  ), // i
    .axi_slv_wready  (axi_slv_wready  ), // o
    .axi_slv_wdata   (axi_slv_wdata   ), // i
    .axi_slv_wstrb   (axi_slv_wstrb   ), // i
    .axi_slv_wlast   (axi_slv_wlast   ), // i
    .axi_slv_wuser   (axi_slv_wuser   ), // i

    .axi_slv_bvalid  (axi_slv_bvalid  ), // o
    .axi_slv_bready  (axi_slv_bready  ), // i
    .axi_slv_bid     (axi_slv_bid     ), // o
    .axi_slv_bresp   (axi_slv_bresp   ), // o
    .axi_slv_buser   (axi_slv_buser   ), // i

    // Slave Write Internal Interface
    .s_wr_data_en    (s_wr_data_en    ), // o
    .s_wr_data_id    (s_wr_data_id    ), // o
    .s_wr_data_addr  (s_wr_data_addr  ), // o
    .s_wr_data_data  (s_wr_data_data  ), // o
    .s_wr_data_strb  (s_wr_data_strb  )  // o
);

//--------------------------------------------------------------------------------
// Memory Module
//--------------------------------------------------------------------------------
EASYAXI_MEM U_EASYAXI_MEM (
    .clk             (clk             ), // i
    .rst_n           (rst_n           ), // i
    
    // Write interface from slave write controller
    .s_wr_data_en    (s_wr_data_en    ), // i
    .s_wr_data_id    (s_wr_data_id    ), // i
    .s_wr_data_addr  (s_wr_data_addr  ), // i
    .s_wr_data_data  (s_wr_data_data  ), // i
    .s_wr_data_strb  (s_wr_data_strb  ), // i
    
    // Read interface to slave read controller
    .s_rd_req_en     (s_rd_req_en     ), // i
    .s_rd_req_id     (s_rd_req_id     ), // i
    .s_rd_req_addr   (s_rd_req_addr   ), // i
    
    .s_rd_resp_en    (s_rd_resp_en    ), // o
    .s_rd_resp_id    (s_rd_resp_id    ), // o
    .s_rd_resp_data  (s_rd_resp_data  ), // o
    .s_rd_resp_resp  (s_rd_resp_resp  )  // o
);

endmodule
