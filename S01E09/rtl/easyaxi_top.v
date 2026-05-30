// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi.v
// Author        : Rongye
// Created On    : 2025-02-05 05:04
// Last Modified : 2026-05-05
// ---------------------------------------------------------------------------------
// Description   : This is the top-level module of the EasyAXI verification platform
//  Designed for educational purposes to AXI (Advanced eXtensible Interface) 
//  protocol implementation and verification.
//   - OST_DEPTH parameter controls the maximum outstanding transactions
//   - AXI bus widths should be configured via `define statements
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_TOP #(
    parameter PROD_NUM_BLOCKS = 15,  // Number of blocks for producer
    parameter CONS_NUM_BLOCKS = 15   // Number of blocks for consumer
)(
    input  wire clk,
    input  wire rst_n,
    
    // Producer control
    input  wire                        prod_start,
    output wire                        prod_done,
    
    // Consumer control
    input  wire                        cons_start,
    output wire                        cons_done,
    output wire                        cons_error
);
//--------------------------------------------------------------------------------
// Inst Producer (Write Controller)
//--------------------------------------------------------------------------------

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

EASYAXI_PROD #(
    .OST_DEPTH    (16            ),
    .NUM_BLOCKS   (PROD_NUM_BLOCKS)
) U_EASYAXI_PROD(
    .clk                (clk                ),
    .rst_n              (rst_n              ),

    .start              (prod_start         ),
    .done               (prod_done          ),

    .axi_mst_awvalid    (axi_mst_awvalid    ),
    .axi_mst_awready    (axi_mst_awready    ),
    .axi_mst_awid       (axi_mst_awid       ),
    .axi_mst_awaddr     (axi_mst_awaddr     ),
    .axi_mst_awlen      (axi_mst_awlen      ),
    .axi_mst_awsize     (axi_mst_awsize     ),
    .axi_mst_awburst    (axi_mst_awburst    ),
    .axi_mst_awuser     (axi_mst_awuser     ),

    .axi_mst_wvalid     (axi_mst_wvalid     ),
    .axi_mst_wready     (axi_mst_wready     ),
    .axi_mst_wdata      (axi_mst_wdata      ),
    .axi_mst_wstrb      (axi_mst_wstrb      ),
    .axi_mst_wlast      (axi_mst_wlast      ),
    .axi_mst_wuser      (axi_mst_wuser      ),

    .axi_mst_bvalid     (axi_mst_bvalid     ),
    .axi_mst_bready     (axi_mst_bready     ),
    .axi_mst_bid        (axi_mst_bid        ),
    .axi_mst_bresp      (axi_mst_bresp      ),
    .axi_mst_buser      (axi_mst_buser      )
);

//--------------------------------------------------------------------------------
// Inst Consumer (Read Controller)
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

EASYAXI_CONS #(
    .OST_DEPTH    (16            ),
    .NUM_BLOCKS   (CONS_NUM_BLOCKS)
) U_EASYAXI_CONS(
    .clk                (clk                ),
    .rst_n              (rst_n              ),

    .start              (cons_start         ),
    .done               (cons_done          ),
    .error              (cons_error         ),

    .axi_mst_arvalid    (axi_mst_arvalid    ),
    .axi_mst_arready    (axi_mst_arready    ),
    .axi_mst_arid       (axi_mst_arid       ),
    .axi_mst_araddr     (axi_mst_araddr     ),
    .axi_mst_arlen      (axi_mst_arlen      ),
    .axi_mst_arsize     (axi_mst_arsize     ),
    .axi_mst_arburst    (axi_mst_arburst    ),
    .axi_mst_aruser     (axi_mst_aruser     ),

    .axi_mst_rvalid     (axi_mst_rvalid     ),
    .axi_mst_rready     (axi_mst_rready     ),
    .axi_mst_rid        (axi_mst_rid        ),
    .axi_mst_rdata      (axi_mst_rdata      ),
    .axi_mst_rresp      (axi_mst_rresp      ),
    .axi_mst_rlast      (axi_mst_rlast      ),
    .axi_mst_ruser      (axi_mst_ruser      )
);

//--------------------------------------------------------------------------------
// Inst SRAM (Slave + Memory)
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

wire                        axi_slv_awvalid;
wire                        axi_slv_awready;
wire  [`AXI_ID_W    -1:0]   axi_slv_awid;
wire  [`AXI_ADDR_W  -1:0]   axi_slv_awaddr;
wire  [`AXI_LEN_W   -1:0]   axi_slv_awlen;
wire  [`AXI_SIZE_W  -1:0]   axi_slv_awsize;
wire  [`AXI_BURST_W -1:0]   axi_slv_awburst;
wire  [`AXI_USER_W  -1:0]   axi_slv_awuser;

wire                        axi_slv_wvalid;
wire                        axi_slv_wready;
wire  [`AXI_DATA_W  -1:0]   axi_slv_wdata;
wire  [`AXI_DATA_W/8-1:0]   axi_slv_wstrb;
wire                        axi_slv_wlast;
wire  [`AXI_USER_W  -1:0]   axi_slv_wuser;

wire                        axi_slv_bvalid;
wire                        axi_slv_bready;
wire  [`AXI_ID_W    -1:0]   axi_slv_bid;
wire  [`AXI_RESP_W  -1:0]   axi_slv_bresp;
wire  [`AXI_USER_W  -1:0]   axi_slv_buser;

EASYAXI_SRAM #(
    .OST_DEPTH  (16      )
) U_EASYAXI_SRAM(
    .clk                (clk                ), // i
    .rst_n              (rst_n              ), // i

    .axi_slv_arvalid    (axi_slv_arvalid    ), // i
    .axi_slv_arready    (axi_slv_arready    ), // o
    .axi_slv_arid       (axi_slv_arid       ), // i
    .axi_slv_araddr     (axi_slv_araddr     ), // i
    .axi_slv_arlen      (axi_slv_arlen      ), // i
    .axi_slv_arsize     (axi_slv_arsize     ), // i
    .axi_slv_arburst    (axi_slv_arburst    ), // i
    .axi_slv_aruser     (axi_slv_aruser     ), // i

    .axi_slv_rvalid     (axi_slv_rvalid     ), // o
    .axi_slv_rready     (axi_slv_rready     ), // i
    .axi_slv_rid        (axi_slv_rid        ), // o
    .axi_slv_rdata      (axi_slv_rdata      ), // o
    .axi_slv_rresp      (axi_slv_rresp      ), // o
    .axi_slv_rlast      (axi_slv_rlast      ), // o
    .axi_slv_ruser      (axi_slv_ruser      ), // o

    .axi_slv_awvalid    (axi_slv_awvalid    ), // i
    .axi_slv_awready    (axi_slv_awready    ), // o
    .axi_slv_awid       (axi_slv_awid       ), // i
    .axi_slv_awaddr     (axi_slv_awaddr     ), // i
    .axi_slv_awlen      (axi_slv_awlen      ), // i
    .axi_slv_awsize     (axi_slv_awsize     ), // i
    .axi_slv_awburst    (axi_slv_awburst    ), // i
    .axi_slv_awuser     (axi_slv_awuser     ), // i

    .axi_slv_wvalid     (axi_slv_wvalid     ), // i
    .axi_slv_wready     (axi_slv_wready     ), // o
    .axi_slv_wdata      (axi_slv_wdata      ), // i
    .axi_slv_wstrb      (axi_slv_wstrb      ), // i
    .axi_slv_wlast      (axi_slv_wlast      ), // i
    .axi_slv_wuser      (axi_slv_wuser      ), // i

    .axi_slv_bvalid     (axi_slv_bvalid     ), // o
    .axi_slv_bready     (axi_slv_bready     ), // i
    .axi_slv_bid        (axi_slv_bid        ), // o
    .axi_slv_bresp      (axi_slv_bresp      ), // o
    .axi_slv_buser      (axi_slv_buser      )  // o
);

//--------------------------------------------------------------------------------
// Link 
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

assign axi_slv_awvalid = axi_mst_awvalid;
assign axi_mst_awready = axi_slv_awready;
assign axi_slv_awid    = axi_mst_awid ;
assign axi_slv_awaddr  = axi_mst_awaddr;
assign axi_slv_awlen   = axi_mst_awlen;
assign axi_slv_awsize  = axi_mst_awsize;
assign axi_slv_awburst = axi_mst_awburst;
assign axi_slv_awuser  = axi_mst_awuser;

assign axi_slv_wvalid = axi_mst_wvalid;
assign axi_mst_wready = axi_slv_wready;
assign axi_slv_wdata  = axi_mst_wdata;
assign axi_slv_wstrb  = axi_mst_wstrb;
assign axi_slv_wlast  = axi_mst_wlast;
assign axi_slv_wuser  = axi_mst_wuser;

assign axi_mst_bvalid = axi_slv_bvalid;
assign axi_slv_bready = axi_mst_bready;
assign axi_mst_bid    = axi_slv_bid;
assign axi_mst_bresp  = axi_slv_bresp;
assign axi_mst_buser  = axi_slv_buser;

endmodule
