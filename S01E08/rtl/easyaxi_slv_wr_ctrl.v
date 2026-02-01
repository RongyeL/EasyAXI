// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_slv_wr_ctrl.v
// Author        : Rongye
// Created On    : 2025-02-06 06:52
// Last Modified : 2026-02-01 06:36
// ---------------------------------------------------------------------------------
// Description   : This module implements an AXI slave write controller.
//   - Supports outstanding transactions (configurable depth via OST_DEPTH)
//   - Supports all AXI burst types: FIXED, INCR, WRAP
//   - Maximum burst length: 8 beats
//   - Write data reception and storage
//   - Response generation with simulated processing latency
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_SLV_WR_CTRL #(
    parameter OST_DEPTH = 16  // Outstanding depth, must be power of 2
)(
// Global
    input  wire                      clk,
    input  wire                      rst_n, 

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

localparam DLY              = 0.1;
localparam MAX_BURST_LEN    = 8;  // Maximum burst length support
localparam BURST_CNT_W      = $clog2(MAX_BURST_LEN);  // Maximum burst length cnt width
localparam REG_ADDR         = 16'h0000;  // Default register address
localparam OST_CNT_W        = OST_DEPTH == 1 ? 1 : $clog2(OST_DEPTH);      // Outstanding counter width
localparam MAX_GET_RESP_DLY = `AXI_RESP_GET_CNT_W'h1F;      // Outstanding counter width

//--------------------------------------------------------------------------------
// Inner Signal 
//--------------------------------------------------------------------------------
// Core control, state tracking, and data buffering signals for write transaction
wire                     wr_buff_set;         
wire                     wr_buff_clr;         
wire                     wr_buff_full;         

reg                      wr_valid_buff_r [OST_DEPTH-1:0];
reg                      wr_result_buff_r[OST_DEPTH-1:0];
reg                      wr_comp_buff_r  [OST_DEPTH-1:0];
reg                      wr_clear_buff_r [OST_DEPTH-1:0];

// Bit-vector representations for status flags
reg  [OST_DEPTH    -1:0] wr_valid_bits;
wire [OST_DEPTH    -1:0] wr_set_bits;
reg  [OST_DEPTH    -1:0] wr_result_bits;
reg  [OST_DEPTH    -1:0] wr_clear_bits;
wire [OST_DEPTH    -1:0] wr_order_bits;


// Outstanding buffer pointer management
wire [OST_CNT_W    -1:0] wr_set_ptr;
wire [OST_CNT_W    -1:0] wr_clr_ptr;
wire [OST_CNT_W    -1:0] wr_data_ptr;
wire [OST_CNT_W    -1:0] wr_result_ptr;

// AXI write transaction payload storage per outstanding entry
reg  [`AXI_LEN_W   -1:0] wr_curr_index_r [OST_DEPTH-1:0];
reg  [`AXI_ID_W    -1:0] wr_id_buff_r    [OST_DEPTH-1:0];
reg  [`AXI_ADDR_W  -1:0] wr_addr_buff_r  [OST_DEPTH-1:0];
reg  [`AXI_LEN_W   -1:0] wr_len_buff_r   [OST_DEPTH-1:0];
reg  [`AXI_SIZE_W  -1:0] wr_size_buff_r  [OST_DEPTH-1:0];
reg  [`AXI_BURST_W -1:0] wr_burst_buff_r [OST_DEPTH-1:0];
reg  [`AXI_USER_W  -1:0] wr_user_buff_r  [OST_DEPTH-1:0];

// Hold write data beats and track per-burst progress
reg  [MAX_BURST_LEN             -1:0] wr_data_vld_r  [OST_DEPTH-1:0];
reg  [MAX_BURST_LEN*`AXI_DATA_W -1:0] wr_data_buff_r [OST_DEPTH-1:0];
reg  [BURST_CNT_W               -1:0] wr_data_cnt_r  [OST_DEPTH-1:0]; // Counter for burst data
reg  [`AXI_RESP_W               -1:0] wr_resp_buff_r [OST_DEPTH-1:0];


// Burst address generation and wrap tracking
reg  [`AXI_ADDR_W  -1:0] wr_curr_addr_r  [OST_DEPTH-1:0];     // Current address
reg                      wr_wrap_en_r    [OST_DEPTH-1:0];     // Wrap happen Tag

// Write response and data flow control signals
wire                     wr_dec_miss;         
wire                     wr_result_en;        
wire [`AXI_ID_W    -1:0] wr_result_id;        

wire                     wr_data_en;        
wire                     wr_data_last;        

// Simulated response/data availability tracking
reg  [`AXI_DATA_GET_CNT_W-1:0] wr_resp_get_cnt   [OST_DEPTH-1:0];
wire                           wr_resp_get_cnt_en[OST_DEPTH-1:0];         
wire                           wr_resp_get       [OST_DEPTH-1:0];         
wire                           wr_resp_err       [OST_DEPTH-1:0];         

// Used to support FIXED / INCR / WRAP burst types
wire [`AXI_ADDR_W    -1:0] wr_start_addr    [OST_DEPTH-1:0]; 
wire [`AXI_LEN_W     -1:0] wr_burst_lenth   [OST_DEPTH-1:0]; 
wire [2**`AXI_SIZE_W -1:0] wr_number_bytes  [OST_DEPTH-1:0]; 
wire [`AXI_ADDR_W    -1:0] wr_wrap_boundary [OST_DEPTH-1:0]; 
wire [`AXI_ADDR_W    -1:0] wr_aligned_addr  [OST_DEPTH-1:0]; 
wire                       wr_wrap_en       [OST_DEPTH-1:0]; 

//--------------------------------------------------------------------------------
// Pointer Management
//--------------------------------------------------------------------------------
assign wr_set_bits  = ~wr_valid_bits;
EASYAXI_ARB #(
    .DEEP_NUM(OST_DEPTH)
) U_SLV_WR_ARB_SET (
    .clk      (clk           ),
    .rst_n    (rst_n         ),
    .queue_i  (wr_set_bits   ),
    .sche_en  (wr_buff_set   ),
    .pointer_o(wr_set_ptr    )
);

EASYAXI_ARB #(
    .DEEP_NUM(OST_DEPTH)
) U_SLV_WR_ARB_CLEAR (
    .clk      (clk           ),
    .rst_n    (rst_n         ),
    .queue_i  (wr_clear_bits ),
    .sche_en  (wr_buff_clr   ),
    .pointer_o(wr_clr_ptr    )
);

EASYAXI_ARB #(
    .DEEP_NUM(OST_DEPTH)
) U_SLV_WR_ARB_RESULT (
    .clk      (clk            ),
    .rst_n    (rst_n          ),
    .queue_i  (wr_result_bits & wr_order_bits),
    .sche_en  (wr_result_en   ),
    .pointer_o(wr_result_ptr  )
);

//--------------------------------------------------------------------------------
// Main Ctrl
//--------------------------------------------------------------------------------
assign wr_buff_set = axi_slv_awvalid & axi_slv_awready;
assign wr_buff_clr = wr_valid_buff_r[wr_clr_ptr] & ~wr_result_buff_r[wr_clr_ptr] & ~wr_comp_buff_r[wr_clr_ptr];

always @(*) begin : SLV_WR_VLD_VEC
    integer i;
    wr_valid_bits = {OST_DEPTH{1'b0}};
    for (i=0; i<OST_DEPTH; i=i+1) begin
        wr_valid_bits[i] = wr_valid_buff_r[i];
    end
end
assign wr_buff_full = &wr_valid_bits;

always @(*) begin : SLV_WR_RESULT_VEC
    integer i;
    wr_result_bits = {OST_DEPTH{1'b0}};
    for (i=0; i<OST_DEPTH; i=i+1) begin
        wr_result_bits[i] = wr_result_buff_r[i];
    end
end

always @(*) begin : SLV_WR_CLEAR_VEC
    integer i;
    wr_clear_bits = {OST_DEPTH{1'b0}};
    for (i=0; i<OST_DEPTH; i=i+1) begin
        wr_clear_bits[i] = wr_clear_buff_r[i];
    end
end

assign wr_dec_miss    = 1'b0/*  (axi_slv_awaddr != REG_ADDR) */;
assign wr_data_en     = axi_slv_wvalid & axi_slv_wready;
assign wr_data_last   = axi_slv_wlast;
assign wr_result_en   = axi_slv_bvalid & axi_slv_bready;
assign wr_result_id   = axi_slv_bid;

genvar i;
generate 
for (i=0; i<OST_DEPTH; i=i+1) begin: GEN_SLV_WR_CTRL
    // Valid buffer control
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_valid_buff_r[i] <= #DLY 1'b0;
        end
        else begin
            if (wr_buff_set && (wr_set_ptr == i)) begin
                wr_valid_buff_r[i] <= #DLY 1'b1;
            end
            if (wr_buff_clr && (wr_clr_ptr == i)) begin
                wr_valid_buff_r[i] <= #DLY 1'b0;
            end
        end
    end

    // Result sent flag buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_result_buff_r[i] <= #DLY 1'b0;
        end
        else begin
            if (wr_buff_set && (wr_set_ptr == i)) begin
                wr_result_buff_r[i] <= #DLY wr_dec_miss ? 1'b1 : 1'b0;
            end
            else if (wr_resp_get[i]) begin
                wr_result_buff_r[i] <= #DLY 1'b1;
            end
            else if (wr_result_en && (wr_result_ptr == i)) begin
                wr_result_buff_r[i] <= #DLY 1'b0;
            end
        end
    end

    // Completion flag buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_comp_buff_r[i] <= #DLY 1'b0;
        end
        else if (wr_buff_set && (wr_set_ptr == i)) begin
            wr_comp_buff_r[i] <= #DLY 1'b1;
        end
        else if (wr_result_en  && (wr_result_ptr == i)) begin
            wr_comp_buff_r[i] <= #DLY 1'b0;
        end
    end

    // Clear flag buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_clear_buff_r[i] <= #DLY 1'b0;
        end
        else begin
            wr_clear_buff_r[i] <= #DLY wr_valid_buff_r[i] & ~wr_result_buff_r[i] & ~wr_comp_buff_r[i];
        end
    end

//--------------------------------------------------------------------------------
// AXI aw Payload Buffer
//--------------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_id_buff_r[i]    <= #DLY {`AXI_ID_W{1'b0}};
            wr_addr_buff_r[i]  <= #DLY {`AXI_ADDR_W{1'b0}};
            wr_len_buff_r[i]   <= #DLY {`AXI_LEN_W{1'b0}};
            wr_size_buff_r[i]  <= #DLY {`AXI_SIZE_W{1'b0}};
            wr_burst_buff_r[i] <= #DLY {`AXI_BURST_W{1'b0}};
            wr_user_buff_r[i]  <= #DLY {`AXI_USER_W{1'b0}};
        end
        else if (wr_buff_set && (wr_set_ptr == i)) begin
            wr_id_buff_r[i]    <= #DLY axi_slv_awid;
            wr_addr_buff_r[i]  <= #DLY axi_slv_awaddr;
            wr_len_buff_r[i]   <= #DLY axi_slv_awlen;
            wr_size_buff_r[i]  <= #DLY axi_slv_awsize;
            wr_burst_buff_r[i] <= #DLY axi_slv_awburst;
            wr_user_buff_r[i]  <= #DLY axi_slv_awuser;
        end
    end

//--------------------------------------------------------------------------------
// Burst Address Control
//--------------------------------------------------------------------------------
    assign wr_start_addr   [i] = (wr_buff_set && (wr_set_ptr == i)) ? axi_slv_awaddr : wr_addr_buff_r[i];
    assign wr_number_bytes [i] = (wr_buff_set && (wr_set_ptr == i)) ? 1 << axi_slv_awsize : 1 << wr_size_buff_r[i];
    assign wr_burst_lenth  [i] = (wr_buff_set && (wr_set_ptr == i)) ? axi_slv_awlen + 1 : wr_len_buff_r[i] + 1;
    assign wr_aligned_addr [i] = wr_start_addr[i] / wr_number_bytes[i] * wr_number_bytes[i];
    assign wr_wrap_boundary[i] = wr_start_addr[i] / (wr_burst_lenth[i] * wr_number_bytes[i]) * (wr_burst_lenth[i] * wr_number_bytes[i]);
    assign wr_wrap_en      [i] = (wr_curr_addr_r[i] + wr_number_bytes[i]) == (wr_wrap_boundary[i] + (wr_burst_lenth[i] * wr_number_bytes[i]));

    // Read index control
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_curr_index_r[i] <= #DLY {`AXI_LEN_W{1'b0}};
        end
        else if (wr_buff_set && (wr_set_ptr == i)) begin
            wr_curr_index_r[i] <= #DLY wr_dec_miss ? wr_burst_lenth[i] : `AXI_LEN_W'h1;
        end
        else if (wr_result_en  && (wr_result_ptr == i)) begin
            wr_curr_index_r[i] <= #DLY {`AXI_LEN_W{1'b0}};
        end
        else if (wr_data_en) begin
            wr_curr_index_r[i] <= #DLY wr_curr_index_r[i] + `AXI_LEN_W'h1;
        end
    end

    // Read wrap control
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_wrap_en_r[i] <= #DLY 1'b0;
        end
        else if (wr_buff_set && (wr_set_ptr == i)) begin
            wr_wrap_en_r[i] <= #DLY 1'b0;
        end
        else if (wr_data_en) begin
            wr_wrap_en_r[i] <= #DLY wr_wrap_en_r[i] | wr_wrap_en[i];
        end
    end

    // Current address control
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_curr_addr_r[i] <= #DLY {`AXI_ADDR_W{1'b0}};
        end
        else if (wr_buff_set && (wr_set_ptr == i)) begin
            wr_curr_addr_r[i] <= #DLY wr_start_addr[i];
        end
        else if (wr_data_en) begin
            case (wr_burst_buff_r[i])
                `AXI_BURST_FIXED: wr_curr_addr_r[i] <= #DLY wr_start_addr[i];
                `AXI_BURST_INCR : wr_curr_addr_r[i] <= #DLY wr_aligned_addr[i] + (wr_curr_index_r[i] * wr_number_bytes[i]);
                `AXI_BURST_WRAP : begin
                    if (wr_wrap_en[i])
                        wr_curr_addr_r[i] <= #DLY wr_wrap_boundary[i];
                    else if (wr_wrap_en_r[i])
                        wr_curr_addr_r[i] <= #DLY wr_start_addr[i] + (wr_curr_index_r[i] * wr_number_bytes[i]) - (wr_number_bytes[i] * wr_burst_lenth[i]);
                    else
                        wr_curr_addr_r[i] <= #DLY wr_aligned_addr[i] + (wr_curr_index_r[i] * wr_number_bytes[i]);
                end
                default: wr_curr_addr_r[i] <= #DLY {`AXI_ADDR_W{1'b0}};
            endcase
        end
    end

//--------------------------------------------------------------------------------
// AXI W Payload Buffer
//--------------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_resp_buff_r[i] <= #DLY {`AXI_RESP_W{1'b0}};
        end
        else if (wr_buff_set && (wr_set_ptr == i)) begin
            wr_resp_buff_r[i] <= #DLY wr_dec_miss ? `AXI_RESP_DECERR : `AXI_RESP_OKAY;
        end
        else if (wr_resp_get[i]) begin
            wr_resp_buff_r[i] <= #DLY wr_resp_err[i] ? `AXI_RESP_SLVERR : `AXI_RESP_OKAY;
        end
    end
    // Burst data beat counter
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_data_cnt_r[i]  <= #DLY {BURST_CNT_W{1'b0}};
        end

        else if (wr_buff_set && (wr_set_ptr == i)) begin
            wr_data_cnt_r[i]  <= #DLY {BURST_CNT_W{1'b0}};
        end
        else if (wr_data_en && (wr_data_ptr == i)) begin
            wr_data_cnt_r[i] <= #DLY wr_data_cnt_r[i] + 1;
        end
    end
    // Burst data buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_data_vld_r [i] <= #DLY {MAX_BURST_LEN{1'b0}};
            wr_data_buff_r[i] <= #DLY {(`AXI_DATA_W*MAX_BURST_LEN){1'b0}};
        end
        else if (wr_buff_set && (wr_set_ptr == i)) begin
            wr_data_vld_r [i] <= #DLY {MAX_BURST_LEN{1'b0}};
            wr_data_buff_r[i] <= #DLY {(`AXI_DATA_W*MAX_BURST_LEN){1'b0}};
        end
        else if (wr_data_en) begin
            wr_data_vld_r [i][(wr_curr_index_r[i]-1)] <= #DLY 1'b1;
            wr_data_buff_r[i][((wr_curr_index_r[i]-1)*`AXI_DATA_W) +: `AXI_DATA_W] <= #DLY {{`AXI_DATA_W-`AXI_ID_W-`AXI_ADDR_W{1'b0}},wr_id_buff_r[i],wr_curr_addr_r[i]};
        end
    end
//--------------------------------------------------------------------------------
// Simulate the resp wait process
//--------------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_resp_get_cnt[i] <= #DLY `AXI_RESP_GET_CNT_W'h0;
        end
        else if (wr_data_en & wr_data_last & (wr_data_ptr == i)) begin
            wr_resp_get_cnt[i] <= #DLY `AXI_RESP_GET_CNT_W'h1;
        end
        else if (wr_resp_get_cnt[i]== MAX_GET_RESP_DLY) begin // wr resp delay is 18 cycle
            wr_resp_get_cnt[i] <= #DLY `AXI_RESP_GET_CNT_W'h0;
        end
        else if (wr_resp_get_cnt[i]>`AXI_RESP_GET_CNT_W'h0) begin
            wr_resp_get_cnt[i] <= #DLY wr_resp_get_cnt[i] + `AXI_RESP_GET_CNT_W'h1;
        end
    end
    assign wr_resp_get   [i] = wr_valid_buff_r[i] & (wr_resp_get_cnt[i]==(MAX_GET_RESP_DLY - 2*wr_id_buff_r[i]));
    assign wr_resp_err   [i] = (wr_id_buff_r[i] == `AXI_ID_W'hF);
end
endgenerate
//--------------------------------------------------------------------------------
// W DATA ORDER CTRL
//--------------------------------------------------------------------------------
EASYAXI_ORDER #(
    .OST_DEPTH  (OST_DEPTH  ),
    .ID_WIDTH   (`AXI_ID_W  )
) U_SLV_WR_ORDER_DATA (
    .clk        (clk                               ),
    .rst_n      (rst_n                             ),

    .push       (axi_slv_awvalid & axi_slv_awready ),
    .push_id    ({`AXI_ID_W{1'b0}}                 ),
    .push_ptr   (wr_set_ptr                        ),

    .pop        (axi_slv_wvalid & axi_slv_wready   ),
    .pop_id     ({`AXI_ID_W{1'b0}}                 ),
    .pop_last   (axi_slv_wlast                     ),

    .order_ptr  (wr_data_ptr                       ),
    .order_bits (                                  )
);

//--------------------------------------------------------------------------------
// RESP ID ORDER CTRL
//--------------------------------------------------------------------------------
EASYAXI_ORDER #(
    .OST_DEPTH  (OST_DEPTH  ),
    .ID_WIDTH   (`AXI_ID_W  )
) U_SLV_WR_ORDER_RESP (
    .clk        (clk                               ),
    .rst_n      (rst_n                             ),

    .push       (axi_slv_awvalid & axi_slv_awready ),
    .push_id    (axi_slv_awid                      ),
    .push_ptr   (wr_set_ptr                        ),

    .pop        (axi_slv_bvalid & axi_slv_bready   ),
    .pop_id     (axi_slv_bid                       ),
    .pop_last   (1'b1                              ), // bresp only once

    .order_ptr  (                                  ),
    .order_bits (wr_order_bits                     )
);
//--------------------------------------------------------------------------------
// Output Signal
//--------------------------------------------------------------------------------
assign axi_slv_awready = ~wr_buff_full;
assign axi_slv_wready  = 1'b1;
assign axi_slv_bvalid  = wr_valid_buff_r [wr_result_ptr] & wr_result_buff_r[wr_result_ptr];
assign axi_slv_bid     = wr_id_buff_r    [wr_result_ptr];
assign axi_slv_bdata   = wr_data_buff_r  [wr_result_ptr][((wr_data_cnt_r[wr_result_ptr])*`AXI_DATA_W) +: `AXI_DATA_W];
assign axi_slv_bresp   = wr_resp_buff_r  [wr_result_ptr];
assign axi_slv_blast   = axi_slv_bvalid & (wr_data_cnt_r[wr_result_ptr] == wr_len_buff_r[wr_result_ptr]);
assign axi_slv_buser   = wr_user_buff_r  [wr_result_ptr];

endmodule
