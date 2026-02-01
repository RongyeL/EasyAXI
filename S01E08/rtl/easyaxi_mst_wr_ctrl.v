// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_mst_wr_ctrl.v
// Author        : Rongye
// Created On    : 2025-02-06 06:45
// Last Modified : 2026-02-01 06:33
// ---------------------------------------------------------------------------------
// Description   : This module implements an AXI master write controller.
//   - Supports outstanding transactions (configurable depth via OST_DEPTH)
//   - Supports all AXI burst types: FIXED, INCR, WRAP
//   - Maximum burst length: 8 beats
//   - Burst address generation and calculation
//   - Write data ordering
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_MST_WR_CTRL #(
    parameter OST_DEPTH = 16  // Outstanding transaction depth (power of 2)
)(
// Global
    input  wire                      clk,
    input  wire                      rst_n, 
    input  wire                      wr_en,
    output wire                      wr_done,

// AXI AW Channel
    output wire                      axi_mst_awvalid,
    input  wire                      axi_mst_awready,
    output wire  [`AXI_ID_W    -1:0] axi_mst_awid,
    output wire  [`AXI_ADDR_W  -1:0] axi_mst_awaddr,
    output wire  [`AXI_LEN_W   -1:0] axi_mst_awlen,
    output wire  [`AXI_SIZE_W  -1:0] axi_mst_awsize,
    output wire  [`AXI_BURST_W -1:0] axi_mst_awburst,
    output wire  [`AXI_USER_W  -1:0] axi_mst_awuser,

// AXI W Channel
    output wire                      axi_mst_wvalid,
    input  wire                      axi_mst_wready,
    output wire  [`AXI_DATA_W  -1:0] axi_mst_wdata,
    output wire  [`AXI_DATA_W/8-1:0] axi_mst_wstrb,
    output wire                      axi_mst_wlast,
    output wire  [`AXI_USER_W  -1:0] axi_mst_wuser,

// AXI B Channel
    input  wire                      axi_mst_bvalid,
    output wire                      axi_mst_bready,
    input  wire  [`AXI_ID_W    -1:0] axi_mst_bid,
    input  wire  [`AXI_RESP_W  -1:0] axi_mst_bresp,
    input  wire  [`AXI_USER_W  -1:0] axi_mst_buser
);

localparam DLY = 0.1;

// Burst configuration
localparam MAX_BURST_LEN    = 8;       // Maximum supported burst length
localparam BURST_CNT_W      = $clog2(MAX_BURST_LEN);
localparam OST_CNT_W        = OST_DEPTH == 1 ? 1 : $clog2(OST_DEPTH);
localparam MAX_REQ_NUM      = 16;       // Maximum number of requests
localparam REQ_CNT_W        = $clog2(MAX_REQ_NUM);
localparam MAX_GET_DATA_DLY = `AXI_DATA_GET_CNT_W'h1C;      // Outstanding counter width

//--------------------------------------------------------------------------------
// Inner Signal 
//--------------------------------------------------------------------------------
// Core control and state signals for outstanding write transaction management
wire                     wr_buff_set;
wire                     wr_buff_clr;
wire                     wr_buff_full;
reg                      wr_buff_set_r;

reg                      wr_valid_buff_r[OST_DEPTH-1:0];
reg                      wr_req_buff_r  [OST_DEPTH-1:0];
reg                      wr_dat_buff_r  [OST_DEPTH-1:0];
reg                      wr_comp_buff_r [OST_DEPTH-1:0];
reg                      wr_clear_buff_r[OST_DEPTH-1:0];

// Bit-vector views of outstanding buffer states
reg  [OST_DEPTH    -1:0] wr_valid_bits;
wire [OST_DEPTH    -1:0] wr_set_bits;
reg  [OST_DEPTH    -1:0] wr_req_bits;
reg  [OST_DEPTH    -1:0] wr_clear_bits;

// Outstanding buffer pointer management
wire [OST_CNT_W    -1:0] wr_set_ptr;
wire [OST_CNT_W    -1:0] wr_clr_ptr;
wire [OST_CNT_W    -1:0] wr_req_ptr;
wire [OST_CNT_W    -1:0] wr_data_ptr;
wire [OST_CNT_W    -1:0] wr_result_ptr;
reg  [OST_CNT_W    -1:0] wr_set_ptr_r;

// AXI write transaction payload storage per outstanding entry
reg  [`AXI_LEN_W   -1:0] wr_curr_index_r[OST_DEPTH-1:0];
reg  [`AXI_ID_W    -1:0] wr_id_buff_r   [OST_DEPTH-1:0];
reg  [`AXI_ADDR_W  -1:0] wr_addr_buff_r [OST_DEPTH-1:0];
reg  [`AXI_LEN_W   -1:0] wr_len_buff_r  [OST_DEPTH-1:0];
reg  [`AXI_SIZE_W  -1:0] wr_size_buff_r [OST_DEPTH-1:0];
reg  [`AXI_BURST_W -1:0] wr_burst_buff_r[OST_DEPTH-1:0];
reg  [`AXI_USER_W  -1:0] wr_user_buff_r [OST_DEPTH-1:0];

reg  [`AXI_ADDR_W  -1:0] wr_addr_buff   [OST_DEPTH-1:0];
reg  [`AXI_LEN_W   -1:0] wr_len_buff    [OST_DEPTH-1:0];
reg  [`AXI_SIZE_W  -1:0] wr_size_buff   [OST_DEPTH-1:0];
reg  [`AXI_BURST_W -1:0] wr_burst_buff  [OST_DEPTH-1:0];

// Write data buffering for burst transmission
reg  [MAX_BURST_LEN                 -1:0] wr_data_vld_r  [OST_DEPTH-1:0];
reg  [`AXI_DATA_W*MAX_BURST_LEN     -1:0] wr_data_buff_r [OST_DEPTH-1:0];
reg  [(`AXI_DATA_W/8)*MAX_BURST_LEN -1:0] wr_strb_buff_r [OST_DEPTH-1:0];
reg  [BURST_CNT_W                   -1:0] wr_data_cnt_r  [OST_DEPTH-1:0];
reg  [`AXI_RESP_W                   -1:0] wr_resp_buff_r [OST_DEPTH-1:0];
wire [OST_DEPTH                     -1:0] wr_resp_err;   
// AXI channel handshake
wire                   wr_req_en;
wire                   wr_data_en;
wire                   wr_data_last;

reg  [REQ_CNT_W  -1:0] wr_req_cnt_r;

// Burst address generation and wrap tracking
reg  [`AXI_ADDR_W-1:0] wr_curr_addr_r  [OST_DEPTH-1:0];
reg                    wr_wrap_en_r    [OST_DEPTH-1:0];

// Write response handling and completion detection
wire                   wr_result_en;
wire [`AXI_ID_W  -1:0] wr_result_id;
wire                   wr_result_last;

// Simulated write data availability and error detection
reg  [`AXI_DATA_GET_CNT_W-1:0] wr_data_get_cnt   [OST_DEPTH-1:0];
wire                           wr_data_get_cnt_en[OST_DEPTH-1:0];
wire                           wr_data_get       [OST_DEPTH-1:0];
wire                           wr_data_err       [OST_DEPTH-1:0];

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
assign wr_set_bits = ~wr_valid_bits;
EASYAXI_ARB #(
    .DEEP_NUM(OST_DEPTH)
) U_MST_WR_ARB_SET (
    .clk      (clk           ),
    .rst_n    (rst_n         ),
    .queue_i  (wr_set_bits   ),
    .sche_en  (wr_buff_set   ),
    .pointer_o(wr_set_ptr  )
);
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        wr_buff_set_r <= #DLY 1'b0;
        wr_set_ptr_r  <= #DLY {OST_DEPTH{1'b0}};
    end
    else begin
        wr_buff_set_r <= #DLY wr_buff_set;
        wr_set_ptr_r  <= #DLY wr_set_ptr;
    end
end
EASYAXI_ARB #(
    .DEEP_NUM(OST_DEPTH)
) U_MST_WR_ARB_CLEAR (
    .clk      (clk           ),
    .rst_n    (rst_n         ),
    .queue_i  (wr_clear_bits ),
    .sche_en  (wr_buff_clr   ),
    .pointer_o(wr_clr_ptr  )
);

EASYAXI_ARB #(
    .DEEP_NUM(OST_DEPTH)
) U_MST_WR_ARB_REQ (
    .clk      (clk         ),
    .rst_n    (rst_n       ),
    .queue_i  (wr_req_bits ),
    .sche_en  (wr_req_en   ),
    .pointer_o(wr_req_ptr  )
);

//--------------------------------------------------------------------------------
// Main Ctrl
//--------------------------------------------------------------------------------
assign wr_buff_set = ~wr_buff_full & wr_en;
assign wr_buff_clr = wr_valid_buff_r[wr_clr_ptr] & ~wr_req_buff_r[wr_clr_ptr] & ~wr_comp_buff_r[wr_clr_ptr];

always @(*) begin : MST_WR_VLD_VEC
    integer i;
    wr_valid_bits = {OST_DEPTH{1'b0}};
    for (i=0; i<OST_DEPTH; i=i+1) begin
        wr_valid_bits[i] = wr_valid_buff_r[i];
    end
end
assign wr_buff_full = &wr_valid_bits;

always @(*) begin : MST_WR_REQ_VEC
    integer i;
    wr_req_bits = {OST_DEPTH{1'b0}};
    for (i=0; i<OST_DEPTH; i=i+1) begin
        wr_req_bits[i] = wr_req_buff_r[i];
    end
end

always @(*) begin : MST_WR_CLEAR_VEC
    integer i;
    wr_clear_bits = {OST_DEPTH{1'b0}};
    for (i=0; i<OST_DEPTH; i=i+1) begin
        wr_clear_bits[i] = wr_clear_buff_r[i];
    end
end

assign wr_req_en      = axi_mst_awvalid & axi_mst_awready;  
assign wr_data_en     = axi_mst_wvalid & axi_mst_wready;    
assign wr_data_last   = axi_mst_wlast;                      
assign wr_result_en   = axi_mst_bvalid & axi_mst_bready;    

genvar i;
generate
for (i=0; i<OST_DEPTH; i=i+1) begin: GEN_MST_WR_CTRL
    // Valid flag buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_valid_buff_r[i] <= #DLY 1'b0;
        end
        else if (wr_buff_set && (wr_set_ptr == i)) begin
            wr_valid_buff_r[i] <= #DLY 1'b1;
        end
        else if (wr_buff_clr && (wr_clr_ptr == i)) begin
            wr_valid_buff_r[i] <= #DLY 1'b0;
        end
    end

    // Request sent flag buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_req_buff_r[i] <= #DLY 1'b0;
        end
        else if (wr_buff_set && (wr_set_ptr == i)) begin
            wr_req_buff_r[i] <= #DLY 1'b1;
        end
        else if (wr_req_en && (wr_req_ptr == i)) begin
            wr_req_buff_r[i] <= #DLY 1'b0;
        end
    end

    // dat sent flag buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_dat_buff_r[i] <= #DLY 1'b0;
        end
        else begin
            if (wr_buff_set && (wr_set_ptr == i)) begin
                wr_dat_buff_r[i] <= #DLY 1'b0;
            end
            else if (wr_data_en && ~wr_data_last && (wr_data_ptr == i) && (wr_data_vld_r[i][(wr_data_cnt_r[i]+1)])) begin
                wr_dat_buff_r[i] <= #DLY 1'b1;
            end 
            else if (wr_data_get[i]) begin
                wr_dat_buff_r[i] <= #DLY 1'b1;
            end
            else if (wr_data_en && (wr_data_ptr == i)) begin
                wr_dat_buff_r[i] <= #DLY 1'b0;
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
        else if (wr_result_en && (wr_result_ptr == i)) begin
            wr_comp_buff_r[i] <= #DLY 1'b0;
        end
    end

    // Clear flag buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_clear_buff_r[i] <= #DLY 1'b0;
        end
        else begin
            wr_clear_buff_r[i] <= #DLY wr_valid_buff_r[i] & ~wr_req_buff_r[i] & ~wr_comp_buff_r[i];
        end
    end

//--------------------------------------------------------------------------------
// AXI AW Payload Buffer
//--------------------------------------------------------------------------------
    always @(*) begin // Burst configuration
        case (wr_req_cnt_r[1:0])  
            2'b00: begin  // INCR burst, len=4
                wr_addr_buff [i] = `AXI_ADDR_W'h0;
                wr_burst_buff[i] = `AXI_BURST_INCR;
                wr_len_buff  [i] = `AXI_LEN_W'h3;  
                wr_size_buff [i] = `AXI_SIZE_4B;
            end
            2'b01: begin  // INCR burst, len=4
                wr_addr_buff [i] = wr_req_cnt_r * `AXI_ADDR_W'h10;
                wr_burst_buff[i] = `AXI_BURST_INCR;
                wr_len_buff  [i] = `AXI_LEN_W'h3;  
                wr_size_buff [i] = `AXI_SIZE_4B;
            end
            2'b10: begin  // WRAP burst, len=4
                wr_addr_buff [i] = `AXI_ADDR_W'h24;
                wr_burst_buff[i] = `AXI_BURST_WRAP;
                wr_len_buff  [i] = `AXI_LEN_W'h3;
                wr_size_buff [i] = `AXI_SIZE_4B;
            end
            2'b11: begin  // FIXED burst, len=8
                wr_addr_buff [i] = `AXI_ADDR_W'h30;
                wr_burst_buff[i] = `AXI_BURST_FIXED;
                wr_len_buff  [i] = `AXI_LEN_W'h3;
                wr_size_buff [i] = `AXI_SIZE_4B;
            end
            default: begin  // Default INCR burst, len=4
                wr_addr_buff [i] = `AXI_ADDR_W'h80;
                wr_burst_buff[i] = `AXI_BURST_INCR;
                wr_len_buff  [i] = `AXI_LEN_W'h3;
                wr_size_buff [i] = `AXI_SIZE_4B;
            end
        endcase
    end
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_id_buff_r    [i] <= #DLY {`AXI_ID_W{1'b0}};
            wr_addr_buff_r  [i] <= #DLY {`AXI_ADDR_W{1'b0}};
            wr_len_buff_r   [i] <= #DLY {`AXI_LEN_W{1'b0}};
            wr_size_buff_r  [i] <= #DLY  `AXI_SIZE_1B;
            wr_burst_buff_r [i] <= #DLY  `AXI_BURST_INCR;
            wr_user_buff_r  [i] <= #DLY {`AXI_USER_W{1'b0}};
        end
        else if (wr_buff_set && (wr_set_ptr == i)) begin
            wr_id_buff_r    [i] <= #DLY wr_req_cnt_r[`AXI_ID_W-1:0];
            wr_addr_buff_r  [i] <= #DLY wr_addr_buff[i];
            wr_burst_buff_r [i] <= #DLY wr_burst_buff[i];
            wr_len_buff_r   [i] <= #DLY wr_len_buff[i];  
            wr_size_buff_r  [i] <= #DLY wr_size_buff[i];
            wr_user_buff_r  [i] <= #DLY wr_req_cnt_r;
        end
    end

//--------------------------------------------------------------------------------
// Burst Address Control
//--------------------------------------------------------------------------------
    assign wr_start_addr   [i] = (wr_buff_set && (wr_set_ptr == i)) ? wr_addr_buff[i]      : wr_addr_buff_r[i];
    assign wr_number_bytes [i] = (wr_buff_set && (wr_set_ptr == i)) ? 1 << wr_size_buff[i] : 1 << wr_size_buff_r[i];
    assign wr_burst_lenth  [i] = (wr_buff_set && (wr_set_ptr == i)) ? wr_len_buff[i] + 1   : wr_len_buff_r[i] + 1;
    assign wr_aligned_addr [i] = wr_start_addr[i] / wr_number_bytes[i] * wr_number_bytes[i];
    assign wr_wrap_boundary[i] = wr_start_addr[i] / (wr_burst_lenth[i] * wr_number_bytes[i]) * (wr_burst_lenth[i] * wr_number_bytes[i]);
    assign wr_wrap_en      [i] = (wr_curr_addr_r[i] + wr_number_bytes[i]) == (wr_wrap_boundary[i] + (wr_burst_lenth[i] * wr_number_bytes[i]));

    // Read index control
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_curr_index_r[i] <= #DLY {`AXI_LEN_W{1'b0}};
        end
        else if (wr_buff_set_r && (wr_set_ptr_r == i)) begin
            wr_curr_index_r[i] <= #DLY `AXI_LEN_W'h1;
        end
        else if (wr_data_en && wr_data_last && (wr_data_ptr == i)) begin
            wr_curr_index_r[i] <= #DLY {`AXI_LEN_W{1'b0}};
        end
        else if (wr_data_get[i]) begin
            wr_curr_index_r[i] <= #DLY wr_curr_index_r[i] + `AXI_LEN_W'h1;
        end
    end

    // Read wrap control
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_wrap_en_r[i] <= #DLY 1'b0;
        end
        else if (wr_buff_set_r && (wr_set_ptr_r == i)) begin
            wr_wrap_en_r[i] <= #DLY 1'b0;
        end
        else if (wr_data_get[i]) begin
            wr_wrap_en_r[i] <= #DLY wr_wrap_en_r[i] | wr_wrap_en[i];
        end
    end

    // Current address control
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_curr_addr_r[i] <= #DLY {`AXI_ADDR_W{1'b0}};
        end
        else if (wr_buff_set_r && (wr_set_ptr_r == i)) begin
            wr_curr_addr_r[i] <= #DLY wr_start_addr[i];
        end
        else if (wr_data_get[i]) begin
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
                default:wr_curr_addr_r[i] <= #DLY {`AXI_ADDR_W{1'b0}};
            endcase
        end
    end


//--------------------------------------------------------------------------------
// AXI W Payload Buffer
//--------------------------------------------------------------------------------
    // Burst data beat counter
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_data_cnt_r[i]  <= #DLY {BURST_CNT_W{1'b0}};
        end
        else if (wr_buff_set_r && (wr_set_ptr_r == i)) begin
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
            wr_strb_buff_r[i] <= #DLY {((`AXI_DATA_W/8)*MAX_BURST_LEN){1'b0}};
        end
        else if (wr_buff_set && (wr_set_ptr == i)) begin
            wr_data_vld_r [i] <= #DLY {MAX_BURST_LEN{1'b0}};
            wr_data_buff_r[i] <= #DLY {(`AXI_DATA_W*MAX_BURST_LEN){1'b0}};
            wr_strb_buff_r[i] <= #DLY {((`AXI_DATA_W/8)*MAX_BURST_LEN){1'b0}};
        end
        else if (wr_data_get[i]) begin
            wr_data_vld_r [i][(wr_curr_index_r[i]-1)] <= #DLY 1'b1;
            wr_data_buff_r[i][((wr_curr_index_r[i]-1)*`AXI_DATA_W) +: `AXI_DATA_W] <= #DLY {{`AXI_DATA_W-`AXI_ID_W-`AXI_ADDR_W{1'b0}},wr_id_buff_r[i],wr_curr_addr_r[i]};
            wr_strb_buff_r[i] <= #DLY {((`AXI_DATA_W/8)*MAX_BURST_LEN){1'b1}};
        end
    end
//--------------------------------------------------------------------------------
// AXI B Payload Buffer
//--------------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_resp_buff_r[i] <= #DLY {`AXI_RESP_W{1'b0}};
        end
        else if (wr_result_en && (wr_result_ptr == i)) begin
            wr_resp_buff_r[i] <= #DLY (axi_mst_bresp > wr_resp_buff_r[i]) ? axi_mst_bresp 
                                                                          : wr_resp_buff_r[i];
        end
    end
    assign wr_resp_err[i] = (wr_resp_buff_r[i] == `AXI_RESP_SLVERR) | 
                            (wr_resp_buff_r[i] == `AXI_RESP_DECERR);
//--------------------------------------------------------------------------------
//--------------------------------------------------------------------------------
// Simulate the data reading process
//--------------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_data_get_cnt[i] <= #DLY `AXI_DATA_GET_CNT_W'h0;
        end
        else if (wr_buff_set_r && (wr_set_ptr_r == i)) begin
            wr_data_get_cnt[i] <= #DLY `AXI_DATA_GET_CNT_W'h1;
        end
        else if (wr_data_get[i] && (wr_curr_index_r[i] < wr_burst_lenth[i])) begin
            wr_data_get_cnt[i] <= #DLY `AXI_DATA_GET_CNT_W'h1;
        end
        else if (wr_data_get_cnt[i]== MAX_GET_DATA_DLY) begin
            wr_data_get_cnt[i] <= #DLY `AXI_DATA_GET_CNT_W'h0;
        end
        else if (wr_data_get_cnt[i]>`AXI_DATA_GET_CNT_W'h0) begin
            wr_data_get_cnt[i] <= #DLY wr_data_get_cnt[i] + `AXI_DATA_GET_CNT_W'h1;
        end
    end
    assign wr_data_get[i] = wr_valid_buff_r[i] & (wr_data_get_cnt[i]==(MAX_GET_DATA_DLY - wr_id_buff_r[i]));
    assign wr_data_err[i] = (wr_id_buff_r[i] == `AXI_ID_W'hF) & (wr_curr_index_r[i] == wr_burst_lenth[i]);
end
endgenerate
//--------------------------------------------------------------------------------
// Request Completion Counter for sim wr_done
//--------------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        wr_req_cnt_r  <= #DLY {REQ_CNT_W{1'b0}};
    end
    else if (wr_buff_set) begin
        wr_req_cnt_r <= #DLY wr_req_cnt_r + 1;
    end
end
assign wr_done = (wr_req_cnt_r == {REQ_CNT_W{1'b1}});  // All requests send completed

//--------------------------------------------------------------------------------
// W ORDER CTRL
//--------------------------------------------------------------------------------
EASYAXI_ORDER #(
    .OST_DEPTH  (OST_DEPTH  ),
    .ID_WIDTH   (`AXI_ID_W  )
) U_MST_WR_ORDER_DATA (
    .clk        (clk                               ),
    .rst_n      (rst_n                             ),

    .push       (axi_mst_awvalid & axi_mst_awready ),
    .push_id    ({`AXI_ID_W{1'b0}}                 ),
    .push_ptr   (wr_req_ptr                        ),

    .pop        (axi_mst_wvalid & axi_mst_wready   ),
    .pop_id     ({`AXI_ID_W{1'b0}}                 ),
    .pop_last   (axi_mst_wlast                     ),

    .order_ptr  (wr_data_ptr                       ),
    .order_bits (                                  )
);

//--------------------------------------------------------------------------------
// RESP ID ORDER CTRL
//--------------------------------------------------------------------------------
EASYAXI_ORDER #(
    .OST_DEPTH  (OST_DEPTH  ),
    .ID_WIDTH   (`AXI_ID_W  )
) U_MST_WR_ORDER_RESP (
    .clk        (clk                               ),
    .rst_n      (rst_n                             ),

    .push       (axi_mst_awvalid & axi_mst_awready ),
    .push_id    (axi_mst_awid                      ),
    .push_ptr   (wr_req_ptr                        ),

    .pop        (axi_mst_bvalid & axi_mst_bready   ),
    .pop_id     (axi_mst_bid                       ),
    .pop_last   (1'b1                              ), // bresp only once

    .order_ptr  (wr_result_ptr                     ),
    .order_bits (                                  )
);
//--------------------------------------------------------------------------------
// Output Signal
//--------------------------------------------------------------------------------
assign axi_mst_awvalid = |wr_req_bits;
assign axi_mst_awid    = wr_id_buff_r    [wr_req_ptr];
assign axi_mst_awaddr  = wr_addr_buff_r  [wr_req_ptr];
assign axi_mst_awlen   = wr_len_buff_r   [wr_req_ptr];
assign axi_mst_awsize  = wr_size_buff_r  [wr_req_ptr];
assign axi_mst_awburst = wr_burst_buff_r [wr_req_ptr];
assign axi_mst_awuser  = wr_user_buff_r  [wr_req_ptr];

assign axi_mst_wvalid  = wr_valid_buff_r [wr_data_ptr] & wr_dat_buff_r[wr_data_ptr];
assign axi_mst_wdata   = wr_data_buff_r  [wr_data_ptr][((wr_data_cnt_r[wr_data_ptr])*`AXI_DATA_W) +: `AXI_DATA_W];
assign axi_mst_wstrb   = wr_strb_buff_r  [wr_data_ptr][((wr_data_cnt_r[wr_data_ptr])*(`AXI_DATA_W/8)) +: (`AXI_DATA_W/8)];
assign axi_mst_wlast   = axi_mst_wvalid & (wr_data_cnt_r[wr_data_ptr] == wr_len_buff_r[wr_data_ptr]);
assign axi_mst_wuser   = wr_user_buff_r  [wr_data_ptr];


assign axi_mst_bready  = 1'b1;  

endmodule
