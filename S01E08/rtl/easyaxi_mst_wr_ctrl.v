// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_mst_wr_ctrl.v
// Author        : Rongye
// Created On    : 2025-02-06 06:45
// Last Modified : 2025-12-20 06:50
// ---------------------------------------------------------------------------------
// Description   : AXI Master with burst support up to length 8 and outstanding capability
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
localparam MAX_BURST_LEN = 8;       // Maximum supported burst length
localparam BURST_CNT_W   = $clog2(MAX_BURST_LEN); 
localparam OST_CNT_W     = OST_DEPTH == 1 ? 1 : $clog2(OST_DEPTH); 
localparam MAX_REQ_NUM   = 32;       // Maximum number of requests
localparam REQ_CNT_W     = $clog2(MAX_REQ_NUM); 

//--------------------------------------------------------------------------------
// Inner Signal 
//--------------------------------------------------------------------------------
wire                                  wr_buff_set;
wire                                  wr_buff_clr;
wire                                  wr_buff_full;

// Outstanding request status buffers
reg                                   wr_valid_buff_r[OST_DEPTH-1:0];
reg                                   wr_req_buff_r  [OST_DEPTH-1:0];
reg                                   wr_comp_buff_r [OST_DEPTH-1:0];
reg                                   wr_clear_buff_r[OST_DEPTH-1:0];

// Bit-vector representations for status flags
reg  [OST_DEPTH    -1:0] wr_valid_bits;
wire [OST_DEPTH    -1:0] wr_set_bits;
reg  [OST_DEPTH    -1:0] wr_req_bits;
reg  [OST_DEPTH    -1:0] wr_clear_bits;

// Buffer management pointers
wire [OST_CNT_W    -1:0] wr_set_ptr_r;
wire [OST_CNT_W    -1:0] wr_clr_ptr_r;
wire [OST_CNT_W    -1:0] wr_req_ptr_r;
wire [OST_CNT_W    -1:0] wr_result_ptr_r;

// Outstanding transaction payload buffers
reg  [`AXI_ID_W    -1:0] wr_id_buff_r   [OST_DEPTH-1:0];
reg  [`AXI_ADDR_W  -1:0] wr_addr_buff_r [OST_DEPTH-1:0];
reg  [`AXI_LEN_W   -1:0] wr_len_buff_r  [OST_DEPTH-1:0];
reg  [`AXI_SIZE_W  -1:0] wr_size_buff_r [OST_DEPTH-1:0];
reg  [`AXI_BURST_W -1:0] wr_burst_buff_r[OST_DEPTH-1:0];
reg  [`AXI_USER_W  -1:0] wr_user_buff_r [OST_DEPTH-1:0];
    
// Read data buffers (supports MAX_BURST_LEN beats per OST entry)    
reg  [`AXI_DATA_W*MAX_BURST_LEN -1:0] wr_data_buff_r [OST_DEPTH-1:0];
reg  [BURST_CNT_W               -1:0] wr_data_cnt_r  [OST_DEPTH-1:0]; // Counter for burst data
reg  [`AXI_RESP_W               -1:0] wr_resp_buff_r [OST_DEPTH-1:0];
wire [OST_DEPTH                 -1:0] wr_resp_err;                    // Error flags

wire                  wr_req_en;          // AW handshake
wire                  wr_result_en;       // B handshake
wire                  wr_result_last;     // RLAST indicator
reg  [REQ_CNT_W -1:0] wr_req_cnt_r;       // Completed request counter

//--------------------------------------------------------------------------------
// Pointer Management
//--------------------------------------------------------------------------------
assign wr_set_bits = ~wr_valid_bits;
EASYAXI_ARB #(
    .DEEP_NUM(OST_DEPTH)
) U_WR_SET_ARB (
    .clk      (clk           ),
    .rst_n    (rst_n         ),
    .queue_i  (wr_set_bits   ),
    .sche_en  (wr_buff_set   ),
    .pointer_o(wr_set_ptr_r  )
);

EASYAXI_ARB #(
    .DEEP_NUM(OST_DEPTH)
) U_WR_CLEAR_ARB (
    .clk      (clk           ),
    .rst_n    (rst_n         ),
    .queue_i  (wr_clear_bits ),
    .sche_en  (wr_buff_clr   ),
    .pointer_o(wr_clr_ptr_r  )
);

EASYAXI_ARB #(
    .DEEP_NUM(OST_DEPTH)
) U_WR_REQ_ARB (
    .clk      (clk         ),
    .rst_n    (rst_n       ),
    .queue_i  (wr_req_bits ),
    .sche_en  (wr_req_en   ),
    .pointer_o(wr_req_ptr_r)
);

//--------------------------------------------------------------------------------
// Main Ctrl
//--------------------------------------------------------------------------------
assign wr_buff_set = ~wr_buff_full & wr_en;
assign wr_buff_clr = wr_valid_buff_r[wr_clr_ptr_r] & ~wr_req_buff_r[wr_clr_ptr_r] & ~wr_comp_buff_r[wr_clr_ptr_r];

always @(*) begin : GEN_VLD_VEC
    integer i;
    wr_valid_bits = {OST_DEPTH{1'b0}};
    for (i=0; i<OST_DEPTH; i=i+1) begin
        wr_valid_bits[i] = wr_valid_buff_r[i];
    end
end
assign wr_buff_full = &wr_valid_bits;

always @(*) begin : GEN_REQ_VEC
    integer i;
    wr_req_bits = {OST_DEPTH{1'b0}};
    for (i=0; i<OST_DEPTH; i=i+1) begin
        wr_req_bits[i] = wr_req_buff_r[i];
    end
end

always @(*) begin : GEN_CLEAR_VEC
    integer i;
    wr_clear_bits = {OST_DEPTH{1'b0}};
    for (i=0; i<OST_DEPTH; i=i+1) begin
        wr_clear_bits[i] = wr_clear_buff_r[i];
    end
end

assign wr_req_en      = axi_mst_awvalid & axi_mst_awready;  // AW handshake
assign wr_result_en   = axi_mst_bvalid & axi_mst_bready;    // B handshake
assign wr_result_last = 1'b0/* axi_mst_wlast */;                     // Burst end flag

genvar i;
generate
for (i=0; i<OST_DEPTH; i=i+1) begin: OST_BUFFERS
    // Valid flag buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_valid_buff_r[i] <= #DLY 1'b0;
        end
        else if (wr_buff_set && (wr_set_ptr_r == i)) begin
            wr_valid_buff_r[i] <= #DLY 1'b1;
        end
        else if (wr_buff_clr && (wr_clr_ptr_r == i)) begin
            wr_valid_buff_r[i] <= #DLY 1'b0;
        end
    end

    // Request sent flag buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_req_buff_r[i] <= #DLY 1'b0;
        end
        else if (wr_buff_set && (wr_set_ptr_r == i)) begin
            wr_req_buff_r[i] <= #DLY 1'b1;
        end
        else if (wr_req_en && (wr_req_ptr_r == i)) begin
            wr_req_buff_r[i] <= #DLY 1'b0;
        end
    end

    // Completion flag buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_comp_buff_r[i] <= #DLY 1'b0;
        end
        else if (wr_buff_set && (wr_set_ptr_r == i)) begin
            wr_comp_buff_r[i] <= #DLY 1'b1;
        end
        else if (wr_result_en && wr_result_last && (wr_result_ptr_r == i)) begin
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
end
endgenerate

//--------------------------------------------------------------------------------
// AXI AW Payload Buffer
//--------------------------------------------------------------------------------
generate
for (i=0; i<OST_DEPTH; i=i+1) begin: AW_PAYLOAD
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_id_buff_r    [i] <= #DLY {`AXI_ID_W{1'b0}};
            wr_addr_buff_r  [i] <= #DLY {`AXI_ADDR_W{1'b0}};
            wr_len_buff_r   [i] <= #DLY {`AXI_LEN_W{1'b0}};
            wr_size_buff_r  [i] <= #DLY  `AXI_SIZE_1B;
            wr_burst_buff_r [i] <= #DLY  `AXI_BURST_INCR;
            wr_user_buff_r  [i] <= #DLY {`AXI_USER_W{1'b0}};
        end
        else if (wr_buff_set && (wr_set_ptr_r == i)) begin
            // wr_id_buff_r    [i] <= #DLY i;
            wr_id_buff_r    [i] <= #DLY wr_req_cnt_r[`AXI_ID_W-1:0];
            wr_user_buff_r  [i] <= #DLY wr_req_cnt_r;
            // Burst configuration
            case (i[1:0])  // Use i for case selection
                3'b000: begin  // INCR burst, len=4
                    wr_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h0;
                    wr_burst_buff_r[i] <= #DLY `AXI_BURST_INCR;
                    wr_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;  
                    wr_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                3'b001: begin  // INCR burst, len=4
                    wr_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h10;
                    wr_burst_buff_r[i] <= #DLY `AXI_BURST_INCR;
                    wr_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;  
                    wr_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                3'b010: begin  // INCR burst, len=4
                    wr_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h20;
                    wr_burst_buff_r[i] <= #DLY `AXI_BURST_INCR;
                    wr_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;
                    wr_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                3'b011: begin  // FIXED burst, len=8
                    wr_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h30;
                    wr_burst_buff_r[i] <= #DLY `AXI_BURST_FIXED;
                    wr_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;
                    wr_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                3'b100: begin  // WRAP burst, len=4
                    wr_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h34;  // Must be aligned to 16B for 4x4B
                    wr_burst_buff_r[i] <= #DLY `AXI_BURST_WRAP;
                    wr_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;
                    wr_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                3'b101: begin  // WRAP burst, len=8
                    wr_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h38;  // Must be aligned to 32B for 8x4B
                    wr_burst_buff_r[i] <= #DLY `AXI_BURST_WRAP;
                    wr_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;
                    wr_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                3'b110: begin  // FIXED burst, len=8
                    wr_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h40;  // Must be aligned to 32B for 8x4B
                    wr_burst_buff_r[i] <= #DLY `AXI_BURST_FIXED;
                    wr_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;
                    wr_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                3'b111: begin  // INCR burst, len=4
                    wr_addr_buff_r [i] <= #DLY wr_addr_buff_r [i] + `AXI_ADDR_W'h20;
                    wr_burst_buff_r[i] <= #DLY `AXI_BURST_INCR;
                    wr_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;  
                    wr_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                default: begin  // Default INCR burst, len=4
                    wr_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h80;
                    wr_burst_buff_r[i] <= #DLY `AXI_BURST_INCR;
                    wr_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;
                    wr_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
            endcase
        end
    end
end
endgenerate

//--------------------------------------------------------------------------------
// AXI W Payload Buffer
//--------------------------------------------------------------------------------
generate
for (i=0; i<OST_DEPTH; i=i+1) begin: W_PAYLOAD
    // always @(posedge clk or negedge rst_n) begin
        // if (~rst_n) begin
            // wr_resp_buff_r[i] <= #DLY {`AXI_RESP_W{1'b0}};
        // end
        // else if (wr_result_en && (wr_result_ptr_r == i)) begin
            // wr_resp_buff_r[i] <= #DLY (axi_mst_bresp > wr_resp_buff_r[i]) ? axi_mst_bresp 
                                                                          // : wr_resp_buff_r[i];
        // end
    // end
    // assign wr_resp_err[i] = (wr_resp_buff_r[i] == `AXI_RESP_SLVERR) | 
                            // (wr_resp_buff_r[i] == `AXI_RESP_DECERR);

    // Burst data beat counter
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_data_cnt_r[i]  <= #DLY {BURST_CNT_W{1'b0}};
        end
        else if (wr_buff_set && (wr_set_ptr_r == i)) begin
            wr_data_cnt_r[i]  <= #DLY {BURST_CNT_W{1'b0}};
        end
        else if (wr_result_en && (wr_result_ptr_r == i)) begin
            wr_data_cnt_r[i] <= #DLY wr_data_cnt_r[i] + 1;
        end
    end
    
    // Burst data buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            wr_data_buff_r[i*(`AXI_DATA_W*MAX_BURST_LEN)] <= #DLY {(`AXI_DATA_W*MAX_BURST_LEN){1'b0}};
        end
        else if (wr_buff_set && (wr_set_ptr_r == i)) begin
            wr_data_buff_r[i*(`AXI_DATA_W*MAX_BURST_LEN)] <= #DLY {MAX_BURST_LEN{{`AXI_DATA_W-`AXI_ID_W-`AXI_ADDR_W{1'b0}},wr_id_buff_r[i],wr_curr_addr_r[i]}};
        end
    end
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

//--------------------------------------------------------------------------------
// RESP ID ORDER CTRL
//--------------------------------------------------------------------------------
EASYAXI_ORDER #(
    .OST_DEPTH(OST_DEPTH),
    .ID_WIDTH (`AXI_ID_W)
) U_EASYAXI_SLV_RD_ORDER (
    .clk        (clk             ),
    .rst_n      (rst_n           ),

    .req_valid  (axi_mst_awvalid ),
    .req_ready  (axi_mst_awready ),
    .req_id     (axi_mst_awid    ),
    .req_ptr    (rd_set_ptr_r    ),

    .resp_valid (axi_slv_rvalid  ),
    .resp_ready (axi_slv_rready  ),
    .resp_id    (axi_slv_rid     ),
    .resp_last  (axi_slv_rlast   ),

    .resp_ptr   (                ),
    .resp_bits  (rd_order_bits   )
);
//--------------------------------------------------------------------------------
// Output Signal
//--------------------------------------------------------------------------------
assign wr_done = (wr_req_cnt_r == {REQ_CNT_W{1'b1}});  // All requests completed

assign axi_mst_awvalid = |wr_req_bits;
assign axi_mst_awid    = wr_id_buff_r    [wr_req_ptr_r];
assign axi_mst_awaddr  = wr_addr_buff_r  [wr_req_ptr_r];
assign axi_mst_awlen   = wr_len_buff_r   [wr_req_ptr_r];
assign axi_mst_awsize  = wr_size_buff_r  [wr_req_ptr_r];
assign axi_mst_awburst = wr_burst_buff_r [wr_req_ptr_r];
assign axi_mst_awuser  = wr_user_buff_r  [wr_req_ptr_r];

assign axi_mst_bready  = 1'b1;  

endmodule
