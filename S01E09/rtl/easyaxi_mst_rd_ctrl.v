// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_mst_rd_ctrl.v
// Author        : Rongye
// Created On    : 2025-02-06 06:45
// Last Modified : 2026-03-02 22:56
// ---------------------------------------------------------------------------------
// Description   : This module implements an AXI master read controller.
//   - Supports outstanding transactions (configurable depth via OST_DEPTH)
//   - Supports all AXI burst types: FIXED, INCR, WRAP
//   - Maximum burst length: 8 beats
//   - Read data collection and response tracking
//   - Response ordering and error detection
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_MST_RD_CTRL #(
    parameter OST_DEPTH = 16,  // Outstanding transaction depth (power of 2)
    parameter LOCAL_DATA_W = 512 // internal request/response data width
)(
// Global
    input  wire                      clk,
    input  wire                      rst_n, 

// Master Request Interface
    input  wire                      m_rd_req_en,
    input  wire  [`AXI_ID_W    -1:0] m_rd_req_id,
    input  wire  [`AXI_ADDR_W  -1:0] m_rd_req_addr,
    input  wire  [7:0]               m_rd_req_size,  // Size in bytes (8-bit to support up to 256 bytes)
    input  wire  [1:0]               m_rd_req_burst,


// Master Response Interface
    output wire                      m_rd_resp_en,
    output wire  [`AXI_ID_W    -1:0] m_rd_resp_id,
    output wire  [LOCAL_DATA_W-1:0]    m_rd_resp_data,  // local data width
    output wire  [`AXI_RESP_W  -1:0] m_rd_resp_resp,

// AXI AR Channel
    output wire                      axi_mst_arvalid,
    input  wire                      axi_mst_arready,
    output wire  [`AXI_ID_W    -1:0] axi_mst_arid,
    output wire  [`AXI_ADDR_W  -1:0] axi_mst_araddr,
    output wire  [`AXI_LEN_W   -1:0] axi_mst_arlen,
    output wire  [`AXI_SIZE_W  -1:0] axi_mst_arsize,
    output wire  [`AXI_BURST_W -1:0] axi_mst_arburst,
    output wire  [`AXI_USER_W  -1:0] axi_mst_aruser,

// AXI R Channel
    input  wire                      axi_mst_rvalid,
    output wire                      axi_mst_rready,
    input  wire  [`AXI_ID_W    -1:0] axi_mst_rid,
    input  wire  [`AXI_DATA_W  -1:0] axi_mst_rdata,
    input  wire  [`AXI_RESP_W  -1:0] axi_mst_rresp,
    input  wire                      axi_mst_rlast,
    input  wire  [`AXI_USER_W  -1:0] axi_mst_ruser
);

localparam DLY = 0.1;

// Burst configuration
localparam MAX_BURST_LEN = 16;       // Maximum supported burst length for 16 beats (512-bit data)
localparam BURST_CNT_W   = $clog2(MAX_BURST_LEN); 
localparam OST_CNT_W     = OST_DEPTH == 1 ? 1 : $clog2(OST_DEPTH); 
localparam MAX_REQ_NUM   = 16;       // Maximum number of requests
localparam REQ_CNT_W     = $clog2(MAX_REQ_NUM); 

//--------------------------------------------------------------------------------
// Inner Signal 
//--------------------------------------------------------------------------------
wire                     rd_buff_set;
wire                     rd_buff_clr;
wire                     rd_buff_full;

// Outstanding request status buffers
reg                      rd_valid_buff_r[OST_DEPTH-1:0];
reg                      rd_req_buff_r  [OST_DEPTH-1:0];
reg                      rd_comp_buff_r [OST_DEPTH-1:0];
reg                      rd_clear_buff_r[OST_DEPTH-1:0];

// Bit-vector representations for status flags
reg  [OST_DEPTH    -1:0] rd_valid_bits;
wire [OST_DEPTH    -1:0] rd_set_bits;
reg  [OST_DEPTH    -1:0] rd_req_bits;
reg  [OST_DEPTH    -1:0] rd_clear_bits;

// Buffer management pointers
wire [OST_CNT_W    -1:0] rd_set_ptr;
wire [OST_CNT_W    -1:0] rd_clr_ptr;
wire [OST_CNT_W    -1:0] rd_req_ptr;
wire [OST_CNT_W    -1:0] rd_result_ptr;

// Outstanding transaction payload buffers
reg  [`AXI_ID_W    -1:0] rd_id_buff_r   [OST_DEPTH-1:0];
reg  [`AXI_ADDR_W  -1:0] rd_addr_buff_r [OST_DEPTH-1:0];
reg  [`AXI_LEN_W   -1:0] rd_len_buff_r  [OST_DEPTH-1:0];
reg  [`AXI_SIZE_W  -1:0] rd_size_buff_r [OST_DEPTH-1:0];
reg  [`AXI_BURST_W -1:0] rd_burst_buff_r[OST_DEPTH-1:0];
reg  [`AXI_USER_W  -1:0] rd_user_buff_r [OST_DEPTH-1:0];

reg  [`AXI_ADDR_W  -1:0] rd_addr_buff   [OST_DEPTH-1:0];
reg  [`AXI_LEN_W   -1:0] rd_len_buff    [OST_DEPTH-1:0];
reg  [`AXI_SIZE_W  -1:0] rd_size_buff   [OST_DEPTH-1:0];
reg  [`AXI_BURST_W -1:0] rd_burst_buff  [OST_DEPTH-1:0];

// Read data buffers (supports MAX_BURST_LEN beats per OST entry)    
reg  [`AXI_DATA_W*MAX_BURST_LEN -1:0] rd_data_buff_r [OST_DEPTH-1:0];
reg  [BURST_CNT_W               -1:0] rd_data_cnt_r  [OST_DEPTH-1:0]; // Counter for burst data
reg  [`AXI_RESP_W               -1:0] rd_resp_buff_r [OST_DEPTH-1:0];
wire [OST_DEPTH                 -1:0] rd_resp_err;                    // Error flags

wire                  rd_req_en;          // AR handshake
wire                  rd_result_en;       // R handshake
wire                  rd_result_last;     // RLAST indicator

//--------------------------------------------------------------------------------
// Pointer Management
//--------------------------------------------------------------------------------
assign rd_set_bits = ~rd_valid_bits;
EASYAXI_ARB #(
    .DEEP_NUM(OST_DEPTH)
) U_MST_RD_ARB_SET (
    .clk      (clk           ),
    .rst_n    (rst_n         ),
    .queue_i  (rd_set_bits   ),
    .sche_en  (rd_buff_set   ),
    .pointer_o(rd_set_ptr    )
);

EASYAXI_ARB #(
    .DEEP_NUM(OST_DEPTH)
) U_MST_RD_ARB_CLEAR (
    .clk      (clk           ),
    .rst_n    (rst_n         ),
    .queue_i  (rd_clear_bits ),
    .sche_en  (rd_buff_clr   ),
    .pointer_o(rd_clr_ptr    )
);

EASYAXI_ARB #(
    .DEEP_NUM(OST_DEPTH)
) U_MST_RD_ARB_REQ (
    .clk      (clk           ),
    .rst_n    (rst_n         ),
    .queue_i  (rd_req_bits   ),
    .sche_en  (rd_req_en     ),
    .pointer_o(rd_req_ptr    )
);

//--------------------------------------------------------------------------------
// Main Ctrl
//--------------------------------------------------------------------------------
assign rd_buff_set = ~rd_buff_full & m_rd_req_en;
assign rd_buff_clr = rd_valid_buff_r[rd_clr_ptr] & ~rd_req_buff_r[rd_clr_ptr] & ~rd_comp_buff_r[rd_clr_ptr];

always @(*) begin : MST_RD_VLD_VEC
    integer i;
    rd_valid_bits = {OST_DEPTH{1'b0}};
    for (i=0; i<OST_DEPTH; i=i+1) begin
        rd_valid_bits[i] = rd_valid_buff_r[i];
    end
end
assign rd_buff_full = &rd_valid_bits;

always @(*) begin : MST_RD_REQ_VEC
    integer i;
    rd_req_bits = {OST_DEPTH{1'b0}};
    for (i=0; i<OST_DEPTH; i=i+1) begin
        rd_req_bits[i] = rd_req_buff_r[i];
    end
end

always @(*) begin : MST_RD_CLEAR_VEC
    integer i;
    rd_clear_bits = {OST_DEPTH{1'b0}};
    for (i=0; i<OST_DEPTH; i=i+1) begin
        rd_clear_bits[i] = rd_clear_buff_r[i];
    end
end

assign rd_req_en      = axi_mst_arvalid & axi_mst_arready;  // AR handshake
assign rd_result_en   = axi_mst_rvalid & axi_mst_rready;    // R handshake
assign rd_result_last = axi_mst_rlast;                     // Burst end flag

genvar i;
generate
for (i=0; i<OST_DEPTH; i=i+1) begin: GEN_MST_RD_CTRL
    // Valid flag buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_valid_buff_r[i] <= #DLY 1'b0;
        end
        else if (rd_buff_set && (rd_set_ptr == i)) begin
            rd_valid_buff_r[i] <= #DLY 1'b1;
        end
        else if (rd_buff_clr && (rd_clr_ptr == i)) begin
            rd_valid_buff_r[i] <= #DLY 1'b0;
        end
    end

    // Request sent flag buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_req_buff_r[i] <= #DLY 1'b0;
        end
        else if (rd_buff_set && (rd_set_ptr == i)) begin
            rd_req_buff_r[i] <= #DLY 1'b1;
        end
        else if (rd_req_en && (rd_req_ptr == i)) begin
            rd_req_buff_r[i] <= #DLY 1'b0;
        end
    end

    // Completion flag buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_comp_buff_r[i] <= #DLY 1'b0;
        end
        else if (rd_buff_set && (rd_set_ptr == i)) begin
            rd_comp_buff_r[i] <= #DLY 1'b1;
        end
        else if (rd_result_en && rd_result_last && (rd_result_ptr == i)) begin
            rd_comp_buff_r[i] <= #DLY 1'b0;
        end
    end

    // Clear flag buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_clear_buff_r[i] <= #DLY 1'b0;
        end
        else begin
            rd_clear_buff_r[i] <= #DLY rd_valid_buff_r[i] & ~rd_req_buff_r[i] & ~rd_comp_buff_r[i];
        end
    end

//--------------------------------------------------------------------------------
// AXI AR Payload Buffer
//--------------------------------------------------------------------------------
    always @(*) begin // Burst configuration from request interface
        rd_addr_buff [i] = m_rd_req_addr;
        rd_burst_buff[i] = m_rd_req_burst;
        
        // -----------------------------------------------------------------------
        // Map byte size to AXI AxSIZE and AxLEN parameters.
        // Identical logic to MST_WR_CTRL; read requests only need byte count.
        // -----------------------------------------------------------------------
        // Convert byte size to AXI SIZE encoding
        // AXI SIZE: 0=1B, 1=2B, 2=4B, 3=8B, 4=16B, 5=32B, 6=64B, 7=128B
        // m_rd_req_size is in bytes, need to convert to log2(size)
        case (m_rd_req_size)
            8'h01: rd_size_buff[i] = 3'b000;  // 1 byte
            8'h02: rd_size_buff[i] = 3'b001;  // 2 bytes
            8'h04: rd_size_buff[i] = 3'b010;  // 4 bytes
            8'h08: rd_size_buff[i] = 3'b011;  // 8 bytes
            8'h10: rd_size_buff[i] = 3'b100;  // 16 bytes
            8'h20: rd_size_buff[i] = 3'b101;  // 32 bytes
            8'h40: rd_size_buff[i] = 3'b110;  // 64 bytes
            8'h80: rd_size_buff[i] = 3'b111;  // 128 bytes
            default: rd_size_buff[i] = 3'b010; // default to 4 bytes
        endcase
        
        // -----------------------------------------------------------------------
        // Burst length calculation.
        // Read direction: AXI R beats collected into internal data; length mapping symmetric with write.
        // -----------------------------------------------------------------------
        // Calculate burst length based on local data width (512-bit = 64B) and AXI data width (32-bit = 4B)
        // For 64B transfer: 64B / 4B = 16 beats -> len = 15
        // For 16B transfer: 16B / 4B = 4 beats -> len = 3
        // For 8B transfer: 8B / 4B = 2 beats -> len = 1
        // For 4B transfer: 4B / 4B = 1 beat -> len = 0
        // local data may be up to 512-bit, only the portion specified by size is used
        case (m_rd_req_size)
            8'h01: rd_len_buff[i] = 8'h0;   // 1 byte -> 1 beat (but AXI min is 4B, so use 1 beat)
            8'h02: rd_len_buff[i] = 8'h0;   // 2 bytes -> 1 beat
            8'h04: rd_len_buff[i] = 8'h0;   // 4 bytes -> 1 beat
            8'h08: rd_len_buff[i] = 8'h1;   // 8 bytes -> 2 beats
            8'h10: rd_len_buff[i] = 8'h3;   // 16 bytes -> 4 beats
            8'h20: rd_len_buff[i] = 8'h7;   // 32 bytes -> 8 beats
            8'h40: rd_len_buff[i] = 8'hF;   // 64 bytes -> 16 beats
            8'h80: rd_len_buff[i] = 8'h1F;  // 128 bytes -> 32 beats (but limited by MAX_BURST_LEN)
            default: rd_len_buff[i] = 8'h0; // default to 1 beat
        endcase

    end
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_id_buff_r    [i] <= #DLY {`AXI_ID_W{1'b0}};
            rd_addr_buff_r  [i] <= #DLY {`AXI_ADDR_W{1'b0}};
            rd_len_buff_r   [i] <= #DLY {`AXI_LEN_W{1'b0}};
            rd_size_buff_r  [i] <= #DLY  `AXI_SIZE_1B;
            rd_burst_buff_r [i] <= #DLY  `AXI_BURST_INCR;
            rd_user_buff_r  [i] <= #DLY {`AXI_USER_W{1'b0}};
        end
        else if (rd_buff_set && (rd_set_ptr == i)) begin
            rd_id_buff_r    [i] <= #DLY m_rd_req_id;
            rd_addr_buff_r  [i] <= #DLY rd_addr_buff[i];
            rd_burst_buff_r [i] <= #DLY rd_burst_buff[i];
            rd_len_buff_r   [i] <= #DLY rd_len_buff[i];  
            rd_size_buff_r  [i] <= #DLY rd_size_buff[i];
            rd_user_buff_r  [i] <= #DLY {`AXI_USER_W{1'b0}};
        end
    end

//--------------------------------------------------------------------------------
// AXI R Payload Buffer
//--------------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_resp_buff_r[i] <= #DLY {`AXI_RESP_W{1'b0}};
        end
        else if (rd_result_en && (rd_result_ptr == i)) begin
            rd_resp_buff_r[i] <= #DLY (axi_mst_rresp > rd_resp_buff_r[i]) ? axi_mst_rresp 
                                                                          : rd_resp_buff_r[i];
        end
    end
    assign rd_resp_err[i] = (rd_resp_buff_r[i] == `AXI_RESP_SLVERR) | 
                            (rd_resp_buff_r[i] == `AXI_RESP_DECERR);

    // Burst data beat counter
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_data_cnt_r[i]  <= #DLY {BURST_CNT_W{1'b0}};
        end
        else if (rd_buff_set && (rd_set_ptr == i)) begin
            rd_data_cnt_r[i]  <= #DLY {BURST_CNT_W{1'b0}};
        end
        else if (rd_result_en && (rd_result_ptr == i)) begin
            rd_data_cnt_r[i] <= #DLY rd_data_cnt_r[i] + 1;
        end
    end
    
    // Burst data buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_data_buff_r[i] <= #DLY {(`AXI_DATA_W*MAX_BURST_LEN){1'b0}};
        end
        else if (rd_buff_set && (rd_set_ptr == i)) begin
            rd_data_buff_r[i] <= #DLY {(`AXI_DATA_W*MAX_BURST_LEN){1'b0}};
        end
        else if (rd_result_en && (rd_result_ptr == i)) begin
            rd_data_buff_r[i][(rd_data_cnt_r[i]*`AXI_DATA_W) +: `AXI_DATA_W] <= #DLY axi_mst_rdata;
        end
    end
end
endgenerate

// Response Interface Output Logic
reg  m_rd_resp_en_r;
reg  [`AXI_ID_W    -1:0] m_rd_resp_id_r;
reg  [LOCAL_DATA_W-1:0]    m_rd_resp_data_r;
reg  [`AXI_RESP_W  -1:0] m_rd_resp_resp_r;

// Response output logic - return result when buffer is cleared
always @(posedge clk or negedge rst_n) begin
    integer beat;
    if (~rst_n) begin
        m_rd_resp_en_r    <= #DLY 1'b0;
        m_rd_resp_id_r    <= #DLY {`AXI_ID_W{1'b0}};
        m_rd_resp_data_r  <= #DLY {LOCAL_DATA_W{1'b0}};
        m_rd_resp_resp_r  <= #DLY {`AXI_RESP_W{1'b0}};
    end
    else if (rd_buff_clr) begin
        // Output response when buffer entry is being cleared
        m_rd_resp_en_r    <= #DLY 1'b1;
        m_rd_resp_id_r    <= #DLY rd_id_buff_r[rd_clr_ptr];
        // Combine 16x 32-bit AXI data into 512-bit local data
        // For 64B transfer, we need to combine all 16 beats
        m_rd_resp_data_r  <= #DLY {LOCAL_DATA_W{1'b0}};
        for (beat = 0; beat < 16; beat = beat + 1) begin
            if (beat < MAX_BURST_LEN) begin
                m_rd_resp_data_r[(beat*`AXI_DATA_W) +: `AXI_DATA_W] <= #DLY 
                    rd_data_buff_r[rd_clr_ptr][(beat*`AXI_DATA_W) +: `AXI_DATA_W];
            end
        end
        m_rd_resp_resp_r  <= #DLY rd_resp_buff_r[rd_clr_ptr];
    end
    else begin
        m_rd_resp_en_r    <= #DLY 1'b0;
    end
end

assign m_rd_resp_en   = m_rd_resp_en_r;
assign m_rd_resp_id   = m_rd_resp_id_r;
assign m_rd_resp_data = m_rd_resp_data_r;
assign m_rd_resp_resp = m_rd_resp_resp_r;

//--------------------------------------------------------------------------------
// RESP ID ORDER CTRL
//--------------------------------------------------------------------------------
EASYAXI_ORDER #(
    .OST_DEPTH(OST_DEPTH),
    .ID_WIDTH (`AXI_ID_W)
) U_MST_RD_ORDER_RESP (
    .clk        (clk                               ),
    .rst_n      (rst_n                             ),

    .push       (axi_mst_arvalid & axi_mst_arready ),
    .push_id    (axi_mst_arid                      ),
    .push_ptr   (rd_req_ptr                        ),

    .pop        (axi_mst_rvalid & axi_mst_rready   ),
    .pop_id     (axi_mst_rid                       ),
    .pop_last   (axi_mst_rlast                     ),

    .order_ptr  (rd_result_ptr                     ),
    .order_bits (                                  ),
    .order_empty(                                  )
);
//--------------------------------------------------------------------------------
// Output Signal
// req -> AXI AR mapping, symmetric with MST_WR AW mapping.
//   rready=1: Master always ready to accept read data for max throughput.
//   R beats collected in rd_data_buff_r; output m_rd_resp_data on rd_buff_clr.
//--------------------------------------------------------------------------------

assign axi_mst_arvalid = |rd_req_bits;
assign axi_mst_arid    = rd_id_buff_r    [rd_req_ptr];
assign axi_mst_araddr  = rd_addr_buff_r  [rd_req_ptr];
assign axi_mst_arlen   = rd_len_buff_r   [rd_req_ptr];
assign axi_mst_arsize  = rd_size_buff_r  [rd_req_ptr];
assign axi_mst_arburst = rd_burst_buff_r [rd_req_ptr];
assign axi_mst_aruser  = rd_user_buff_r  [rd_req_ptr];

assign axi_mst_rready  = 1'b1;  

endmodule
