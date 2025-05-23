// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_mst.v
// Author        : Rongye
// Created On    : 2025-02-06 06:45
// Last Modified : 2025-05-24 04:49
// ---------------------------------------------------------------------------------
// Description   : AXI Master with burst support up to length 8 and outstanding capability
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_MST #(
    parameter OST_DEPTH = 32  // Outstanding transaction depth (power of 2)
)(
// Global
    input  wire                      clk,
    input  wire                      rst_n, 
    input  wire                      enable,
    output wire                      done,

// AXI AR Channel
    output wire                      axi_mst_arvalid,
    input  wire                      axi_mst_arready,
    output wire  [`AXI_ID_W    -1:0] axi_mst_arid,
    output wire  [`AXI_ADDR_W  -1:0] axi_mst_araddr,
    output wire  [`AXI_LEN_W   -1:0] axi_mst_arlen,
    output wire  [`AXI_SIZE_W  -1:0] axi_mst_arsize,
    output wire  [`AXI_BURST_W -1:0] axi_mst_arburst,

// AXI R Channel
    input  wire                      axi_mst_rvalid,
    output wire                      axi_mst_rready,
    input  wire  [`AXI_ID_W    -1:0] axi_mst_rid,
    input  wire  [`AXI_DATA_W  -1:0] axi_mst_rdata,
    input  wire  [`AXI_RESP_W  -1:0] axi_mst_rresp,
    input  wire                      axi_mst_rlast
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
wire                                  rd_buff_set;
wire                                  rd_buff_clr;
wire                                  rd_buff_full;

// Outstanding request status buffers
reg                                   rd_valid_buff_r[OST_DEPTH-1:0];
reg                                   rd_req_buff_r  [OST_DEPTH-1:0];
reg                                   rd_comp_buff_r [OST_DEPTH-1:0];

// Bit-vector representations for status flags
reg  [OST_DEPTH    -1:0] rd_valid_bits;
reg  [OST_DEPTH    -1:0] rd_req_bits;

// Buffer management pointers
reg  [OST_CNT_W    -1:0] rd_set_ptr_r;
reg  [OST_CNT_W    -1:0] rd_clr_ptr_r;
reg  [OST_CNT_W    -1:0] rd_req_ptr_r;

// Outstanding transaction payload buffers
reg  [`AXI_ID_W    -1:0] rd_id_buff_r   [OST_DEPTH-1:0];
reg  [`AXI_ADDR_W  -1:0] rd_addr_buff_r [OST_DEPTH-1:0];
reg  [`AXI_LEN_W   -1:0] rd_len_buff_r  [OST_DEPTH-1:0];
reg  [`AXI_SIZE_W  -1:0] rd_size_buff_r [OST_DEPTH-1:0];
reg  [`AXI_BURST_W -1:0] rd_burst_buff_r[OST_DEPTH-1:0];
    
// Read data buffers (supports MAX_BURST_LEN beats per OST entry)    
reg  [`AXI_DATA_W*MAX_BURST_LEN -1:0] rd_data_buff_r [OST_DEPTH-1:0];
reg  [BURST_CNT_W               -1:0] rd_data_cnt_r  [OST_DEPTH-1:0]; // Counter for burst data
reg  [`AXI_RESP_W               -1:0] rd_resp_buff_r [OST_DEPTH-1:0];
wire [OST_DEPTH                 -1:0] rd_resp_err;                    // Error flags

wire                  rd_req_en;          // AR handshake
wire                  rd_result_en;       // R handshake
wire [`AXI_ID_W -1:0] rd_result_id;       // Current RID
wire                  rd_result_last;     // RLAST indicator
reg  [REQ_CNT_W -1:0] rd_req_cnt_r;       // Completed request counter

//--------------------------------------------------------------------------------
// Pointer Management
//--------------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_set_ptr_r  <= #DLY {OST_CNT_W{1'b0}};
    end
    else if (rd_buff_set) begin
        rd_set_ptr_r  <= #DLY ((rd_set_ptr_r + 1) < OST_DEPTH) ? rd_set_ptr_r + 1 : {OST_CNT_W{1'b0}};
    end
end

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_clr_ptr_r <= #DLY {OST_CNT_W{1'b0}};
    end
    else if (rd_buff_clr) begin
        rd_clr_ptr_r <= #DLY ((rd_clr_ptr_r + 1) < OST_DEPTH) ? rd_clr_ptr_r + 1 : {OST_CNT_W{1'b0}};
    end
end

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_req_ptr_r  <= #DLY {OST_CNT_W{1'b0}};
    end
    else if (rd_req_en) begin
        rd_req_ptr_r  <= #DLY ((rd_req_ptr_r + 1) < OST_DEPTH) ? rd_req_ptr_r + 1 : {OST_CNT_W{1'b0}};
    end
end

//--------------------------------------------------------------------------------
// Main Ctrl
//--------------------------------------------------------------------------------
assign rd_buff_set = ~rd_buff_full & enable;
assign rd_buff_clr = rd_valid_buff_r[rd_clr_ptr_r] & ~rd_req_buff_r[rd_clr_ptr_r] & ~rd_comp_buff_r[rd_clr_ptr_r];

always @(*) begin : GEN_VLD_VEC
    integer i;
    rd_valid_bits = {OST_DEPTH{1'b0}};
    for (i=0; i<OST_DEPTH; i=i+1) begin
        rd_valid_bits[i] = rd_valid_buff_r[i];
    end
end
assign rd_buff_full = &rd_valid_bits;

assign rd_req_en      = axi_mst_arvalid & axi_mst_arready;  // AR handshake
assign rd_result_en   = axi_mst_rvalid & axi_mst_rready;    // R handshake
assign rd_result_id   = axi_mst_rid;                       // Current RID
assign rd_result_last = axi_mst_rlast;                     // Burst end flag

genvar i;
generate
for (i=0; i<OST_DEPTH; i=i+1) begin: OST_BUFFERS
    // Valid flag buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_valid_buff_r[i] <= #DLY 1'b0;
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_valid_buff_r[i] <= #DLY 1'b1;
        end
        else if (rd_buff_clr && (rd_clr_ptr_r == i)) begin
            rd_valid_buff_r[i] <= #DLY 1'b0;
        end
    end

    // Request sent flag buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_req_buff_r[i] <= #DLY 1'b0;
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_req_buff_r[i] <= #DLY 1'b1;
        end
        else if (rd_req_en && (rd_req_ptr_r == i)) begin
            rd_req_buff_r[i] <= #DLY 1'b0;
        end
    end

    // Completion flag buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_comp_buff_r[i] <= #DLY 1'b0;
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_comp_buff_r[i] <= #DLY 1'b1;
        end
        else if (rd_result_en && rd_result_last && (rd_result_id == rd_id_buff_r[i])) begin
            rd_comp_buff_r[i] <= #DLY 1'b0;
        end
    end
end
endgenerate

//--------------------------------------------------------------------------------
// AXI AR Payload Buffer
//--------------------------------------------------------------------------------
generate
for (i=0; i<OST_DEPTH; i=i+1) begin: AR_PAYLOAD
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_id_buff_r    [i] <= #DLY{`AXI_ID_W{1'b0}};
            rd_addr_buff_r  [i] <= #DLY{`AXI_ADDR_W{1'b0}};
            rd_len_buff_r   [i] <= #DLY{`AXI_LEN_W{1'b0}};
            rd_size_buff_r  [i] <= #DLY `AXI_SIZE_1B;
            rd_burst_buff_r [i] <= #DLY `AXI_BURST_INCR;
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_id_buff_r    [i] <= #DLY i;
            // Burst configuration
            case (i[1:0])  // Use i for case selection
                3'b000: begin  // INCR burst, len=4
                    rd_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h0;
                    rd_burst_buff_r[i] <= #DLY `AXI_BURST_INCR;
                    rd_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;  
                    rd_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                3'b001: begin  // INCR burst, len=4
                    rd_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h10;
                    rd_burst_buff_r[i] <= #DLY `AXI_BURST_INCR;
                    rd_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;  
                    rd_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                3'b010: begin  // INCR burst, len=4
                    rd_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h20;
                    rd_burst_buff_r[i] <= #DLY `AXI_BURST_INCR;
                    rd_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;
                    rd_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                3'b011: begin  // FIXED burst, len=8
                    rd_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h30;
                    rd_burst_buff_r[i] <= #DLY `AXI_BURST_FIXED;
                    rd_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;
                    rd_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                3'b100: begin  // WRAP burst, len=4
                    rd_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h34;  // Must be aligned to 16B for 4x4B
                    rd_burst_buff_r[i] <= #DLY `AXI_BURST_WRAP;
                    rd_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;
                    rd_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                3'b101: begin  // WRAP burst, len=8
                    rd_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h38;  // Must be aligned to 32B for 8x4B
                    rd_burst_buff_r[i] <= #DLY `AXI_BURST_WRAP;
                    rd_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;
                    rd_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                3'b110: begin  // FIXED burst, len=8
                    rd_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h40;  // Must be aligned to 32B for 8x4B
                    rd_burst_buff_r[i] <= #DLY `AXI_BURST_FIXED;
                    rd_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;
                    rd_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                3'b111: begin  // INCR burst, len=4
                    rd_addr_buff_r [i] <= #DLY rd_addr_buff_r [i] + `AXI_ADDR_W'h20;
                    rd_burst_buff_r[i] <= #DLY `AXI_BURST_INCR;
                    rd_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;  
                    rd_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
                default: begin  // Default INCR burst, len=4
                    rd_addr_buff_r [i] <= #DLY `AXI_ADDR_W'h80;
                    rd_burst_buff_r[i] <= #DLY `AXI_BURST_INCR;
                    rd_len_buff_r  [i] <= #DLY `AXI_LEN_W'h3;
                    rd_size_buff_r [i] <= #DLY `AXI_SIZE_4B;
                end
            endcase
        end
    end
end
endgenerate

//--------------------------------------------------------------------------------
// AXI R Payload Buffer
//--------------------------------------------------------------------------------
generate
for (i=0; i<OST_DEPTH; i=i+1) begin: R_PAYLOAD
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_resp_buff_r[i] <= #DLY {`AXI_RESP_W{1'b0}};
        end
        else if (rd_result_en && (rd_result_id == rd_id_buff_r[i])) begin
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
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_data_cnt_r[i]  <= #DLY {BURST_CNT_W{1'b0}};
        end
        else if (rd_result_en && (rd_result_id == rd_id_buff_r[i])) begin
            rd_data_cnt_r[i] <= #DLY rd_data_cnt_r[i] + 1;
        end
    end
    
    // Burst data buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_data_buff_r[i] <= #DLY {(`AXI_DATA_W*MAX_BURST_LEN){1'b0}};
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_data_buff_r[i] <= #DLY {(`AXI_DATA_W*MAX_BURST_LEN){1'b0}};
        end
        else if (rd_result_en && (rd_result_id == rd_id_buff_r[i])) begin
            rd_data_buff_r[i][(rd_data_cnt_r[i]*`AXI_DATA_W) +: `AXI_DATA_W] <= #DLY axi_mst_rdata;
        end
    end
end
endgenerate

//--------------------------------------------------------------------------------
// Request Completion Counter for sim done
//--------------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_req_cnt_r  <= #DLY {REQ_CNT_W{1'b0}};
    end
    else if (rd_result_en & rd_result_last) begin
        rd_req_cnt_r <= #DLY rd_req_cnt_r + 1;
    end
end

//--------------------------------------------------------------------------------
// Output Signal
//--------------------------------------------------------------------------------
assign done = (rd_req_cnt_r == {REQ_CNT_W{1'b1}});  // All requests completed

always @(*) begin : GEN_REQ_VEC
    integer i;
    rd_req_bits = {OST_DEPTH{1'b0}};
    for (i=0; i<OST_DEPTH; i=i+1) begin
        rd_req_bits[i] = rd_req_buff_r[i];
    end
end

assign axi_mst_arvalid = |rd_req_bits;
assign axi_mst_arid    = rd_id_buff_r    [rd_req_ptr_r];
assign axi_mst_araddr  = rd_addr_buff_r  [rd_req_ptr_r];
assign axi_mst_arlen   = rd_len_buff_r   [rd_req_ptr_r];
assign axi_mst_arsize  = rd_size_buff_r  [rd_req_ptr_r];
assign axi_mst_arburst = rd_burst_buff_r [rd_req_ptr_r];

assign axi_mst_rready  = 1'b1;  

endmodule
