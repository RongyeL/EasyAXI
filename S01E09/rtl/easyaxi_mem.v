// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_mem.v
// Author        : Rongye
// Created On    : 2026-03-02 02:40
// Last Modified : 2026-03-03 22:39
// ---------------------------------------------------------------------------------
// Description   : Simple memory module to store write data from slave write controller
//   - Stores write data from slave write internal write/read interface
//   - Provides read data to slave read controller
//   - 16-word memory (256B with 128-bit data width)
//   - 128-bit internal write/read interface (16B per word)
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_MEM (
    input  wire                      clk,
    input  wire                      rst_n,
    
    // Write interface from slave write controller
    input  wire                      s_wr_data_en,
    input  wire  [`AXI_ID_W    -1:0] s_wr_data_id,
    input  wire  [`AXI_ADDR_W  -1:0] s_wr_data_addr,
    input  wire  [127:0]             s_wr_data_data,   // 128-bit data for 16B internal write/read interface
    input  wire  [15:0]              s_wr_data_strb,   // 16-byte strobe
    
    // Read interface to slave read controller
    input  wire                      s_rd_req_en,
    input  wire  [`AXI_ID_W    -1:0] s_rd_req_id,
    input  wire  [`AXI_ADDR_W  -1:0] s_rd_req_addr,
    
    output wire                      s_rd_resp_en,
    output wire  [`AXI_ID_W    -1:0] s_rd_resp_id,
    output wire  [127:0]             s_rd_resp_data,   // 128-bit data for 16B internal write/read interface
    output wire  [`AXI_RESP_W  -1:0] s_rd_resp_resp
);

localparam DLY = 0.1;
localparam MEM_DEPTH = 16;  // 16 words = 256B with 128-bit data (16 bytes per word)
localparam MEM_ADDR_W = $clog2(MEM_DEPTH);

// Memory array - 128-bit wide for 16B internal write/read interface
reg [127:0] mem_array [0:MEM_DEPTH-1];

// Address >> 4 for 16B aligned word address
wire [MEM_ADDR_W-1:0] mem_wr_addr = s_wr_data_addr >> 4;
wire [MEM_ADDR_W-1:0] mem_rd_addr = s_rd_req_addr >> 4;
// Byte offset within the 16B word (from address bits [3:0])
wire [3:0] wr_byte_offset = s_wr_data_addr[3:0];

// Write operation: 1B granularity with byte offset
always @(posedge clk or negedge rst_n) begin
    integer i;
    if (~rst_n) begin
        // Initialize memory to zeros
        for (i = 0; i < MEM_DEPTH; i = i + 1) begin
            mem_array[i] <= #DLY 128'h0;
        end
    end
    else if (s_wr_data_en) begin
        for (i = 0; i < 16; i = i + 1) begin
            if (s_wr_data_strb[i]) begin
                // Write byte i to memory at position (wr_byte_offset + i) within the 16B word
                mem_array[mem_wr_addr][((wr_byte_offset + i)*8)+:8] <= #DLY s_wr_data_data[(i*8)+:8];
            end
        end
    end
end

// Read operation
reg                      s_rd_resp_en_r;
reg  [`AXI_ID_W    -1:0] s_rd_resp_id_r;
reg  [127:0]             s_rd_resp_data_r;
reg  [`AXI_RESP_W  -1:0] s_rd_resp_resp_r;
always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            s_rd_resp_en_r   <= #DLY 1'b0;
            s_rd_resp_id_r   <= #DLY {`AXI_ID_W{1'b0}};
            s_rd_resp_data_r <= #DLY 128'h0;
            s_rd_resp_resp_r <= #DLY {`AXI_RESP_W{1'b0}};
        end
        else if (s_rd_req_en) begin
            s_rd_resp_en_r   <= #DLY 1'b1;
            s_rd_resp_id_r   <= #DLY s_rd_req_id;
            if (mem_rd_addr < MEM_DEPTH) begin
                s_rd_resp_data_r <= #DLY mem_array[mem_rd_addr];
                s_rd_resp_resp_r <= #DLY `AXI_RESP_OKAY;
            end
            else begin
                s_rd_resp_data_r <= #DLY 128'h0;
                s_rd_resp_resp_r <= #DLY `AXI_RESP_DECERR;  // Decode error for out-of-range address
            end
        end
        else begin
            s_rd_resp_en_r <= #DLY 1'b0;
        end
    end

assign s_rd_resp_en   = s_rd_resp_en_r;
assign s_rd_resp_id   = s_rd_resp_id_r;
assign s_rd_resp_data = s_rd_resp_data_r;
assign s_rd_resp_resp = s_rd_resp_resp_r;

endmodule
