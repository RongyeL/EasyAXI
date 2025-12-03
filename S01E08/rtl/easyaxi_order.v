// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_order.v
// Author        : Rongye
// Created On    : 2025-08-25 07:35
// Last Modified : 2025-09-06 02:03
// ---------------------------------------------------------------------------------
// Description   : 
//   Tracks outstanding AXI transactions using per‑ID FIFOs.
//   wr on request handshake, rd on last‑beat response.
// -FHDR----------------------------------------------------------------------------
module EASYAXI_ORDER #(
    parameter OST_DEPTH = 16, 
    parameter ID_WIDTH  = 4
)(
    input  wire                         clk,
    input  wire                         rst_n,
    
    input  wire                         req_valid,
    input  wire                         req_ready,
    input  wire [ID_WIDTH-1:0]          req_id,
    input  wire [$clog2(OST_DEPTH)-1:0] req_ptr,

    input  wire                         resp_valid,
    input  wire                         resp_ready,
    input  wire [ID_WIDTH-1:0]          resp_id,
    input  wire                         resp_last,

    output wire [$clog2(OST_DEPTH)-1:0] resp_ptr,
    output reg  [OST_DEPTH-1:0]         resp_bits
);

localparam ID_NUM    = 2**ID_WIDTH;
localparam PTR_WIDTH = $clog2(OST_DEPTH);

wire [PTR_WIDTH-1:0] fifo_data_out [ID_NUM-1:0];
wire                 fifo_empty    [ID_NUM-1:0];
wire                 fifo_full     [ID_NUM-1:0];
reg                  fifo_wr       [ID_NUM-1:0];
reg                  fifo_rd       [ID_NUM-1:0];
reg  [PTR_WIDTH-1:0] fifo_data_in  [ID_NUM-1:0];

wire [OST_DEPTH-1:0] fifo_bitmap   [ID_NUM-1:0];

// Instantiate FIFO per ID
genvar i;
generate
    for (i = 0; i < ID_NUM; i = i + 1) begin : GEN_ORDER_FIFO
        EASYAXI_FIFO #(
            .DATA_WIDTH(PTR_WIDTH),
            .DEPTH     (OST_DEPTH)
        ) U_EASYAXI_FIFO (
            .clk       (clk             ),
            .rst_n     (rst_n           ),
            .wr        (fifo_wr      [i]),
            .rd        (fifo_rd      [i]),
            .data_in   (fifo_data_in [i]),
            .data_out  (fifo_data_out[i]),
            .empty     (fifo_empty   [i]),
            .full      (fifo_full    [i])
        );
    end
endgenerate

// wr on request, rd on last response
always @(*) begin
    integer j;
    for (j = 0; j < ID_NUM; j = j + 1) begin
        fifo_wr[j]      = 1'b0;
        fifo_rd[j]      = 1'b0;
        fifo_data_in[j] = {PTR_WIDTH{1'b0}};
    end
    if (req_valid && req_ready) begin
        fifo_wr[req_id]      = 1'b1;
        fifo_data_in[req_id] = req_ptr;
    end
    if (resp_valid && resp_ready && ~fifo_empty[resp_id] && resp_last) begin
        fifo_rd[resp_id] = 1'b1;
    end
end

assign resp_ptr = fifo_data_out[resp_id];

// Generate one-hot bitmap per FIFO
generate
    for (i = 0; i < ID_NUM; i = i + 1) begin : GEN_FIFO_BITMAP
        assign fifo_bitmap[i] = fifo_empty[i] ? {OST_DEPTH{1'b0}} 
                                              : (1 << fifo_data_out[i]);
    end
endgenerate

// OR all bitmaps into one combined output
always @(*) begin
integer k;
    resp_bits = {OST_DEPTH{1'b0}};
    for (k=0; k<ID_NUM; k=k+1) begin
        resp_bits = resp_bits | fifo_bitmap[k];
    end
end


endmodule

