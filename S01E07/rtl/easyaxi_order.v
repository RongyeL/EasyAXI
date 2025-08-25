// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_order.v
// Author        : Rongye
// Created On    : 2025-08-25 07:35
// Last Modified : 2025-08-25 08:13
// ---------------------------------------------------------------------------------
// Description   : 
//
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_ORDER #(
    parameter OST_DEPTH = 16,
    parameter ID_WIDTH  = 4
)(
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire                      req_valid,
    input  wire                      req_ready,
    input  wire [ID_WIDTH-1:0]       req_id,
    input  wire [$clog2(OST_DEPTH)-1:0] req_ptr,
    input  wire                      resp_valid,
    input  wire                      resp_ready,
    input  wire [ID_WIDTH-1:0]       resp_id,
    input  wire                      resp_last,
    output wire [$clog2(OST_DEPTH)-1:0] resp_ptr
);

localparam ID_NUM = 2**ID_WIDTH;
localparam PTR_WIDTH = $clog2(OST_DEPTH);

// FIFO接口信号
wire [PTR_WIDTH-1:0] fifo_data_out [ID_NUM-1:0];
wire                 fifo_empty    [ID_NUM-1:0];
wire                 fifo_full     [ID_NUM-1:0];
reg                  fifo_push     [ID_NUM-1:0];
reg                  fifo_pop      [ID_NUM-1:0];
reg  [PTR_WIDTH-1:0] fifo_data_in  [ID_NUM-1:0];

// 例化所有 FIFO
genvar i;
generate
    for (i = 0; i < ID_NUM; i = i + 1) begin : GEN_ORDER_FIFO
        EASYAXI_FIFO #(
            .DATA_WIDTH(PTR_WIDTH),
            .DEPTH     (OST_DEPTH)
        ) U_EASYAXI_FIFO (
            .clk       (clk             ),
            .rst_n     (rst_n           ),
            .push      (fifo_push    [i]),
            .pop       (fifo_pop     [i]),
            .data_in   (fifo_data_in [i]),
            .data_out  (fifo_data_out[i]),
            .empty     (fifo_empty   [i]),
            .full      (fifo_full    [i])
        );
    end
endgenerate

// 控制逻辑
always @(*) begin
integer j;
for (j=0; j<ID_NUM; j=j+1) begin
    fifo_push[j]    = 1'b0;
    fifo_pop[j]     = 1'b0;
    fifo_data_in[j] = {PTR_WIDTH{1'b0}};
    if (req_valid && req_ready) begin
        fifo_push   [req_id] = 1'b1;
        fifo_data_in[req_id] = req_ptr;
    end
    if (resp_valid && resp_ready && ~fifo_empty[resp_id]) begin
        if (resp_last) begin
            fifo_pop[resp_id] = 1'b1;
        end
    end
end
end

assign resp_ptr   = fifo_data_out[resp_id];

endmodule

