// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_fifo.v
// Author        : Rongye
// Created On    : 2025-08-25 07:53
// Last Modified : 2025-09-06 02:08
// ---------------------------------------------------------------------------------
// Description   : 
//
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_FIFO #(
    parameter DATA_WIDTH = 4,
    parameter DEPTH = 16
)(
    input  wire                  clk,
    input  wire                  rst_n,

    input  wire                  wr,
    input  wire                  rd,
    input  wire [DATA_WIDTH-1:0] data_in,
    output wire [DATA_WIDTH-1:0] data_out,
    output wire                  empty,
    output wire                  full
);

localparam DLY = 0.1;
localparam PTR_WIDTH = $clog2(DEPTH);

reg [DATA_WIDTH-1:0] fifo[DEPTH-1:0];
reg [PTR_WIDTH -1:0] wr_ptr, rd_ptr;
reg                  wr_wrap, rd_wrap;

wire ptr_equal  = (wr_ptr == rd_ptr);
wire wrap_equal = (wr_wrap == rd_wrap);

assign empty = ptr_equal && wrap_equal;
assign full  = ptr_equal && ~wrap_equal;

assign data_out = fifo[rd_ptr];

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        wr_ptr  <= #DLY 0;
        wr_wrap <= #DLY 0;
    end 
    else if (wr) begin
        if (wr_ptr == DEPTH - 1) begin
            wr_ptr  <= #DLY 0;
            wr_wrap <= #DLY ~wr_wrap;
        end 
        else begin
            wr_ptr <= #DLY wr_ptr + 1;
        end
    end
end
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        rd_ptr  <= #DLY 0;
        rd_wrap <= #DLY 0;
    end 
    else if (rd) begin
        if (rd_ptr == DEPTH - 1) begin
            rd_ptr  <= #DLY 0;
            rd_wrap <= #DLY ~rd_wrap;
        end 
        else begin
            rd_ptr <= #DLY rd_ptr + 1;
        end
    end
end
always @(posedge clk) begin 
    if (wr) begin 
        fifo[rd_ptr] <= #DLY data_in; 
    end 
end
endmodule

