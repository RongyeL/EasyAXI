// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_fifo.v
// Author        : Rongye
// Created On    : 2025-08-25 07:53
// Last Modified : 2025-09-06 01:59
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
reg [PTR_WIDTH -1:0] wr_ptr,rd_ptr;
reg [PTR_WIDTH   :0] count;

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        wr_ptr <= #DLY 0;
        rd_ptr <= #DLY 0;
        count  <= #DLY 0;
    end 
    else begin
        if (wr && ~full) begin
            rd_ptr <= #DLY rd_ptr + 1;
            count  <= #DLY count + 1;
        end
        if (rd && ~empty) begin
            wr_ptr <= #DLY wr_ptr + 1;
            count  <= #DLY count - 1;
        end
    end
end
always @(posedge clk) begin
    if (wr && !full) begin
        fifo[rd_ptr] <= #DLY data_in;
    end
end

assign empty    = (count == 0);
assign full     = (count == DEPTH);
assign data_out = fifo[wr_ptr];

endmodule

