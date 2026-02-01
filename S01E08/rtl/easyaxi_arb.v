// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_arb.v
// Author        : Rongye
// Created On    : 2025-08-04 08:29
// Last Modified : 2026-02-01 06:37
// ---------------------------------------------------------------------------------
// Description   : This module implements a round-robin arbiter.
//   - Supports configurable depth via DEEP_NUM parameter
//   - Round-robin scheduling with fairness guarantee
//   - Combinational grant generation with one-hot to index conversion
//   - Handles empty request queue gracefully
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_ARB #(
    parameter DEEP_NUM = 8
)(
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire [DEEP_NUM        -1:0] queue_i,
    input  wire                        sche_en,
    output wire [$clog2(DEEP_NUM)-1:0] pointer_o
);

reg  [DEEP_NUM-1:0] req_power;
wire [DEEP_NUM-1:0] req_after_power = queue_i & req_power;

wire [DEEP_NUM-1:0] old_mask = {req_after_power[DEEP_NUM-2:0] | old_mask[DEEP_NUM-2:0], 1'b0};
wire [DEEP_NUM-1:0] new_mask = {queue_i[DEEP_NUM-2:0] | new_mask[DEEP_NUM-2:0], 1'b0};

wire old_grant_work = (|req_after_power);

wire [DEEP_NUM-1:0] old_grant = ~old_mask & req_after_power;
wire [DEEP_NUM-1:0] new_grant = ~new_mask & queue_i;
wire [DEEP_NUM-1:0] grant = old_grant_work ? old_grant : new_grant;

function automatic [$clog2(DEEP_NUM)-1:0] onehot_to_index;
    input [DEEP_NUM-1:0] onehot;
    integer i;
    begin
        onehot_to_index = {$clog2(DEEP_NUM){1'b0}};
        for (i = 0; i < DEEP_NUM; i = i + 1) begin
            if (onehot[i]) begin
                onehot_to_index = i;
            end
        end
    end
endfunction

assign pointer_o = (|queue_i) ? onehot_to_index(grant) : {$clog2(DEEP_NUM){1'b0}};

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        req_power <= {DEEP_NUM{1'b1}};
    end
    else if (sche_en) begin
        if (old_grant_work) begin
            req_power <= old_mask;
        end
        else if (|queue_i) begin
            req_power <= new_mask;
        end
    end
end

endmodule
