// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_arb.v
// Author        : Rongye
// Created On    : 2025-08-04 08:29
// Last Modified : 2025-08-04 08:39
// ---------------------------------------------------------------------------------
// Description   : 
//
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_ARB #(
    parameter DEEP_NUM = 8
)(
    input  wire                clk,        // System clock
    input  wire                rst_n,      // Asynchronous active-low reset
    input  wire [DEEP_NUM-1:0] queue_i,    // Request queue input, DEEP_NUM-bit wide (1=request pending)
    input  wire                sche_en,    // Scheduling enable signal
    output reg  [DEEP_NUM-1:0] pointer_o   // Grant output (one-hot encoded)
);

// Internal signals
reg  [DEEP_NUM-1:0] last_grant;    // Last granted request
reg  [DEEP_NUM-1:0] mask;          // Priority mask
wire [DEEP_NUM-1:0] masked_queue;  // Masked request queue
wire [DEEP_NUM-1:0] unmasked_grant;// Grant before priority consideration
wire [DEEP_NUM-1:0] grant;         // Final grant signal
wire no_masked_req;                // Flag when no requests in masked portion
reg  found;                        // Flag to indicate when last_grant bit is found

// Mask generation: sets bits higher than last grant to 1
always @(*) begin
integer i,j;
    mask = {DEEP_NUM{1'b0}};
    found = 1'b0;
    for (i = 0; i < DEEP_NUM; i = i + 1) begin
        if (!found && last_grant[i]) begin
            found = 1'b1;
        end
        if (found) begin
            for (j = 0; j < DEEP_NUM; j = j + 1) begin
                mask[j] = (j > i);
            end
        end
    end
end

// Generate masked request queue
assign masked_queue = queue_i & mask;

// Check if there are any requests in masked portion
assign no_masked_req = (masked_queue == {DEEP_NUM{1'b0}});

// Priority encoder: selects lowest set bit in masked queue
assign unmasked_grant = masked_queue & (-masked_queue);

// If no requests in masked portion, select from full queue
assign grant = no_masked_req ? (queue_i & (-queue_i)) : unmasked_grant;

// Output register update
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        pointer_o  <= {DEEP_NUM{1'b0}};
        last_grant <= {DEEP_NUM{1'b0}};
    end 
    else if (sche_en) begin
        // Only update when there are pending requests
        if (queue_i != {DEEP_NUM{1'b0}}) begin
            pointer_o  <= grant;
            last_grant <= grant;
        end 
        else begin
            pointer_o <= {DEEP_NUM{1'b0}};
            // Maintain last_grant when no requests
        end
    end
end

endmodule
