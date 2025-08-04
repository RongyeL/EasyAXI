// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2022 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : tb_rvseed.v
// Author        : Rongye
// Created On    : 2022-03-25 04:18
// Last Modified : 2025-08-04 09:29
// ---------------------------------------------------------------------------------
// Description   : 
//
//
// -FHDR----------------------------------------------------------------------------
`timescale 1ns / 1ps

module TESTBENCH ();

reg                  clk;
reg                  rst_n;
reg                  enable;
wire                 done;


localparam SIM_PERIOD = 20; // 20ns -> 50MHz

integer k;
initial begin
    #(SIM_PERIOD/2);
    clk = 1'b0;
    reset;
    // $finish;
end

initial begin
    #(SIM_PERIOD * 10000);
    $display("Time Out");
    $finish;
end

always #(SIM_PERIOD/2) clk = ~clk;

task reset;                // reset 1 clock
    begin
        enable = 0; 
        rst_n = 0; 
        #(SIM_PERIOD * 1);
        rst_n = 1;
        #(SIM_PERIOD * 5 + 1);
        enable = 1; 
    end
endtask

always begin
   wait (done == 1) begin
       enable = 0; 
       #(SIM_PERIOD * 100 + 1);
       $finish;
   end
end
EASYAXI_TOP U_EASYAXI_TOP (
    .clk                            ( clk                           ),
    .rst_n                          ( rst_n                         ),
    .enable                         ( enable                        ),
    .done                           ( done                          )
);

// vcs 
initial begin
    $fsdbDumpfile("sim_out.fsdb");
    $fsdbDumpvars("+all");
end

endmodule
