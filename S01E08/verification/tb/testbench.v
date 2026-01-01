// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2022 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : tb_rvseed.v
// Author        : Rongye
// Created On    : 2022-03-25 04:18
// Last Modified : 2026-01-01 00:56
// ---------------------------------------------------------------------------------
// Description   : 
//
//
// -FHDR----------------------------------------------------------------------------
`timescale 1ns / 1ps

module TESTBENCH ();

reg                  clk;
reg                  rst_n;
reg                  rd_en;
wire                 rd_done;
reg                  wr_en;
wire                 wr_done;


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
        rd_en = 0; 
        wr_en = 0; 
        rst_n = 0; 
        #(SIM_PERIOD * 1);
        rst_n = 1;
        #(SIM_PERIOD * 5 + 1);
        rd_en = 1; 
        #(SIM_PERIOD * 5 + 1);
        wr_en = 1; 
    end
endtask

always begin
   wait (rd_done == 1) begin
       #(SIM_PERIOD * 3);
       rd_en = 0; 
       #(SIM_PERIOD * 150 + 1);
       // $finish;
   end
   wait (wr_done == 1) begin
       #(SIM_PERIOD * 3);
       wr_en = 0; 
       #(SIM_PERIOD * 150 + 1);
       $finish;
   end
end
EASYAXI_TOP U_EASYAXI_TOP (
    .clk                            ( clk                           ),
    .rst_n                          ( rst_n                         ),
    .rd_en                          ( rd_en                         ),
    .rd_done                        ( rd_done                       ),
    .wr_en                          ( wr_en                         ),
    .wr_done                        ( wr_done                       )
);

// vcs 
initial begin
    $fsdbDumpfile("sim_out.fsdb");
    $fsdbDumpvars("+all");
end

endmodule
