// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2022 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easy_axi_para.v
// Author        : Rongye
// Created On    : 2022-12-27 21:06
// Last Modified : 2025-02-08 23:53
// ---------------------------------------------------------------------------------
// Description   : 
//
//
// -FHDR----------------------------------------------------------------------------

`define AXI_ID_WIDTH           4
`define AXI_ADDR_WIDTH         16
`define AXI_DATA_WIDTH         32
`define AXI_LEN_WIDTH          8
`define AXI_SIZE_WIDTH         3
`define AXI_BURST_WIDTH        2
`define AXI_LOCK_WIDTH         1
`define AXI_CACHE_WIDTH        4
`define AXI_PROT_WIDTH         3
`define AXI_QOS_WIDTH          4
`define AXI_REGION_WIDTH       4
`define AXI_RESP_WIDTH         2


`define AXI_BURST_FIXED        `AXI_BURST_WIDTH'b00
`define AXI_BURST_INCR         `AXI_BURST_WIDTH'b01
`define AXI_BURST_WRAP         `AXI_BURST_WIDTH'b10
`define AXI_BURST_RSV          `AXI_BURST_WIDTH'b11

`define AXI_SIZE_1_BYTE        `AXI_SIZE_WIDTH'b000
`define AXI_SIZE_2_BYTE        `AXI_SIZE_WIDTH'b001
`define AXI_SIZE_4_BYTE        `AXI_SIZE_WIDTH'b010
`define AXI_SIZE_8_BYTE        `AXI_SIZE_WIDTH'b011
`define AXI_SIZE_16_BYTE       `AXI_SIZE_WIDTH'b100
`define AXI_SIZE_32_BYTE       `AXI_SIZE_WIDTH'b101
`define AXI_SIZE_64_BYTE       `AXI_SIZE_WIDTH'b110
`define AXI_SIZE_128_BYTE      `AXI_SIZE_WIDTH'b111

