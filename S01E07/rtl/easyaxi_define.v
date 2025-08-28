// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2022 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_define.v
// Author        : Rongye
// Created On    : 2022-12-27 21:06
// Last Modified : 2025-08-28 08:39
// ---------------------------------------------------------------------------------
// Description   : 
//
//
// -FHDR----------------------------------------------------------------------------

`define AXI_ID_W           4
`define AXI_ADDR_W         16
`define AXI_DATA_W         32
`define AXI_LEN_W          8
`define AXI_SIZE_W         3
`define AXI_BURST_W        2
`define AXI_LOCK_W         1
`define AXI_CACHE_W        4
`define AXI_PROT_W         3
`define AXI_QOS_W          4
`define AXI_REGION_W       4
`define AXI_RESP_W         2
`define AXI_USER_W         5

`define AXI_SIZE_1B        3'b000
`define AXI_SIZE_2B        3'b001
`define AXI_SIZE_4B        3'b010
`define AXI_SIZE_8B        3'b011
`define AXI_SIZE_16B       3'b100
`define AXI_SIZE_32B       3'b101
`define AXI_SIZE_64B       3'b110
`define AXI_SIZE_128B      3'b111

`define AXI_BURST_FIXED    2'b00
`define AXI_BURST_INCR     2'b01
`define AXI_BURST_WRAP     2'b10
`define AXI_BURST_RSV      2'b11

`define AXI_RESP_OKAY      2'b00
`define AXI_RESP_EXOK      2'b01
`define AXI_RESP_SLVERR    2'b10
`define AXI_RESP_DECERR    2'b11

`define AXI_DATA_GET_CNT_W 5
