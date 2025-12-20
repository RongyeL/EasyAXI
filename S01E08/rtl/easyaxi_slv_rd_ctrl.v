// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_slv.v
// Author        : Rongye
// Created On    : 2025-02-06 06:52
// Last Modified : 2025-12-20 07:48
// ---------------------------------------------------------------------------------
// Description   : AXI Slave with burst support up to length 8 and outstanding capability
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_SLV_RD_CTRL #(
    parameter OST_DEPTH = 16  // Outstanding depth, must be power of 2
)(
// Global
    input  wire                      clk,
    input  wire                      rst_n, 

// AXI AR Channel
    input  wire                      axi_slv_arvalid,
    output wire                      axi_slv_arready,
    input  wire  [`AXI_ID_W    -1:0] axi_slv_arid,
    input  wire  [`AXI_ADDR_W  -1:0] axi_slv_araddr,
    input  wire  [`AXI_LEN_W   -1:0] axi_slv_arlen,
    input  wire  [`AXI_SIZE_W  -1:0] axi_slv_arsize,
    input  wire  [`AXI_BURST_W -1:0] axi_slv_arburst,
    input  wire  [`AXI_USER_W  -1:0] axi_slv_aruser,

// AXI R Channel
    output wire                      axi_slv_rvalid,
    input  wire                      axi_slv_rready,
    output wire  [`AXI_ID_W    -1:0] axi_slv_rid,
    output wire  [`AXI_DATA_W  -1:0] axi_slv_rdata,
    output wire  [`AXI_RESP_W  -1:0] axi_slv_rresp,
    output wire                      axi_slv_rlast,
    output wire  [`AXI_USER_W  -1:0] axi_slv_ruser
);

localparam DLY           = 0.1;
localparam MAX_BURST_LEN = 8;  // Maximum burst length support
localparam BURST_CNT_W   = $clog2(MAX_BURST_LEN);  // Maximum burst length cnt width
localparam REG_ADDR      = 16'h0000;  // Default register address
localparam OST_CNT_W     = OST_DEPTH == 1 ? 1 : $clog2(OST_DEPTH);      // Outstanding counter width
localparam MAX_GET_DATA_DLY = `AXI_DATA_GET_CNT_W'h1C;      // Outstanding counter width

//--------------------------------------------------------------------------------
// Inner Signal 
//--------------------------------------------------------------------------------
wire                     rd_buff_set;         
wire                     rd_buff_clr;         
wire                     rd_buff_full;         

// Outstanding request status buffers
reg                      rd_valid_buff_r [OST_DEPTH-1:0];
reg                      rd_result_buff_r[OST_DEPTH-1:0];
reg                      rd_comp_buff_r  [OST_DEPTH-1:0];
reg                      rd_clear_buff_r [OST_DEPTH-1:0];

// Bit-vector representations for status flags
reg  [OST_DEPTH    -1:0] rd_valid_bits;
wire [OST_DEPTH    -1:0] rd_set_bits;
reg  [OST_DEPTH    -1:0] rd_result_bits;
reg  [OST_DEPTH    -1:0] rd_clear_bits;
wire [OST_DEPTH    -1:0] rd_order_bits;


// Buffer management pointers
wire [OST_CNT_W    -1:0] rd_set_ptr_r;
wire [OST_CNT_W    -1:0] rd_clr_ptr_r;
wire [OST_CNT_W    -1:0] rd_result_ptr_r;
wire [OST_CNT_W    -1:0] rd_result_order_r;

// Outstanding transaction payload buffers
reg  [`AXI_LEN_W   -1:0] rd_curr_index_r [OST_DEPTH-1:0];
reg  [`AXI_ID_W    -1:0] rd_id_buff_r    [OST_DEPTH-1:0];
reg  [`AXI_ADDR_W  -1:0] rd_addr_buff_r  [OST_DEPTH-1:0];
reg  [`AXI_LEN_W   -1:0] rd_len_buff_r   [OST_DEPTH-1:0];
reg  [`AXI_SIZE_W  -1:0] rd_size_buff_r  [OST_DEPTH-1:0];
reg  [`AXI_BURST_W -1:0] rd_burst_buff_r [OST_DEPTH-1:0];
reg  [`AXI_USER_W  -1:0] rd_user_buff_r  [OST_DEPTH-1:0];

// Read data buffers (supports MAX_BURST_LEN beats per OST entry)    
reg  [MAX_BURST_LEN             -1:0] rd_data_vld_r  [OST_DEPTH-1:0];
reg  [MAX_BURST_LEN*`AXI_DATA_W -1:0] rd_data_buff_r [OST_DEPTH-1:0];
reg  [BURST_CNT_W               -1:0] rd_data_cnt_r  [OST_DEPTH-1:0]; // Counter for burst data
reg  [`AXI_RESP_W               -1:0] rd_resp_buff_r [OST_DEPTH-1:0];
wire [OST_DEPTH                 -1:0] rd_resp_err;                    // Error flags


reg  [`AXI_ADDR_W  -1:0] rd_curr_addr_r  [OST_DEPTH-1:0];     // Current address
reg                      rd_wrap_en_r    [OST_DEPTH-1:0];     // Wrap happen Tag

wire                     rd_dec_miss;         
wire                     rd_result_en;        
wire [`AXI_ID_W    -1:0] rd_result_id;        
wire                     rd_result_last;      

reg  [`AXI_DATA_GET_CNT_W-1:0] rd_data_get_cnt   [OST_DEPTH-1:0];
wire                           rd_data_get_cnt_en[OST_DEPTH-1:0];         
wire                           rd_data_get       [OST_DEPTH-1:0];         
wire                           rd_data_err       [OST_DEPTH-1:0];         

// Burst address calculation
wire [`AXI_ADDR_W    -1:0] rd_start_addr    [OST_DEPTH-1:0]; // Start address based on axi_addr
wire [`AXI_LEN_W     -1:0] rd_burst_lenth   [OST_DEPTH-1:0]; // Burst_length
wire [2**`AXI_SIZE_W -1:0] rd_number_bytes  [OST_DEPTH-1:0]; // Number of bytes
wire [`AXI_ADDR_W    -1:0] rd_wrap_boundary [OST_DEPTH-1:0]; // Wrap boundary address
wire [`AXI_ADDR_W    -1:0] rd_aligned_addr  [OST_DEPTH-1:0]; // Aligned address

wire                       rd_wrap_en       [OST_DEPTH-1:0]; // Wrap happen

//--------------------------------------------------------------------------------
// Pointer Management
//--------------------------------------------------------------------------------
EASYAXI_ARB #(
    .DEEP_NUM(OST_DEPTH)
) U_RD_SET_ARB (
    .clk      (clk           ),
    .rst_n    (rst_n         ),
    .queue_i  (rd_set_bits   ),
    .sche_en  (rd_buff_set   ),
    .pointer_o(rd_set_ptr_r  )
);

EASYAXI_ARB #(
    .DEEP_NUM(OST_DEPTH)
) U_RD_CLEAR_ARB (
    .clk      (clk           ),
    .rst_n    (rst_n         ),
    .queue_i  (rd_clear_bits ),
    .sche_en  (rd_buff_clr   ),
    .pointer_o(rd_clr_ptr_r  )
);

EASYAXI_ARB #(
    .DEEP_NUM(OST_DEPTH)
) U_RD_RESULT_ARB (
    .clk      (clk            ),
    .rst_n    (rst_n          ),
    .queue_i  (rd_result_bits & rd_order_bits),
    .sche_en  (rd_result_en   ),
    .pointer_o(rd_result_ptr_r)
);

//--------------------------------------------------------------------------------
// Main Ctrl
//--------------------------------------------------------------------------------
assign rd_buff_set = axi_slv_arvalid & axi_slv_arready;
assign rd_buff_clr = rd_valid_buff_r[rd_clr_ptr_r] & ~rd_result_buff_r[rd_clr_ptr_r] & ~rd_comp_buff_r[rd_clr_ptr_r];

always @(*) begin : GEN_VLD_VEC
    integer i;
    rd_valid_bits = {OST_DEPTH{1'b0}};
    for (i=0; i<OST_DEPTH; i=i+1) begin
        rd_valid_bits[i] = rd_valid_buff_r[i];
    end
end
assign rd_buff_full = &rd_valid_bits;
assign rd_set_bits = ~rd_valid_bits;

always @(*) begin : GEN_RESULT_VEC
    integer i;
    rd_result_bits = {OST_DEPTH{1'b0}};
    for (i=0; i<OST_DEPTH; i=i+1) begin
        rd_result_bits[i] = rd_result_buff_r[i];
    end
end

always @(*) begin : GEN_CLEAR_VEC
    integer i;
    rd_clear_bits = {OST_DEPTH{1'b0}};
    for (i=0; i<OST_DEPTH; i=i+1) begin
        rd_clear_bits[i] = rd_clear_buff_r[i];
    end
end

assign rd_dec_miss    = 1'b0/*  (axi_slv_araddr != REG_ADDR) */;
assign rd_result_en   = axi_slv_rvalid & axi_slv_rready;
assign rd_result_id   = axi_slv_rid;
assign rd_result_last = axi_slv_rlast;

genvar i;
generate 
for (i=0; i<OST_DEPTH; i=i+1) begin: OST_BUFFERS
    // Valid buffer control
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_valid_buff_r[i] <= #DLY 1'b0;
        end
        else begin
            if (rd_buff_set && (rd_set_ptr_r == i)) begin
                rd_valid_buff_r[i] <= #DLY 1'b1;
            end
            if (rd_buff_clr && (rd_clr_ptr_r == i)) begin
                rd_valid_buff_r[i] <= #DLY 1'b0;
            end
        end
    end

    // Result sent flag buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_result_buff_r[i] <= #DLY 1'b0;
        end
        else begin
            if (rd_buff_set && (rd_set_ptr_r == i)) begin
                rd_result_buff_r[i] <= #DLY rd_dec_miss ? 1'b1 : 1'b0;
            end
            else if (rd_result_en && ~rd_result_last && (rd_result_ptr_r == i) && (rd_data_vld_r[i][(rd_data_cnt_r[i]+1)])) begin
                rd_result_buff_r[i] <= #DLY 1'b1;
            end 
            else if (rd_data_get[i]) begin
                rd_result_buff_r[i] <= #DLY 1'b1;
            end
            else if (rd_result_en && (rd_result_ptr_r == i)) begin
                rd_result_buff_r[i] <= #DLY 1'b0;
            end
        end
    end

    // Completion flag buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_comp_buff_r[i] <= #DLY 1'b0;
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_comp_buff_r[i] <= #DLY 1'b1;
        end
        else if (rd_result_en && rd_result_last && (rd_result_ptr_r == i)) begin
            rd_comp_buff_r[i] <= #DLY 1'b0;
        end
    end

    // Clear flag buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_clear_buff_r[i] <= #DLY 1'b0;
        end
        else begin
            rd_clear_buff_r[i] <= #DLY rd_valid_buff_r[i] & ~rd_result_buff_r[i] & ~rd_comp_buff_r[i];
        end
    end
end
endgenerate

//--------------------------------------------------------------------------------
// AXI AR Payload Buffer
//--------------------------------------------------------------------------------
generate 
for (i=0; i<OST_DEPTH; i=i+1) begin: AR_PAYLOAD
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_id_buff_r[i]    <= #DLY {`AXI_ID_W{1'b0}};
            rd_addr_buff_r[i]  <= #DLY {`AXI_ADDR_W{1'b0}};
            rd_len_buff_r[i]   <= #DLY {`AXI_LEN_W{1'b0}};
            rd_size_buff_r[i]  <= #DLY {`AXI_SIZE_W{1'b0}};
            rd_burst_buff_r[i] <= #DLY {`AXI_BURST_W{1'b0}};
            rd_user_buff_r[i]  <= #DLY {`AXI_USER_W{1'b0}};
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_id_buff_r[i]    <= #DLY axi_slv_arid;
            rd_addr_buff_r[i]  <= #DLY axi_slv_araddr;
            rd_len_buff_r[i]   <= #DLY axi_slv_arlen;
            rd_size_buff_r[i]  <= #DLY axi_slv_arsize;
            rd_burst_buff_r[i] <= #DLY axi_slv_arburst;
            rd_user_buff_r[i]  <= #DLY axi_slv_aruser;
        end
    end
end
endgenerate

//--------------------------------------------------------------------------------
// Burst Address Control
//--------------------------------------------------------------------------------
generate 
for (i=0; i<OST_DEPTH; i=i+1) begin: BURST_CTRL
    assign rd_start_addr   [i] = rd_addr_buff_r[i];
    assign rd_number_bytes [i] = 1 << rd_size_buff_r[i];
    assign rd_burst_lenth  [i] = rd_len_buff_r[i] + 1;
    assign rd_aligned_addr [i] = rd_start_addr[i] / rd_number_bytes[i] * rd_number_bytes[i];
    assign rd_wrap_boundary[i] = rd_start_addr[i] / (rd_burst_lenth[i] * rd_number_bytes[i]) * (rd_burst_lenth[i] * rd_number_bytes[i]);
    assign rd_wrap_en      [i] = (rd_curr_addr_r[i] + rd_number_bytes[i]) == (rd_wrap_boundary[i] + (rd_burst_lenth[i] * rd_number_bytes[i]));

    // Read index control
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_curr_index_r[i] <= #DLY {`AXI_LEN_W{1'b0}};
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_curr_index_r[i] <= #DLY rd_dec_miss ? rd_burst_lenth[i] : `AXI_LEN_W'h1;
        end
        else if (rd_result_en && rd_result_last && (rd_result_ptr_r == i)) begin
            rd_curr_index_r[i] <= #DLY {`AXI_LEN_W{1'b0}};
        end
        else if (rd_data_get[i]) begin
            rd_curr_index_r[i] <= #DLY rd_curr_index_r[i] + `AXI_LEN_W'h1;
        end
    end

    // Read wrap control
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_wrap_en_r[i] <= #DLY 1'b0;
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_wrap_en_r[i] <= #DLY 1'b0;
        end
        else if (rd_data_get[i]) begin
            rd_wrap_en_r[i] <= #DLY rd_wrap_en_r[i] | rd_wrap_en[i];
        end
    end

    // Current address control
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_curr_addr_r[i] <= #DLY {`AXI_ADDR_W{1'b0}};
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_curr_addr_r[i] <= #DLY axi_slv_araddr;
        end
        else if (rd_data_get[i]) begin
            case (rd_burst_buff_r[i])
                `AXI_BURST_FIXED: rd_curr_addr_r[i] <= #DLY axi_slv_araddr;
                `AXI_BURST_INCR : rd_curr_addr_r[i] <= #DLY rd_aligned_addr[i] + (rd_curr_index_r[i] * rd_number_bytes[i]);
                `AXI_BURST_WRAP : begin
                    if (rd_wrap_en[i])
                        rd_curr_addr_r[i] <= #DLY rd_wrap_boundary[i];
                    else if (rd_wrap_en_r[i])
                        rd_curr_addr_r[i] <= #DLY rd_addr_buff_r[i] + (rd_curr_index_r[i] * rd_number_bytes[i]) - (rd_number_bytes[i] * rd_burst_lenth[i]);
                    else
                        rd_curr_addr_r[i] <= #DLY rd_aligned_addr[i] + (rd_curr_index_r[i] * rd_number_bytes[i]);
                end
                default: rd_curr_addr_r[i] <= #DLY {`AXI_ADDR_W{1'b0}};
            endcase
        end
    end
end
endgenerate

//--------------------------------------------------------------------------------
// AXI R Payload Buffer
//--------------------------------------------------------------------------------
generate 
for (i=0; i<OST_DEPTH; i=i+1) begin: R_PAYLOAD
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_resp_buff_r[i] <= #DLY {`AXI_RESP_W{1'b0}};
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_resp_buff_r[i] <= #DLY rd_dec_miss ? `AXI_RESP_DECERR : `AXI_RESP_OKAY;
        end
        else if (rd_data_get[i]) begin
            rd_resp_buff_r[i] <= #DLY rd_data_err[i] ? `AXI_RESP_SLVERR : `AXI_RESP_OKAY;
        end
    end
    // Burst data beat counter
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_data_cnt_r[i]  <= #DLY {BURST_CNT_W{1'b0}};
        end

        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_data_cnt_r[i]  <= #DLY {BURST_CNT_W{1'b0}};
        end
        else if (rd_result_en && (rd_result_ptr_r == i)) begin
            rd_data_cnt_r[i] <= #DLY rd_data_cnt_r[i] + 1;
        end
    end
    // Burst data buffer
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_data_vld_r [i] <= #DLY {MAX_BURST_LEN{1'b0}};
            rd_data_buff_r[i] <= #DLY {(`AXI_DATA_W*MAX_BURST_LEN){1'b0}};
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_data_vld_r [i] <= #DLY {MAX_BURST_LEN{1'b0}};
            rd_data_buff_r[i] <= #DLY {(`AXI_DATA_W*MAX_BURST_LEN){1'b0}};
        end
        else if (rd_data_get[i]) begin
            rd_data_vld_r [i][(rd_curr_index_r[i]-1)] <= #DLY 1'b1;
            rd_data_buff_r[i][((rd_curr_index_r[i]-1)*`AXI_DATA_W) +: `AXI_DATA_W] <= #DLY {{`AXI_DATA_W-`AXI_ID_W-`AXI_ADDR_W{1'b0}},rd_id_buff_r[i],rd_curr_addr_r[i]};
        end
    end
end
endgenerate

//--------------------------------------------------------------------------------
// Simulate the data reading process
//--------------------------------------------------------------------------------
generate 
for (i=0; i<OST_DEPTH; i=i+1) begin: RD_DATA_PROC_SIM
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            rd_data_get_cnt[i] <= #DLY `AXI_DATA_GET_CNT_W'h0;
        end
        else if (rd_buff_set && (rd_set_ptr_r == i)) begin
            rd_data_get_cnt[i] <= #DLY `AXI_DATA_GET_CNT_W'h1;
        end
        else if (rd_data_get[i] && (rd_curr_index_r[i] < rd_burst_lenth[i])) begin
            rd_data_get_cnt[i] <= #DLY `AXI_DATA_GET_CNT_W'h1;
        end
        else if (rd_data_get_cnt[i]== MAX_GET_DATA_DLY) begin // rd data delay is 18 cycle
            rd_data_get_cnt[i] <= #DLY `AXI_DATA_GET_CNT_W'h0;
        end
        else if (rd_data_get_cnt[i]>`AXI_DATA_GET_CNT_W'h0) begin
            rd_data_get_cnt[i] <= #DLY rd_data_get_cnt[i] + `AXI_DATA_GET_CNT_W'h1;
        end
    end
    assign rd_data_get   [i] = rd_valid_buff_r[i] & (rd_data_get_cnt[i]==(MAX_GET_DATA_DLY - rd_id_buff_r[i]));
    assign rd_data_err   [i] = (rd_id_buff_r[i] == `AXI_ID_W'hF) & (rd_curr_index_r[i] == rd_burst_lenth[i]);
end
endgenerate

//--------------------------------------------------------------------------------
// RESP ID ORDER CTRL
//--------------------------------------------------------------------------------
EASYAXI_ORDER #(
    .OST_DEPTH(OST_DEPTH),
    .ID_WIDTH (`AXI_ID_W)
) U_EASYAXI_SLV_RD_ORDER (
    .clk        (clk             ),
    .rst_n      (rst_n           ),

    .req_valid  (axi_slv_arvalid ),
    .req_ready  (axi_slv_arready ),
    .req_id     (axi_slv_arid    ),
    .req_ptr    (rd_set_ptr_r    ),

    .resp_valid (axi_slv_rvalid  ),
    .resp_ready (axi_slv_rready  ),
    .resp_id    (axi_slv_rid     ),
    .resp_last  (axi_slv_rlast   ),

    .resp_ptr   (                ),
    .resp_bits  (rd_order_bits   )
);
//--------------------------------------------------------------------------------
// Output Signal
//--------------------------------------------------------------------------------
assign axi_slv_arready = ~rd_buff_full;
assign axi_slv_rvalid  = rd_valid_buff_r [rd_result_ptr_r] & rd_result_buff_r[rd_result_ptr_r];
assign axi_slv_rid     = rd_id_buff_r    [rd_result_ptr_r];
assign axi_slv_rdata   = rd_data_buff_r  [rd_result_ptr_r][((rd_data_cnt_r[rd_result_ptr_r])*`AXI_DATA_W) +: `AXI_DATA_W];
assign axi_slv_rresp   = rd_resp_buff_r  [rd_result_ptr_r];
assign axi_slv_rlast   = axi_slv_rvalid & (rd_data_cnt_r[rd_result_ptr_r] == rd_len_buff_r[rd_result_ptr_r]);
assign axi_slv_ruser   = rd_user_buff_r  [rd_result_ptr_r];

endmodule
