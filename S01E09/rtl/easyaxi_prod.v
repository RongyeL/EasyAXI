// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_prod.v
// Author        : Rongye
// Created On    : 2026-05-05
// Last Modified : 2026-05-08
// ---------------------------------------------------------------------------------
// Description   : CQE Producer module.
//   Writes 16 data blocks (idx 0~15) to SRAM via AXI, then writes CQE for each.
//   - Data Block[idx] at address 0x40 + idx*0x10, burst length = idx%4 (1~4 words)
//   - CQE[idx] at address 0x00 + idx, 1 byte: {LENGTH[3:0], 3'b000, 1'b1}
//   - AXI ID = idx
//   - Each word = {idx[3:0], word_cnt[7:0], 16'h0000, idx[3:0]}
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_PROD #(
    parameter OST_DEPTH     = 16,            // Outstanding transaction depth (power of 2)
    parameter NUM_BLOCKS    = 15,            // Number of blocks to write (0~NUM_BLOCKS-1)
    parameter BURST_TYPE    = `AXI_BURST_INCR, // Burst type: FIXED/INCR/WRAP
    parameter PROD_GAP_EN   = 1,             // 1=enable production gap, 0=rapid back-to-back
    parameter SIZE_MODE     = 0,             // 0=varying size per idx, 1=fixed size from FIXED_SIZE
    parameter [7:0] FIXED_SIZE = 8'd16       // Data block size in bytes when SIZE_MODE=1
)(
    input  wire                      clk,
    input  wire                      rst_n,

    // Control interface
    input  wire                      start,       // Trigger to start write transactions
    output wire                      done,        // All write transactions completed

// AXI AW Channel
    output wire                      axi_mst_awvalid,
    input  wire                      axi_mst_awready,
    output wire  [`AXI_ID_W    -1:0] axi_mst_awid,
    output wire  [`AXI_ADDR_W  -1:0] axi_mst_awaddr,
    output wire  [`AXI_LEN_W   -1:0] axi_mst_awlen,
    output wire  [`AXI_SIZE_W  -1:0] axi_mst_awsize,
    output wire  [`AXI_BURST_W -1:0] axi_mst_awburst,
    output wire  [`AXI_USER_W  -1:0] axi_mst_awuser,

// AXI W Channel
    output wire                      axi_mst_wvalid,
    input  wire                      axi_mst_wready,
    output wire  [`AXI_DATA_W  -1:0] axi_mst_wdata,
    output wire  [`AXI_DATA_W/8-1:0] axi_mst_wstrb,
    output wire                      axi_mst_wlast,
    output wire  [`AXI_USER_W  -1:0] axi_mst_wuser,

// AXI B Channel
    input  wire                      axi_mst_bvalid,
    output wire                      axi_mst_bready,
    input  wire  [`AXI_ID_W    -1:0] axi_mst_bid,
    input  wire  [`AXI_RESP_W  -1:0] axi_mst_bresp,
    input  wire  [`AXI_USER_W  -1:0] axi_mst_buser
);

localparam DLY = 0.1;

// State machine: one Data Block + one CQE per idx, then gap before next idx
// -------------------------------------------------------
// State flow diagram:
//
//                  +----> WRITE_DATA --+      +--> WRITE_CQE --+
//                  |        |          |      |      |          |
//   start           |   AW+W handshake |     |    AW+W handshake |
//   +---+           |        v         |     |      v          |
//   |   v           |   WAIT_BRESP     |     |   WAIT_CQE      |
//  IDLE ------------+   (wait B resp)   |     |   (wait B resp)   |
//   ^                          |        v     |      |          v
//   |                     m_wr_resp_en   |     | m_wr_resp_en   |
//   |                          +------> NEXT --+      +------> NEXT
//   |                          |                                |
//   |                     idx == 15                        idx < 15
//   |                          |                                |
//   |                          v                                v
//   +----------------------- DONE                         PROD_GAP
//     (return to IDLE when start=0)                  (wait 16+idx*3 cycles)
// -------------------------------------------------------
localparam IDLE        = 4'd0;
localparam WRITE_DATA  = 4'd1;  // Write Data Block: issue write request to MST_WR_CTRL via m_wr_req_*
localparam WAIT_BRESP  = 4'd2;  // Wait for m_wr_resp_en from MST_WR_CTRL (B response complete)
localparam WRITE_CQE   = 4'd3;  // Write CQE notification: 1 byte at address 0x00+idx
localparam WAIT_CQE    = 4'd4;  // Wait for B response after CQE write
localparam NEXT        = 4'd5;  // Check if all 16 blocks are done
localparam DONE        = 4'd6;  // All complete, assert done
localparam PROD_GAP    = 4'd7;  // Production gap: simulate Producer pacing in real scenarios

reg [3:0] state_r, state_n;
reg [3:0] idx_r;      // Current block index (0-15)
reg [7:0] gap_cnt_r;  // PROD_GAP interval counter

// PROD_GAP interval: 20 + idx*18, range 20~74 for idx 0~3
// Creates CQE MISS at idx=2 and idx=3 where Producer lags behind Consumer's poll
wire [7:0] prod_interval;
assign prod_interval = PROD_GAP_EN ? 8'd20 + idx_r * 8'd18 : 8'd1;

// Request generation signals - simplified upper-layer interface
// Producer only needs these 8 signals without direct AXI AW/W/B handling
reg                      m_wr_req_en_r;
reg  [`AXI_ID_W    -1:0] m_wr_req_id_r;
reg  [`AXI_ADDR_W  -1:0] m_wr_req_addr_r;
reg  [7:0]               m_wr_req_size_r;   // Byte count: MST_WR_CTRL converts to AXI size/len
reg  [1:0]               m_wr_req_burst_r;  // Fixed INCR
reg  [127:0]             m_wr_req_data_r;   // Present all data at once; controller splits into AXI beats
reg  [15:0]              m_wr_req_strb_r;   // Byte strobe

// Response signals -- pulse on transaction completion
wire                     m_wr_resp_en;      // Pulse: write transaction complete
wire [`AXI_ID_W    -1:0] m_wr_resp_id;      // Completed transaction ID
wire [`AXI_RESP_W  -1:0] m_wr_resp_resp;    // B response (OKAY/SLVERR/DECERR)

wire cqe_written;
assign cqe_written = (state_r == WAIT_CQE) && m_wr_resp_en;

reg                      done_r;

// Generate data for a block: each word = {size_code[3:0], byte_ofs[7:0], 16'h0000, idx[3:0]}
function [127:0] gen_block_data;
    input [3:0] idx;
    input [4:0] size;       // block size in bytes
    input [3:0] size_code;  // size code from CQE (1/2/4/8/15)
    integer i;
    begin
        gen_block_data = 128'h0;
        for (i = 0; i < size; i = i + 4) begin
            gen_block_data[(i*8)+:32] = {size_code[3:0], i[7:0], 16'h0000, idx[3:0]};
        end
    end
endfunction

// State machine sequential
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        state_r    <= #DLY IDLE;
        idx_r      <= #DLY 4'd0;
        gap_cnt_r  <= #DLY 8'd0;
    end
    else begin
        state_r <= #DLY state_n;
        if (state_r == IDLE && start) begin
            idx_r      <= #DLY 4'd0;
            gap_cnt_r  <= #DLY 8'd0;
        end
        else if (state_r == NEXT) begin
            idx_r      <= #DLY idx_r + 4'd1;
            gap_cnt_r  <= #DLY 8'd0;
        end
        else if (state_r == PROD_GAP) begin
            gap_cnt_r <= #DLY gap_cnt_r + 8'd1;
        end
    end
end

// State machine combinational
// WRITE_DATA and WRITE_CQE each last 1 cycle, issuing a single m_wr_req_en pulse
// Then transition to WAIT_BRESP/WAIT_CQE to wait for m_wr_resp_en from MST_WR_CTRL
always @(*) begin
    state_n = state_r;
    case (state_r)
        IDLE:        if (start)                                          state_n = WRITE_DATA;
        WRITE_DATA:                                                      state_n = WAIT_BRESP;
        WAIT_BRESP:  if (m_wr_resp_en)                                   state_n = WRITE_CQE;
        WRITE_CQE:                                                       state_n = WAIT_CQE;
        WAIT_CQE:    if (m_wr_resp_en)                                   state_n = NEXT;
        NEXT:        if (idx_r == NUM_BLOCKS - 1)                        state_n = DONE;
                     else if (PROD_GAP_EN)                              state_n = PROD_GAP;
                     else                                                state_n = WRITE_DATA;
        PROD_GAP:    if (gap_cnt_r >= prod_interval - 1)                 state_n = WRITE_DATA;
        DONE:        if (~start)                                        state_n = IDLE;
        default:                                                        state_n = IDLE;
    endcase
end

// Request generation
// Data Block size: varying by idx%5 when SIZE_MODE=0, fixed FIXED_SIZE when SIZE_MODE=1
wire [4:0] w_size;       // block size in bytes
wire [3:0] w_size_code;  // size code stored in CQE (16 encoded as 15, 4-bit max)
assign w_size = SIZE_MODE ? FIXED_SIZE[4:0] :
                (idx_r % 5) == 0 ? 5'd1  :
                (idx_r % 5) == 1 ? 5'd2  :
                (idx_r % 5) == 2 ? 5'd4  :
                (idx_r % 5) == 3 ? 5'd8  : 5'd16;
assign w_size_code = SIZE_MODE ? FIXED_SIZE[3:0] :
                     (idx_r % 5) == 0 ? 4'd1  :
                     (idx_r % 5) == 1 ? 4'd2  :
                     (idx_r % 5) == 2 ? 4'd4  :
                     (idx_r % 5) == 3 ? 4'd8  : 4'd15;

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        m_wr_req_en_r    <= #DLY 1'b0;
        m_wr_req_id_r    <= #DLY {`AXI_ID_W{1'b0}};
        m_wr_req_addr_r  <= #DLY {`AXI_ADDR_W{1'b0}};
        m_wr_req_size_r  <= #DLY 8'h40;
        m_wr_req_burst_r <= #DLY 2'b01;
        m_wr_req_data_r  <= #DLY 128'h0;
        m_wr_req_strb_r  <= #DLY 16'h0;
    end
    else if (state_r == WRITE_DATA) begin
        // Write Data Block[idx]: at 0x10 + idx*0x10 (16B aligned), size varies
        m_wr_req_en_r    <= #DLY 1'b1;
        m_wr_req_id_r    <= #DLY idx_r;
        m_wr_req_addr_r  <= #DLY 16'h10 + {idx_r, 4'h0};  // 0x10 + idx*0x10
        m_wr_req_size_r  <= #DLY w_size;                   // block size (1,2,4,8,16)
        m_wr_req_burst_r <= #DLY BURST_TYPE;  // Configurable burst type
        m_wr_req_data_r  <= #DLY gen_block_data(idx_r, w_size, w_size_code);
        // Strobe: 1 byte -> 16'h0001, 2 bytes -> 16'h0003, 4 bytes -> 16'h000F,
        //         8 bytes -> 16'h00FF, 16 bytes -> 16'hFFFF
        case (w_size)
            5'd1:  m_wr_req_strb_r <= #DLY 16'h0001;
            5'd2:  m_wr_req_strb_r <= #DLY 16'h0003;
            5'd4:  m_wr_req_strb_r <= #DLY 16'h000F;
            5'd8:  m_wr_req_strb_r <= #DLY 16'h00FF;
            5'd16: m_wr_req_strb_r <= #DLY 16'hFFFF;
            default: m_wr_req_strb_r <= #DLY 16'h0001;
        endcase
    end
    else if (state_r == WRITE_CQE) begin
        // Write CQE[idx]: 1 byte at 0x00 + idx, {SIZE_CODE[3:0], 3'b000, VALID}
        m_wr_req_en_r    <= #DLY 1'b1;
        m_wr_req_id_r    <= #DLY idx_r;
        m_wr_req_addr_r  <= #DLY {12'h000, idx_r};       // 0x00 + idx
        m_wr_req_size_r  <= #DLY 8'h01;                   // 1 byte
        m_wr_req_burst_r <= #DLY BURST_TYPE;  // Configurable burst type
        m_wr_req_data_r  <= #DLY {120'h0, w_size_code, 3'b000, 1'b1};  // {SIZE_CODE, 0, VALID}
        m_wr_req_strb_r  <= #DLY 16'h0001;               // only byte 0
    end
    else begin
        m_wr_req_en_r    <= #DLY 1'b0;
    end
end

// Done signal
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        done_r <= #DLY 1'b0;
    end
    else if (state_r == DONE) begin
        done_r <= #DLY 1'b1;
    end
    else if (~start) begin
        done_r <= #DLY 1'b0;
    end
end

assign done = done_r;

// -----------------------------------------------------------------------
// Instantiate master write controller
// Producer connects to MST_WR_CTRL via m_wr_req_*/m_wr_resp_* interface
// MST_WR_CTRL manages AXI AW/W/B handshaking and OST pipelining internally
// Exposes simple req/resp interface to upper layer
// -----------------------------------------------------------------------
EASYAXI_MST_WR_CTRL #(
    .OST_DEPTH  (OST_DEPTH  ),
    .LOCAL_DATA_W (128        )
) U_EASYAXI_MST_WR_CTRL(
    .clk             (clk             ),
    .rst_n           (rst_n           ),

    .m_wr_req_en     (m_wr_req_en_r   ),
    .m_wr_req_id     (m_wr_req_id_r   ),
    .m_wr_req_addr   (m_wr_req_addr_r ),
    .m_wr_req_size   (m_wr_req_size_r ),
    .m_wr_req_burst  (m_wr_req_burst_r),
    .m_wr_req_data   (m_wr_req_data_r ),
    .m_wr_req_strb   (m_wr_req_strb_r ),

    .m_wr_resp_en    (m_wr_resp_en    ),
    .m_wr_resp_id    (m_wr_resp_id    ),
    .m_wr_resp_resp  (m_wr_resp_resp  ),

    .axi_mst_awvalid (axi_mst_awvalid ),
    .axi_mst_awready (axi_mst_awready ),
    .axi_mst_awid    (axi_mst_awid    ),
    .axi_mst_awaddr  (axi_mst_awaddr  ),
    .axi_mst_awlen   (axi_mst_awlen   ),
    .axi_mst_awsize  (axi_mst_awsize  ),
    .axi_mst_awburst (axi_mst_awburst ),
    .axi_mst_awuser  (axi_mst_awuser  ),

    .axi_mst_wvalid  (axi_mst_wvalid  ),
    .axi_mst_wready  (axi_mst_wready  ),
    .axi_mst_wdata   (axi_mst_wdata   ),
    .axi_mst_wstrb   (axi_mst_wstrb   ),
    .axi_mst_wlast   (axi_mst_wlast   ),
    .axi_mst_wuser   (axi_mst_wuser   ),

    .axi_mst_bvalid  (axi_mst_bvalid  ),
    .axi_mst_bready  (axi_mst_bready  ),
    .axi_mst_bid     (axi_mst_bid     ),
    .axi_mst_bresp   (axi_mst_bresp   ),
    .axi_mst_buser   (axi_mst_buser   )
);

endmodule
