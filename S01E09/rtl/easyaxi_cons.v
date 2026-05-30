// +FHDR----------------------------------------------------------------------------
//                 Copyright (c) 2025 
//                       ALL RIGHTS RESERVED
// ---------------------------------------------------------------------------------
// Filename      : easyaxi_cons.v
// Author        : Rongye
// Created On    : 2026-05-05
// Last Modified : 2026-05-08
// ---------------------------------------------------------------------------------
// Description   : CQE Consumer module.
//   Polls CQE table, reads data blocks when CQE is valid, verifies data.
//   - Polls CQE[idx] at address 0x00 + idx every 16 cycles
//   - When VALID=1, reads Data Block[idx] at 0x40 + idx*0x10
//   - Verifies data: each word[3:0] should equal idx
//   - AXI ID = idx
//
// -FHDR----------------------------------------------------------------------------
module EASYAXI_CONS #(
    parameter OST_DEPTH    = 16,               // Outstanding transaction depth (power of 2)
    parameter NUM_BLOCKS   = 15,               // Number of blocks to read (0~NUM_BLOCKS-1)
    parameter BURST_TYPE   = `AXI_BURST_INCR,  // Burst type: FIXED/INCR/WRAP
    parameter SIZE_MODE    = 0,                // 0=varying per CQE, 1=fixed size from FIXED_SIZE
    parameter [7:0] FIXED_SIZE = 8'd16         // Data block size when SIZE_MODE=1, overrides CQE
)(
    input  wire                      clk,
    input  wire                      rst_n,

    // Control interface
    input  wire                      start,       // Trigger to start read transactions
    output wire                      done,        // All read transactions completed
    output wire                      error,       // Data mismatch detected

// AXI AR Channel
    output wire                      axi_mst_arvalid,
    input  wire                      axi_mst_arready,
    output wire  [`AXI_ID_W    -1:0] axi_mst_arid,
    output wire  [`AXI_ADDR_W  -1:0] axi_mst_araddr,
    output wire  [`AXI_LEN_W   -1:0] axi_mst_arlen,
    output wire  [`AXI_SIZE_W  -1:0] axi_mst_arsize,
    output wire  [`AXI_BURST_W -1:0] axi_mst_arburst,
    output wire  [`AXI_USER_W  -1:0] axi_mst_aruser,

// AXI R Channel
    input  wire                      axi_mst_rvalid,
    output wire                      axi_mst_rready,
    input  wire  [`AXI_ID_W    -1:0] axi_mst_rid,
    input  wire  [`AXI_DATA_W  -1:0] axi_mst_rdata,
    input  wire  [`AXI_RESP_W  -1:0] axi_mst_rresp,
    input  wire                      axi_mst_rlast,
    input  wire  [`AXI_USER_W  -1:0] axi_mst_ruser
);

localparam DLY = 0.1;
localparam POLL_INTERVAL = 16;

// State machine: poll CQE table -> if valid, read Data Block -> verify -> next
// -------------------------------------------------------
// State flow diagram:
//
//                    +-> POLL_WAIT(16cycle) -> READ_CQE -> WAIT_CQE -> CHECK_CQE
//                    |                                                       |
//   start            |              +---- VALID=1 (HIT) <----+---- VALID=1? -+
//   +---+            |              |                         |       |       ^
//   |   v            |              v                         |   VALID=0     |
//  IDLE ------+   (every 16 cycles)  READ_DATA                |   (MISS)      |
//   ^                        |      |                         |       v       |
//   |                   WAIT_DATA   |                    (continue polling)  POLL_WAIT-+
//   |                        v      |
//   |   start=0           VERIFY <--+
//   |   return to         (data verify)
//   |   IDLE                    |
//   |                          NEXT
//   |                           |
//   |                    idx == 15? -> DONE
//   +--------------------------------+
// -------------------------------------------------------
localparam IDLE        = 4'd0;
localparam POLL_WAIT   = 4'd1;  // Poll interval: read CQE table every 16 cycles
localparam READ_CQE    = 4'd2;  // Issue CQE table read request (16 bytes at address 0x00)
localparam WAIT_CQE    = 4'd3;  // Wait for m_rd_resp_en from MST_RD_CTRL
localparam CHECK_CQE   = 4'd4;  // Check CQE[idx] VALID bit
localparam READ_DATA   = 4'd5;  // CQE valid, issue Data Block read request
localparam WAIT_DATA   = 4'd6;  // Wait for Data Block read to complete
localparam VERIFY      = 4'd7;  // Verify data: each word[3:0] should equal idx
localparam NEXT        = 4'd8;  // Check if all 16 blocks are done
localparam DONE        = 4'd9;  // All complete, assert done

reg [3:0] state_r, state_n;
reg [3:0] idx_r;       // Current block index (0-15)
reg [7:0] poll_cnt_r;  // Poll interval counter
reg [3:0] r_size_code_r;  // Size code read from CQE

// Request generation signals -- simplified upper-layer interface, no AXI AR/R handling needed
reg                      m_rd_req_en_r;
reg  [`AXI_ID_W    -1:0] m_rd_req_id_r;
reg  [`AXI_ADDR_W  -1:0] m_rd_req_addr_r;
reg  [7:0]               m_rd_req_size_r;   // Byte count -> MST_RD_CTRL converts to AXI size/len
reg  [1:0]               m_rd_req_burst_r;

// Response signals -- read data return
wire                     m_rd_resp_en;       // Pulse: read data ready
wire [`AXI_ID_W    -1:0] m_rd_resp_id;
wire [127:0]             m_rd_resp_data;     // R beats collected into single 128-bit word
wire [`AXI_RESP_W  -1:0] m_rd_resp_resp;

// CQE valid check: extract bit 0 at position idx from 128-bit CQE table
wire cqe_valid;
wire cqe_invalid;  // Waveform-only: inverse of cqe_valid for easier MISS spotting
assign cqe_valid   = (state_r == CHECK_CQE) && m_rd_resp_data[(idx_r*8) +: 1];
assign cqe_invalid = (state_r == CHECK_CQE) && ~m_rd_resp_data[(idx_r*8) +: 1];

reg                      done_r;
reg                      error_r;

// State machine sequential
always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        state_r       <= #DLY IDLE;
        idx_r         <= #DLY 4'd0;
        poll_cnt_r    <= #DLY 8'd0;
        r_size_code_r <= #DLY 4'd0;
    end
    else begin
        state_r <= #DLY state_n;
        if (state_r == IDLE && start) begin
            idx_r         <= #DLY 4'd0;
            poll_cnt_r    <= #DLY 8'd0;
            r_size_code_r <= #DLY 4'd0;
        end
        else if (state_r == POLL_WAIT) begin
            poll_cnt_r <= #DLY poll_cnt_r + 8'd1;
        end
        else if (state_r == CHECK_CQE) begin
            if (cqe_valid) begin
                r_size_code_r <= #DLY m_rd_resp_data[(idx_r*8)+7 -: 4];
            end else begin
                poll_cnt_r <= #DLY 8'd0;
            end
        end
        else if (state_r == NEXT) begin
            idx_r      <= #DLY idx_r + 4'd1;
            poll_cnt_r <= #DLY 8'd0;
        end
    end
end

// State machine combinational
// CHECK_CQE is the key branch point:
//   VALID=1 (HIT)  -> READ_DATA, CQE ready, read Data Block directly
//   VALID=0 (MISS) -> POLL_WAIT, not ready yet, retry after 16 cycles
always @(*) begin
    state_n = state_r;
    case (state_r)
        IDLE: begin
            if (start) state_n = POLL_WAIT;
        end
        POLL_WAIT: begin
            if (poll_cnt_r >= POLL_INTERVAL - 1) state_n = READ_CQE;
        end
        READ_CQE: begin
            state_n = WAIT_CQE;
        end
        WAIT_CQE: begin
            if (m_rd_resp_en) state_n = CHECK_CQE;
        end
        CHECK_CQE: begin
            if (cqe_valid) state_n = READ_DATA;
            else state_n = POLL_WAIT;
        end
        READ_DATA: begin
            state_n = WAIT_DATA;
        end
        WAIT_DATA: begin
            if (m_rd_resp_en) state_n = VERIFY;
        end
        VERIFY: begin
            state_n = NEXT;
        end
        NEXT: begin
            if (idx_r == NUM_BLOCKS - 1) state_n = DONE;
            else state_n = POLL_WAIT;
        end
        DONE: begin
            if (~start) state_n = IDLE;
        end
        default: state_n = IDLE;
    endcase
end

// Decode size code from CQE to actual byte size
// When SIZE_MODE=1, use FIXED_SIZE directly instead of decoding CQE
wire [4:0] r_size;
assign r_size = SIZE_MODE ? FIXED_SIZE[4:0] :
                (r_size_code_r == 4'd1)  ? 5'd1  :
                (r_size_code_r == 4'd2)  ? 5'd2  :
                (r_size_code_r == 4'd4)  ? 5'd4  :
                (r_size_code_r == 4'd8)  ? 5'd8  : 5'd16;

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        m_rd_req_en_r    <= #DLY 1'b0;
        m_rd_req_id_r    <= #DLY {`AXI_ID_W{1'b0}};
        m_rd_req_addr_r  <= #DLY {`AXI_ADDR_W{1'b0}};
        m_rd_req_size_r  <= #DLY 8'h00;
        m_rd_req_burst_r <= #DLY BURST_TYPE;
    end
    else if (state_r == READ_CQE) begin
        // Read entire CQE table (16 bytes at address 0x00): fetch all 16 CQE entries at once
        m_rd_req_en_r    <= #DLY 1'b1;
        m_rd_req_id_r    <= #DLY idx_r;
        m_rd_req_addr_r  <= #DLY 16'h0000;                // CQE table base
        m_rd_req_size_r  <= #DLY 8'h10;                   // 16 bytes -> 4 beats on 32-bit AXI
        m_rd_req_burst_r <= #DLY BURST_TYPE;
    end
    else if (state_r == READ_DATA) begin
        // Read Data Block[idx]: address 0x10 + idx*0x10, size decoded from CQE
        m_rd_req_en_r    <= #DLY 1'b1;
        m_rd_req_id_r    <= #DLY idx_r;
        m_rd_req_addr_r  <= #DLY 16'h10 + {idx_r, 4'h0};  // Data Block base
        m_rd_req_size_r  <= #DLY r_size;                   // Block size decoded from CQE
        m_rd_req_burst_r <= #DLY BURST_TYPE;
    end
    else begin
        m_rd_req_en_r    <= #DLY 1'b0;
    end
end

// Data verification
// Check if a word at byte offset 'byte_ofs' matches expected data
function check_word;
    input [3:0] idx;
    input [7:0] byte_ofs;
    input [127:0] data;
    begin
        check_word = (data[byte_ofs*8 +: 4] === idx);
    end
endfunction

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        error_r <= #DLY 1'b0;
    end
    else if (state_r == IDLE && start) begin
        error_r <= #DLY 1'b0;
    end
    else if (state_r == VERIFY) begin
        // Check response status
        if (m_rd_resp_resp != `AXI_RESP_OKAY) begin
            $display("CONS ERROR [idx=%0d]: resp=%0d (expect OKAY=%0d)", idx_r, m_rd_resp_resp, `AXI_RESP_OKAY);
            error_r <= #DLY 1'b1;
        end
        else begin
            // Verify each word: bits [31:28]=SIZE, bits [3:0]=idx
            // Only check bytes that are actually written per wstrb
            if (r_size >= 5'd1 && m_rd_resp_data[3:0] !== idx_r) begin
                $display("CONS ERROR [idx=%0d, size=%0d]: byte0[3:0]=%0d (expect %0d)", idx_r, r_size, m_rd_resp_data[3:0], idx_r);
                error_r <= #DLY 1'b1;
            end
            if (r_size >= 5'd4 && m_rd_resp_data[31:28] !== r_size_code_r) begin
                $display("CONS ERROR [idx=%0d, size=%0d]: byte0[31:28]=%0d (expect SIZE=%0d)", idx_r, r_size, m_rd_resp_data[31:28], r_size_code_r);
                error_r <= #DLY 1'b1;
            end
            if (r_size >= 5'd5 && m_rd_resp_data[35:32] !== idx_r) begin
                $display("CONS ERROR [idx=%0d, size=%0d]: byte4[3:0]=%0d (expect %0d)", idx_r, r_size, m_rd_resp_data[35:32], idx_r);
                error_r <= #DLY 1'b1;
            end
            if (r_size >= 5'd8 && m_rd_resp_data[63:60] !== r_size_code_r) begin
                $display("CONS ERROR [idx=%0d, size=%0d]: byte4[31:28]=%0d (expect SIZE=%0d)", idx_r, r_size, m_rd_resp_data[63:60], r_size_code_r);
                error_r <= #DLY 1'b1;
            end
            if (r_size >= 5'd9 && m_rd_resp_data[67:64] !== idx_r) begin
                $display("CONS ERROR [idx=%0d, size=%0d]: byte8[3:0]=%0d (expect %0d)", idx_r, r_size, m_rd_resp_data[67:64], idx_r);
                error_r <= #DLY 1'b1;
            end
            if (r_size >= 5'd12 && m_rd_resp_data[95:92] !== r_size_code_r) begin
                $display("CONS ERROR [idx=%0d, size=%0d]: byte8[31:28]=%0d (expect SIZE=%0d)", idx_r, r_size, m_rd_resp_data[95:92], r_size_code_r);
                error_r <= #DLY 1'b1;
            end
            if (r_size >= 5'd13 && m_rd_resp_data[99:96] !== idx_r) begin
                $display("CONS ERROR [idx=%0d, size=%0d]: byte12[3:0]=%0d (expect %0d)", idx_r, r_size, m_rd_resp_data[99:96], idx_r);
                error_r <= #DLY 1'b1;
            end
            if (r_size >= 5'd16 && m_rd_resp_data[127:124] !== r_size_code_r) begin
                $display("CONS ERROR [idx=%0d, size=%0d]: byte12[31:28]=%0d (expect SIZE=%0d)", idx_r, r_size, m_rd_resp_data[127:124], r_size_code_r);
                error_r <= #DLY 1'b1;
            end
        end
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

assign done  = done_r;
assign error = error_r;

// -----------------------------------------------------------------------
// Instantiate master read controller
// Consumer only operates m_rd_req_en + addr + size;
// MST_RD_CTRL handles AR handshake, R beat collection, OST management, and response ordering
// -----------------------------------------------------------------------
EASYAXI_MST_RD_CTRL #(
    .OST_DEPTH  (OST_DEPTH  ),
    .LOCAL_DATA_W (128        )
) U_EASYAXI_MST_RD_CTRL(
    .clk             (clk             ),
    .rst_n           (rst_n           ),

    .m_rd_req_en     (m_rd_req_en_r   ),
    .m_rd_req_id     (m_rd_req_id_r   ),
    .m_rd_req_addr   (m_rd_req_addr_r ),
    .m_rd_req_size   (m_rd_req_size_r ),
    .m_rd_req_burst  (m_rd_req_burst_r),

    .m_rd_resp_en    (m_rd_resp_en    ),
    .m_rd_resp_id    (m_rd_resp_id    ),
    .m_rd_resp_data  (m_rd_resp_data  ),
    .m_rd_resp_resp  (m_rd_resp_resp  ),

    .axi_mst_arvalid (axi_mst_arvalid ),
    .axi_mst_arready (axi_mst_arready ),
    .axi_mst_arid    (axi_mst_arid    ),
    .axi_mst_araddr  (axi_mst_araddr  ),
    .axi_mst_arlen   (axi_mst_arlen   ),
    .axi_mst_arsize  (axi_mst_arsize  ),
    .axi_mst_arburst (axi_mst_arburst ),
    .axi_mst_aruser  (axi_mst_aruser  ),

    .axi_mst_rvalid  (axi_mst_rvalid  ),
    .axi_mst_rready  (axi_mst_rready  ),
    .axi_mst_rid     (axi_mst_rid     ),
    .axi_mst_rdata   (axi_mst_rdata   ),
    .axi_mst_rresp   (axi_mst_rresp   ),
    .axi_mst_rlast   (axi_mst_rlast   ),
    .axi_mst_ruser   (axi_mst_ruser   )
);

endmodule
