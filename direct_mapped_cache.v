//==============================================================================
// direct_mapped_cache.v
//
// Parameterized direct-mapped cache controller (pure Verilog-2001/2005).
//   * Write-back, write-allocate
//   * Byte-enable (write strobe) support
//   * Single-cycle hit (combinational read, registered write)
//   * Miss handling by FSM: [WRITEBACK dirty victim] -> REFILL -> serve
//   * Hit/miss performance counters
//
// Address breakdown (byte address):
//   | TAG | INDEX | WORD_OFFSET | BYTE_OFFSET |
//
// Constraints (all must be powers of two):
//   DATA_WIDTH     >= 8 and a multiple of 8
//   WORDS_PER_LINE >= 2
//   NUM_LINES      >= 2
//
// CPU-side protocol:
//   Hold cpu_req (and addr/we/wdata/wstrb) stable until cpu_ready pulses.
//   cpu_ready is high for one cycle when the request completes. On a read,
//   cpu_rdata is valid in the same cycle as cpu_ready.
//
// Memory-side protocol (word-granular, handshake):
//   Controller holds mem_req/mem_we/mem_addr/mem_wdata stable until mem_ready.
//   mem_ready high = write word accepted (mem_we=1) OR mem_rdata valid (mem_we=0).
//==============================================================================
`timescale 1ns/1ps

module direct_mapped_cache #(
    parameter ADDR_WIDTH     = 32,   // byte address width
    parameter DATA_WIDTH     = 32,   // word width in bits
    parameter WORDS_PER_LINE = 4,    // words per cache line
    parameter NUM_LINES      = 64    // number of cache lines
)(
    input  wire                        clk,
    input  wire                        rst_n,       // active-low synchronous reset

    // ---------------- CPU / requester side ----------------
    input  wire                        cpu_req,
    input  wire                        cpu_we,
    input  wire [ADDR_WIDTH-1:0]       cpu_addr,
    input  wire [DATA_WIDTH-1:0]       cpu_wdata,
    input  wire [DATA_WIDTH/8-1:0]     cpu_wstrb,   // byte enables for writes
    output wire [DATA_WIDTH-1:0]       cpu_rdata,
    output wire                        cpu_ready,

    // ---------------- Main memory side ----------------
    output wire                        mem_req,
    output wire                        mem_we,
    output wire [ADDR_WIDTH-1:0]       mem_addr,
    output wire [DATA_WIDTH-1:0]       mem_wdata,
    input  wire [DATA_WIDTH-1:0]       mem_rdata,
    input  wire                        mem_ready,

    // ---------------- Statistics ----------------
    output reg  [31:0]                 hit_count,
    output reg  [31:0]                 miss_count
);

    //--------------------------------------------------------------------------
    // Derived parameters
    //--------------------------------------------------------------------------
    localparam BYTES_PER_WORD = DATA_WIDTH / 8;
    localparam BYTE_OFF_W     = $clog2(BYTES_PER_WORD);
    localparam WORD_OFF_W     = $clog2(WORDS_PER_LINE);
    localparam INDEX_W        = $clog2(NUM_LINES);
    localparam TAG_W          = ADDR_WIDTH - INDEX_W - WORD_OFF_W - BYTE_OFF_W;

    // Address field positions
    localparam WORD_LO  = BYTE_OFF_W;
    localparam INDEX_LO = BYTE_OFF_W + WORD_OFF_W;
    localparam TAG_LO   = BYTE_OFF_W + WORD_OFF_W + INDEX_W;

    // FSM states
    localparam [1:0] S_IDLE      = 2'd0,
                     S_WRITEBACK = 2'd1,
                     S_REFILL    = 2'd2;

    //--------------------------------------------------------------------------
    // Storage
    //--------------------------------------------------------------------------
    reg [DATA_WIDTH-1:0] data_array [0:NUM_LINES*WORDS_PER_LINE-1];
    reg [TAG_W-1:0]      tag_array  [0:NUM_LINES-1];
    reg [NUM_LINES-1:0]  valid;
    reg [NUM_LINES-1:0]  dirty;

    //--------------------------------------------------------------------------
    // Address decode
    //--------------------------------------------------------------------------
    wire [TAG_W-1:0]      req_tag    = cpu_addr[ADDR_WIDTH-1 : TAG_LO];
    wire [INDEX_W-1:0]    req_index  = cpu_addr[TAG_LO-1     : INDEX_LO];
    wire [WORD_OFF_W-1:0] req_word   = cpu_addr[INDEX_LO-1   : WORD_LO];

    wire [TAG_W-1:0]      stored_tag = tag_array[req_index];
    wire                  line_valid = valid[req_index];
    wire                  line_dirty = dirty[req_index];
    wire                  hit        = line_valid && (stored_tag == req_tag);

    wire [INDEX_W+WORD_OFF_W-1:0] hit_word_addr = {req_index, req_word};

    //--------------------------------------------------------------------------
    // FSM registers
    //--------------------------------------------------------------------------
    reg [1:0]             state;
    reg [WORD_OFF_W-1:0]  word_cnt;

    wire last_word = (word_cnt == WORDS_PER_LINE-1);

    wire [INDEX_W+WORD_OFF_W-1:0] fill_word_addr = {req_index, word_cnt};

    //--------------------------------------------------------------------------
    // CPU-side outputs
    //--------------------------------------------------------------------------
    assign cpu_ready = (state == S_IDLE) && cpu_req && hit;
    assign cpu_rdata = data_array[hit_word_addr];

    //--------------------------------------------------------------------------
    // Memory-side outputs
    //--------------------------------------------------------------------------
    wire in_wb     = (state == S_WRITEBACK);
    wire in_refill = (state == S_REFILL);

    assign mem_req   = in_wb | in_refill;
    assign mem_we    = in_wb;
    assign mem_wdata = data_array[fill_word_addr];

    // Writeback uses the victim's (old) tag; refill uses the requested tag.
    wire [TAG_W-1:0] mem_tag = in_wb ? stored_tag : req_tag;

    assign mem_addr = {mem_tag, req_index, word_cnt, {BYTE_OFF_W{1'b0}}};

    //--------------------------------------------------------------------------
    // Sequential logic: FSM + arrays + counters
    //--------------------------------------------------------------------------
    integer i;

    always @(posedge clk) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            word_cnt   <= {WORD_OFF_W{1'b0}};
            valid      <= {NUM_LINES{1'b0}};
            dirty      <= {NUM_LINES{1'b0}};
            hit_count  <= 32'd0;
            miss_count <= 32'd0;
        end else begin
            case (state)
                //--------------------------------------------------------------
                S_IDLE: begin
                    word_cnt <= {WORD_OFF_W{1'b0}};
                    if (cpu_req) begin
                        if (hit) begin
                            hit_count <= hit_count + 32'd1;
                            if (cpu_we) begin
                                for (i = 0; i < BYTES_PER_WORD; i = i + 1) begin
                                    if (cpu_wstrb[i])
                                        data_array[hit_word_addr][8*i +: 8]
                                            <= cpu_wdata[8*i +: 8];
                                end
                                dirty[req_index] <= 1'b1;
                            end
                        end else begin
                            miss_count <= miss_count + 32'd1;
                            if (line_valid && line_dirty)
                                state <= S_WRITEBACK;
                            else
                                state <= S_REFILL;
                        end
                    end
                end

                //--------------------------------------------------------------
                // Write the dirty victim line back to memory, word by word
                S_WRITEBACK: begin
                    if (mem_ready) begin
                        if (last_word) begin
                            word_cnt <= {WORD_OFF_W{1'b0}};
                            state    <= S_REFILL;
                        end else begin
                            word_cnt <= word_cnt + 1'b1;
                        end
                    end
                end

                //--------------------------------------------------------------
                // Fetch the requested line from memory, word by word
                S_REFILL: begin
                    if (mem_ready) begin
                        data_array[fill_word_addr] <= mem_rdata;
                        if (last_word) begin
                            tag_array[req_index] <= req_tag;
                            valid[req_index]     <= 1'b1;
                            dirty[req_index]     <= 1'b0;
                            word_cnt             <= {WORD_OFF_W{1'b0}};
                            state                <= S_IDLE;  // request now hits
                        end else begin
                            word_cnt <= word_cnt + 1'b1;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
