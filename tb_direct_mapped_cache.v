`timescale 1ns/1ps
//------------------------------------------------------------------------------
// Self-checking testbench: directed tests + constrained-random traffic checked
// against a reference memory model. Prints PASS/FAIL.
//------------------------------------------------------------------------------
module tb_direct_mapped_cache;

    parameter ADDR_WIDTH     = 16;
    parameter DATA_WIDTH     = 32;
    parameter WORDS_PER_LINE = 4;
    parameter NUM_LINES      = 16;     // 16*4*4 = 256-byte cache
    parameter MEM_LATENCY    = 2;      // extra wait cycles per memory word
    parameter MEM_WORDS      = 4096;   // 16 KB backing memory
    parameter NUM_RANDOM     = 5000;

    localparam BYTE_OFF_W   = $clog2(DATA_WIDTH/8);
    localparam CACHE_BYTES  = NUM_LINES*WORDS_PER_LINE*(DATA_WIDTH/8);
    localparam [DATA_WIDTH/8-1:0] ALL = {(DATA_WIDTH/8){1'b1}};

    reg                       clk = 0;
    reg                       rst_n = 0;
    reg                       cpu_req = 0, cpu_we = 0;
    reg  [ADDR_WIDTH-1:0]     cpu_addr = 0;
    reg  [DATA_WIDTH-1:0]     cpu_wdata = 0;
    reg  [DATA_WIDTH/8-1:0]   cpu_wstrb = 0;
    wire [DATA_WIDTH-1:0]     cpu_rdata;
    wire                      cpu_ready;
    wire                      mem_req, mem_we;
    wire [ADDR_WIDTH-1:0]     mem_addr;
    wire [DATA_WIDTH-1:0]     mem_wdata;
    wire [DATA_WIDTH-1:0]     mem_rdata;
    wire                      mem_ready;
    wire [31:0]               hit_count, miss_count;

    direct_mapped_cache #(
        .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH),
        .WORDS_PER_LINE(WORDS_PER_LINE), .NUM_LINES(NUM_LINES)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .cpu_req(cpu_req), .cpu_we(cpu_we), .cpu_addr(cpu_addr),
        .cpu_wdata(cpu_wdata), .cpu_wstrb(cpu_wstrb),
        .cpu_rdata(cpu_rdata), .cpu_ready(cpu_ready),
        .mem_req(mem_req), .mem_we(mem_we), .mem_addr(mem_addr),
        .mem_wdata(mem_wdata), .mem_rdata(mem_rdata), .mem_ready(mem_ready),
        .hit_count(hit_count), .miss_count(miss_count)
    );

    always #5 clk = ~clk;

    //---------------- Behavioral main memory ----------------
    reg [DATA_WIDTH-1:0] mem     [0:MEM_WORDS-1];
    reg [DATA_WIDTH-1:0] ref_mem [0:MEM_WORDS-1];
    integer lat = 0;

    assign mem_ready = mem_req && (lat == MEM_LATENCY);
    assign mem_rdata = mem[mem_addr[ADDR_WIDTH-1:BYTE_OFF_W]];

    always @(posedge clk) begin
        if (!mem_req || mem_ready) lat <= 0;
        else                       lat <= lat + 1;
        if (mem_ready && mem_we)
            mem[mem_addr[ADDR_WIDTH-1:BYTE_OFF_W]] <= mem_wdata;
    end

    //---------------- Bookkeeping ----------------
    integer errors = 0;
    integer k, n, timeout;

    task cpu_access(input we, input [ADDR_WIDTH-1:0] addr,
                    input [DATA_WIDTH-1:0] wdata, input [DATA_WIDTH/8-1:0] wstrb);
        reg [DATA_WIDTH-1:0] expected;
        integer b;
        reg done;
        begin
            @(negedge clk);
            cpu_req = 1; cpu_we = we; cpu_addr = addr;
            cpu_wdata = wdata; cpu_wstrb = wstrb;
            timeout = 0;
            done = 0;
            // cpu_ready is sampled at the rising edge (pre-NBA values), i.e.
            // exactly what the DUT sees when it completes the request.
            while (!done && timeout < 1000) begin
                @(posedge clk);
                if (cpu_ready) begin
                    done = 1;
                    if (!we) begin
                        expected = ref_mem[addr[ADDR_WIDTH-1:BYTE_OFF_W]];
                        if (cpu_rdata !== expected) begin
                            $display("FAIL: read addr=%h got=%h exp=%h t=%0t",
                                     addr, cpu_rdata, expected, $time);
                            errors = errors + 1;
                        end
                    end else begin
                        for (b = 0; b < DATA_WIDTH/8; b = b + 1)
                            if (wstrb[b])
                                ref_mem[addr[ADDR_WIDTH-1:BYTE_OFF_W]][8*b +: 8] = wdata[8*b +: 8];
                    end
                end
                timeout = timeout + 1;
            end
            if (!done) begin
                $display("FAIL: timeout addr=%h", addr);
                errors = errors + 1;
            end
            @(negedge clk);
            cpu_req = 0;
        end
    endtask

    initial begin
        for (k = 0; k < MEM_WORDS; k = k + 1) begin
            mem[k]     = k * 32'h01010101 + 32'h1234;
            ref_mem[k] = k * 32'h01010101 + 32'h1234;
        end
        repeat (4) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);

        // ---- Directed ----
        cpu_access(0, 16'h0040, 0, ALL);                    // cold read miss
        cpu_access(0, 16'h0040, 0, ALL);                    // hit
        cpu_access(0, 16'h0044, 0, ALL);                    // same line hit
        cpu_access(1, 16'h0040, 32'hDEADBEEF, ALL);         // write hit
        cpu_access(0, 16'h0040, 0, ALL);
        cpu_access(1, 16'h0044, 32'hAABBCCDD, 4'b0101);     // partial write
        cpu_access(0, 16'h0044, 0, ALL);
        cpu_access(0, 16'h0040 + CACHE_BYTES, 0, ALL);      // conflict -> writeback
        cpu_access(0, 16'h0040, 0, ALL);                    // re-read written data
        cpu_access(1, 16'h0080 + CACHE_BYTES, 32'h11223344, ALL); // write miss (allocate)
        cpu_access(0, 16'h0080 + CACHE_BYTES, 0, ALL);

        // ---- Random ----
        for (n = 0; n < NUM_RANDOM; n = n + 1) begin
            cpu_access($random & 1,
                       ($random & (MEM_WORDS*(DATA_WIDTH/8)-1)) & ~((DATA_WIDTH/8)-1),
                       $random,
                       ($random & 1) ? ALL : $random);
        end

        // ---- Final consistency: every word read back through the cache ----
        for (k = 0; k < MEM_WORDS; k = k + 1)
            cpu_access(0, k*(DATA_WIDTH/8), 0, ALL);

        $display("hits=%0d misses=%0d", hit_count, miss_count);
        if (errors == 0) $display("PASS: all checks passed");
        else             $display("FAIL: %0d errors", errors);
        $finish;
    end

    initial begin
        #500000000;
        $display("FAIL: global timeout");
        $finish;
    end
endmodule
