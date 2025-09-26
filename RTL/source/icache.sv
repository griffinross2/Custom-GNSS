/******************/
/* I-Cache module */
/******************/
`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;
`include "axi_controller_if.vh"
`include "cache_if.vh"

module icache (
    input logic clk, nrst,
    axi_controller_if.cache amif,
    cache_if.cache cif
);

typedef enum logic [1:0] {
    CACHE_INIT,
    CACHE_IDLE,
    CACHE_CHECK,
    CACHE_READ
} icache_state_t;

icache_state_t state, next_state;
logic [ICACHE_SET_IDX_W-1:0] init_idx; // For initialization
logic hit;
logic block_idx;    // which block in set got a hit
logic lru_idx;      // which block in set is Least Recently Used
logic next_lru_idx; // next state for lru
logic ren;
logic wen;
icache_addr_t addr;
word_t wdata;
word_t rdata0;
word_t rdata1;
icache_meta_t wmeta0; // Tag + valid bit + lru bit
icache_meta_t wmeta1; // Tag + valid bit + lru bit
icache_meta_t rmeta0; // Tag + valid bit + lru bit
icache_meta_t rmeta1; // Tag + valid bit + lru bit

always_ff @(posedge clk) begin
    if (~nrst) begin
        state <= CACHE_INIT;
        init_idx <= '0;
    end else begin
        state <= next_state;
        init_idx <= init_idx + 1;
    end
end

// Next state logic
always_comb begin
    next_state = state;
    case (state)
        CACHE_INIT: begin
            if (init_idx + {{ICACHE_SET_IDX_W-1{1'b0}}, 1'b1} == '0) begin
                next_state = CACHE_IDLE;
            end else begin
                next_state = CACHE_INIT;
            end
        end
        
        CACHE_IDLE: begin
            if (cif.read) begin
                next_state = CACHE_CHECK;
            end
        end

        CACHE_CHECK: begin
            if (hit && cif.done) begin
                next_state = CACHE_IDLE;
            end else if (!hit) begin
                next_state = CACHE_READ;
            end
        end

        CACHE_READ: begin
            if (amif.ready && cif.done) begin
                next_state = CACHE_IDLE;
            end
        end

        default: begin
            next_state = CACHE_IDLE;
        end
    endcase
end

// State output logic
always_comb begin
    amif.read = '0;
    amif.write = '0;
    amif.store = '0;
    amif.done = '0;
    amif.addr = '0;

    cif.ready = '0;
    cif.load = '0;

    ren = '0;
    wen = '0;
    addr = cif.addr;
    wdata = '0;

    next_lru_idx = '0;

    wmeta0 = rmeta0;
    wmeta1 = rmeta1;
    
    hit = 1'b0;
    block_idx = '0;
    lru_idx = '0;
    
    // Cache hit detection
    
    // Check if the address is in the cache
    if (rmeta0.valid && rmeta0.tag == addr.tag) begin
        hit = 1'b1;
        block_idx = 1'b0;
    end

    if (rmeta1.valid && rmeta1.tag == addr.tag) begin
        hit = 1'b1;
        block_idx = 1'b1;
    end

    if (rmeta0.valid && rmeta1.valid) begin
        // Both blocks are valid, check LRU
        lru_idx = rmeta0.lru;
    end else if (rmeta0.valid) begin
        // Only block 0 is valid
        lru_idx = 1'b1;
    end else if (rmeta1.valid) begin
        // Only block 1 is valid
        lru_idx = 1'b0;
    end

    // State output

    case (state)
        CACHE_INIT: begin
            // Initialize the cache
            wen = 1'b1;

            // Set the address to the current index
            addr.set_index = init_idx;

            // Write default values to metadata
            wmeta0.valid = 1'b0;
            wmeta0.lru = 1'b0;
            wmeta1.valid = 1'b0;
            wmeta1.lru = 1'b0;
        end
        
        CACHE_IDLE: begin
            if (cif.read) begin
                // Signal BRAM to read
                ren = 1'b1;
            end
        end

        CACHE_CHECK: begin
            // Keep reading this data
            ren = 1'b1;

            if (hit) begin
                // Tell datapath we have the data
                cif.load = block_idx ? rdata1 : rdata0;
                cif.ready = 1'b1;

                // If done, change lru
                if (cif.done) begin
                    // Update the cache entry
                    next_lru_idx = ~block_idx;
                    wen = 1'b1;
                    wdata = lru_idx ? rdata1 : rdata0;
                    
                    // Update the metadata
                    wmeta0.lru = next_lru_idx;
                    wmeta1.lru = next_lru_idx;
                end
            end else begin
                // Start the read
                amif.read = 1'b1;
                amif.addr = cif.addr;
            end
        end

        CACHE_READ: begin
            // Keep reading this cache until done
            ren = 1'b1;

            // Keep reading the data from the memory
            amif.read = 1'b1;
            amif.addr = cif.addr;

            if (amif.ready) begin
                // Provide the data to the datapath
                cif.load = amif.load;
                cif.ready = 1'b1;

                if (cif.done) begin
                    // Write the data to the cache
                    next_lru_idx = ~lru_idx;    // We are writing to LRU, so next LRU is the other block
                    wen = 1'b1;
                    addr = cif.addr;
                    wdata = amif.load;

                    // Update the metadata
                    if (lru_idx == 1'b0) begin
                        wmeta0.tag = addr.tag;
                        wmeta0.valid = 1'b1;
                        wmeta0.lru = next_lru_idx;
                        wmeta1.lru = next_lru_idx;
                    end else begin
                        wmeta1.tag = addr.tag;
                        wmeta1.valid = 1'b1;
                        wmeta1.lru = next_lru_idx;
                        wmeta0.lru = next_lru_idx;
                    end

                    // Signal that we are done
                    amif.done = 1'b1;
                end
            end
        end

        default: begin
        end
    endcase
end

// Cache RAM

// First block of set - metadata
xpm_memory_spram #(
    .ADDR_WIDTH_A(ICACHE_SET_IDX_W),
    .MEMORY_SIZE(512 * (ICACHE_TAG_W + 2)),  // 512 sets
    .WRITE_DATA_WIDTH_A(ICACHE_TAG_W + 2), // Tag + valid + lru
    .BYTE_WRITE_WIDTH_A(ICACHE_TAG_W + 2),
    .READ_DATA_WIDTH_A(ICACHE_TAG_W + 2),
    .READ_LATENCY_A(1),
    .RST_MODE_A("ASYNC"),
    .MEMORY_PRIMITIVE("block")
) block_0_meta (
    .clka(clk),
    .rsta(~nrst),
    .ena(1'b1),
    .wea(wen),
    .addra(addr.set_index),
    .dina(wmeta0),
    .douta(rmeta0),
    .injectdbiterra(1'b0),
    .injectsbiterra(1'b0),
    .regcea(1'b1),
    .sleep(1'b0)
);

// Second block of set - metadata
xpm_memory_spram #(
    .ADDR_WIDTH_A(ICACHE_SET_IDX_W),
    .MEMORY_SIZE(512 * (ICACHE_TAG_W + 2)),  // 512 sets
    .WRITE_DATA_WIDTH_A(ICACHE_TAG_W + 2), // Tag + valid + bit
    .BYTE_WRITE_WIDTH_A(ICACHE_TAG_W + 2),
    .READ_DATA_WIDTH_A(ICACHE_TAG_W + 2),
    .READ_LATENCY_A(1),
    .RST_MODE_A("ASYNC"),
    .MEMORY_PRIMITIVE("block")
) block_1_meta (
    .clka(clk),
    .rsta(~nrst),
    .ena(1'b1),
    .wea(wen),
    .addra(addr.set_index),
    .dina(wmeta1),
    .douta(rmeta1),
    .injectdbiterra(1'b0),
    .injectsbiterra(1'b0),
    .regcea(1'b1),
    .sleep(1'b0)
);

// First block of set - data
xpm_memory_spram #(
    .ADDR_WIDTH_A(ICACHE_SET_IDX_W),
    .MEMORY_SIZE(512 * 32),  // 512 sets
    .WRITE_DATA_WIDTH_A(32),
    .BYTE_WRITE_WIDTH_A(32),
    .READ_DATA_WIDTH_A(32),
    .READ_LATENCY_A(1),
    .RST_MODE_A("ASYNC"),
    .MEMORY_PRIMITIVE("block")
) block_0_data (
    .clka(clk),
    .rsta(~nrst),
    .ena(1'b1),  // Only replace this block if it is LRU
    .wea(wen & (lru_idx == 1'b0)),  // Only replace this block if it is LRU
    .addra(addr.set_index),
    .dina(wdata),
    .douta(rdata0),
    .injectdbiterra(1'b0),
    .injectsbiterra(1'b0),
    .regcea(1'b1),
    .sleep(1'b0)
);

// Second block of set - data
xpm_memory_spram #(
    .ADDR_WIDTH_A(ICACHE_SET_IDX_W),
    .MEMORY_SIZE(512 * 32),  // 512 sets
    .WRITE_DATA_WIDTH_A(32),
    .BYTE_WRITE_WIDTH_A(32),
    .READ_DATA_WIDTH_A(32),
    .READ_LATENCY_A(1),
    .RST_MODE_A("ASYNC"),
    .MEMORY_PRIMITIVE("block")
) block_1_data (
    .clka(clk),
    .rsta(~nrst),
    .ena(1'b1),  // Only replace this block if it is LRU
    .wea(wen & (lru_idx == 1'b1)),  // Only replace this block if it is LRU
    .addra(addr.set_index),
    .dina(wdata),
    .douta(rdata1),
    .injectdbiterra(1'b0),
    .injectsbiterra(1'b0),
    .regcea(1'b1),
    .sleep(1'b0)
);

endmodule