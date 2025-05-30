/******************/
/* D-Cache module */
/******************/
`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;
`include "axi_controller_if.vh"
`include "cache_if.vh"

module dcache (
    input logic clk, nrst,
    axi_controller_if.cache amif,
    cache_if.cache cif
);

typedef enum logic [3:0] {
    IDLE,
    CHECK,
    READ_0,
    READ_1,
    READ_2,
    READ_3,
    WRITEBACK_0,
    WRITEBACK_1,
    WRITEBACK_2,
    WRITEBACK_3,
    FLUSH_CHECK,
    FLUSH_0,
    FLUSH_1,
    FLUSH_2,
    FLUSH_3,
    FLUSH_DONE
} dcache_state_t;

dcache_state_t state, next_state;
logic hit;
logic block_idx;    // which block in set got a hit
logic lru_idx;      // which block in set is Least Recently Used
logic next_lru_idx; // next state for lru
logic write_idx;    // which block in set is being written to
logic ren;
logic wen;
dcache_addr_t addr;
word_t wdata [0:3];     // 4 words
word_t rdata0 [0:3];    // 4 words
word_t rdata1 [0:3];    // 4 words
dcache_meta_t wmeta0;   // Tag + valid bit + dirty bit + lru bit
dcache_meta_t wmeta1;   // Tag + valid bit + dirty bit + lru bit
dcache_meta_t rmeta0;   // Tag + valid bit + dirty bit + lru bit
dcache_meta_t rmeta1;   // Tag + valid bit + dirty bit + lru bit
dcache_meta_t rmeta [0:1];
assign rmeta[0] = rmeta0;
assign rmeta[1] = rmeta1;
logic [ICACHE_SET_IDX_W:0] flush_block, next_flush_block;

always_ff @(posedge clk) begin
    if (~nrst) begin
        state <= IDLE;
    end else begin
        state <= next_state;
    end
end

// Next state logic
always_comb begin
    next_state = state;
    case (state)
        IDLE: begin
            if (!cif.halt && (cif.read || |cif.write)) begin
                next_state = CHECK;
            end
        end

        CHECK: begin
            if (hit && cif.done) begin
                next_state = IDLE;
            end else if (!hit && !(rmeta[lru_idx].valid && rmeta[lru_idx].dirty)) begin
                next_state = READ_0;
            end else if (!hit && rmeta[lru_idx].valid && rmeta[lru_idx].dirty) begin
                next_state = WRITEBACK_0;
            end
        end

        READ_0: begin
            if (amif.ready) begin
                next_state = READ_1;
            end
        end

        READ_1: begin
            if (amif.ready) begin
                next_state = READ_2;
            end
        end

        READ_2: begin
            if (amif.ready) begin
                next_state = READ_3;
            end
        end

        READ_3: begin
            if (amif.ready && cif.done) begin
                next_state = IDLE;
            end
        end

        WRITEBACK_0: begin
            if (amif.ready) begin
                next_state = WRITEBACK_1;
            end
        end

        WRITEBACK_1: begin
            if (amif.ready) begin
                next_state = WRITEBACK_2;
            end
        end

        WRITEBACK_2: begin
            if (amif.ready) begin
                next_state = WRITEBACK_3;
            end
        end

        WRITEBACK_3: begin
            if (amif.ready) begin
                next_state = READ_0;
            end
        end

        FLUSH_CHECK: begin
            if (flush_block == 10'd1023 && !(rmeta[flush_block[0]].valid && rmeta[flush_block[0]].dirty)) begin
                next_state = FLUSH_DONE;
            end else if (!hit && !(rmeta[flush_block[0]].valid && rmeta[flush_block[0]].dirty)) begin
                next_state = FLUSH_CHECK;
            end else if (!hit && rmeta[flush_block[0]].valid && rmeta[flush_block[0]].dirty) begin
                next_state = FLUSH_0;
            end
        end

        default: begin
            next_state = IDLE;
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
    wdata = '{default: '0};

    next_lru_idx = '0;

    wmeta0 = rmeta0;
    wmeta1 = rmeta1;
    
    hit = 1'b0;
    block_idx = '0;
    lru_idx = '0;
    write_idx = '0;

    // TEMPORARY
    cif.flushed = cif.halt;
    
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
        IDLE: begin
            if (!cif.halt && cif.read || |cif.write) begin
                // Signal BRAM to read
                ren = 1'b1;
            end
        end

        CHECK: begin
            // Keep reading this data
            ren = 1'b1;

            if (hit && cif.read) begin
                // Tell datapath we have the data
                cif.load = block_idx ? rdata1[addr.word_off] : rdata0[addr.word_off];
                cif.ready = 1'b1;

                // If done, change lru
                if (cif.done) begin
                    // Update the cache entry (dummy write data)
                    next_lru_idx = ~block_idx;
                    wen = 1'b1;
                    wdata = write_idx ? rdata1 : rdata0;
                    
                    // Update the metadata
                    wmeta0.lru = next_lru_idx;
                    wmeta1.lru = next_lru_idx;
                end
            end else if (hit && |cif.write) begin
                // Tell datapath we are ready
                cif.ready = 1'b1;

                // If done, change lru and write data
                if (cif.done) begin
                    // Update the cache entry
                    next_lru_idx = ~block_idx;
                    wen = 1'b1;
                    write_idx = block_idx;
                    wdata = write_idx ? rdata1 : rdata0;
                    wdata[addr.word_off] = cif.store;
                    
                    // Update the metadata
                    wmeta0.lru = next_lru_idx;
                    wmeta1.lru = next_lru_idx;
                    if (block_idx) begin
                        wmeta1.dirty = 1'b1;
                    end else begin
                        wmeta0.dirty = 1'b0;
                    end
                end
            end else if (!hit && !(rmeta[lru_idx].valid && rmeta[lru_idx].dirty)) begin
                // Start the read (first word)
                amif.read = 1'b1;
                amif.addr = {cif.addr[31:4], 4'b0000};
            end else if (!hit && rmeta[lru_idx].valid && rmeta[lru_idx].dirty) begin
                // Start the writeback (first word)
                amif.write = 2'b11;
                amif.addr = {cif.addr[31:4], 4'b0000};
                amif.store =  lru_idx ? rdata1[0] : rdata0[0];
            end
        end

        READ_0: begin
            // Keep reading this cache until done
            ren = 1'b1;

            // Keep reading the first word from the memory
            amif.read = 1'b1;
            amif.addr = {cif.addr[31:4], 4'b0000};

            if (amif.ready) begin
                // Start reading the second word from the memory
                amif.read = 1'b1;
                amif.addr = {cif.addr[31:4], 4'b0100};

                // Write the first word to cache (keeping other words the same)
                wen = 1'b1;
                write_idx = lru_idx;
                wdata[0] = amif.load;
                wdata[1] = lru_idx ? rdata1[1] : rdata0[1];
                wdata[2] = lru_idx ? rdata1[2] : rdata0[2];
                wdata[3] = lru_idx ? rdata1[3] : rdata0[3];

                // Signal that we are done
                amif.done = 1'b1;
            end
        end

        READ_1: begin
            // Keep reading this cache until done
            ren = 1'b1;

            // Keep reading the second word from the memory
            amif.read = 1'b1;
            amif.addr = {cif.addr[31:4], 4'b0100};

            if (amif.ready) begin
                // Start reading the fourth word from the memory
                amif.read = 1'b1;
                amif.addr = {cif.addr[31:4], 4'b1000};

                // Write the second word to cache (keeping other words the same)
                wen = 1'b1;
                write_idx = lru_idx;
                wdata[0] = lru_idx ? rdata1[0] : rdata0[0];
                wdata[1] = amif.load;
                wdata[2] = lru_idx ? rdata1[2] : rdata0[2];
                wdata[3] = lru_idx ? rdata1[3] : rdata0[3];

                // Signal that we are done
                amif.done = 1'b1;
            end
        end

        READ_2: begin
            // Keep reading this cache until done
            ren = 1'b1;

            // Keep reading the third word from the memory
            amif.read = 1'b1;
            amif.addr = {cif.addr[31:4], 4'b1000};

            if (amif.ready) begin
                // Start reading the fourth word from the memory
                amif.read = 1'b1;
                amif.addr = {cif.addr[31:4], 4'b1100};

                // Write the third word to cache (keeping other words the same)
                wen = 1'b1;
                write_idx = lru_idx;
                wdata[0] = lru_idx ? rdata1[0] : rdata0[0];
                wdata[1] = lru_idx ? rdata1[1] : rdata0[1];
                wdata[2] = amif.load;
                wdata[3] = lru_idx ? rdata1[3] : rdata0[3];

                // Signal that we are done
                amif.done = 1'b1;
            end
        end

        READ_3: begin
            // Keep reading this cache until done
            ren = 1'b1;

            // Keep reading the fourth word from the memory
            amif.read = 1'b1;
            amif.addr = {cif.addr[31:4], 4'b1100};

            if (amif.ready) begin
                // Provide the data to the datapath
                cif.load = amif.load;
                cif.ready = 1'b1;

                if (cif.done) begin
                    // Write the fourth word to cache (keeping other words the same)
                    wen = 1'b1;
                    write_idx = lru_idx;
                    wdata[0] = lru_idx ? rdata1[0] : rdata0[0];
                    wdata[1] = lru_idx ? rdata1[1] : rdata0[1];
                    wdata[2] = lru_idx ? rdata1[2] : rdata0[2];
                    wdata[3] = amif.load;

                    // Update the metadata
                    next_lru_idx = ~lru_idx;    // We are writing to LRU, so next LRU is the other block
                    if (lru_idx == 1'b0) begin
                        wmeta0.tag = addr.tag;
                        wmeta0.valid = 1'b1;
                        wmeta0.dirty = 1'b0;
                        wmeta0.lru = next_lru_idx;
                        wmeta1.lru = next_lru_idx;
                    end else begin
                        wmeta1.tag = addr.tag;
                        wmeta1.valid = 1'b1;
                        wmeta1.dirty = 1'b0;
                        wmeta1.lru = next_lru_idx;
                        wmeta0.lru = next_lru_idx;
                    end

                    // Signal that we are done
                    amif.done = 1'b1;
                end
            end
        end

        WRITEBACK_0: begin
            // Keep reading this cache until done
            ren = 1'b1;

            // Keep writing the first word to memory
            amif.write = 2'b11;
            amif.addr = {rmeta[lru_idx].tag, addr.set_index, 4'b0000};  // Reconstruct address
            amif.store = lru_idx ? rdata1[0] : rdata0[0];

            if (amif.ready) begin
                // Start writing the second word to memory
                amif.write = 2'b11;
                amif.addr = {rmeta[lru_idx].tag, addr.set_index, 4'b0100};  // Reconstruct address
                amif.store =  lru_idx ? rdata1[1] : rdata0[1];

                // Signal that we are done
                amif.done = 1'b1;
            end
        end

        WRITEBACK_1: begin
            // Keep reading this cache until done
            ren = 1'b1;

            // Keep writing the second word to memory
            amif.write = 2'b11;
            amif.addr = {rmeta[lru_idx].tag, addr.set_index, 4'b0100};  // Reconstruct address
            amif.store =  lru_idx ? rdata1[1] : rdata0[1];

            if (amif.ready) begin
                // Start writing the third word to memory
                amif.write = 2'b11;
                amif.addr = {rmeta[lru_idx].tag, addr.set_index, 4'b1000};  // Reconstruct address
                amif.store =  lru_idx ? rdata1[2] : rdata0[2];

                // Signal that we are done
                amif.done = 1'b1;
            end
        end

        WRITEBACK_2: begin
            // Keep reading this cache until done
            ren = 1'b1;

            // Keep writing the third word to memory
            amif.write = 2'b11;
            amif.addr = {rmeta[lru_idx].tag, addr.set_index, 4'b1000};  // Reconstruct address
            amif.store =  lru_idx ? rdata1[2] : rdata0[2];

            if (amif.ready) begin
                // Start writing the fourth word to memory
                amif.write = 2'b11;
                amif.addr = {rmeta[lru_idx].tag, addr.set_index, 4'b1100};  // Reconstruct address
                amif.store =  lru_idx ? rdata1[3] : rdata0[3];

                // Signal that we are done
                amif.done = 1'b1;
            end
        end

        WRITEBACK_3: begin
            // Keep reading this cache until done
            ren = 1'b1;

            // Keep writing the fourth word to memory
            amif.write = 2'b11;
            amif.addr = {rmeta[lru_idx].tag, addr.set_index, 4'b1100};  // Reconstruct address
            amif.store =  lru_idx ? rdata1[3] : rdata0[3];

            if (amif.ready) begin
                // Start the read (first word)
                amif.read = 1'b1;
                amif.addr = {cif.addr[31:4], 4'b0000};

                // Signal that we are done
                amif.done = 1'b1;
            end
        end

        default: begin
        end
    endcase
end

// Cache RAM

// First block of set - metadata
xpm_memory_spram #(
    .ADDR_WIDTH_A(DCACHE_SET_IDX_W),
    .MEMORY_SIZE(512 * (DCACHE_TAG_W + 3)),  // 512 sets
    .WRITE_DATA_WIDTH_A(DCACHE_TAG_W + 3), // Tag + valid + dirty + lru
    .BYTE_WRITE_WIDTH_A(DCACHE_TAG_W + 3),
    .READ_DATA_WIDTH_A(DCACHE_TAG_W + 3),
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
    .MEMORY_SIZE(512 * (DCACHE_TAG_W + 3)),  // 512 sets
    .WRITE_DATA_WIDTH_A(DCACHE_TAG_W + 3), // Tag + valid + dirty + lru
    .BYTE_WRITE_WIDTH_A(DCACHE_TAG_W + 3),
    .READ_DATA_WIDTH_A(DCACHE_TAG_W + 3),
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

// First block of set - data word 0
xpm_memory_spram #(
    .ADDR_WIDTH_A(ICACHE_SET_IDX_W),
    .MEMORY_SIZE(512 * 32),  // 512 sets
    .WRITE_DATA_WIDTH_A(32),
    .BYTE_WRITE_WIDTH_A(32),
    .READ_DATA_WIDTH_A(32),
    .READ_LATENCY_A(1),
    .RST_MODE_A("ASYNC"),
    .MEMORY_PRIMITIVE("block")
) block_0_data_0 (
    .clka(clk),
    .rsta(~nrst),
    .ena(1'b1),
    .wea(wen & (write_idx == 1'b0)),
    .addra(addr.set_index),
    .dina(wdata[0]),
    .douta(rdata0[0]),
    .injectdbiterra(1'b0),
    .injectsbiterra(1'b0),
    .regcea(1'b1),
    .sleep(1'b0)
);

// First block of set - data word 1
xpm_memory_spram #(
    .ADDR_WIDTH_A(ICACHE_SET_IDX_W),
    .MEMORY_SIZE(512 * 32),  // 512 sets
    .WRITE_DATA_WIDTH_A(32),
    .BYTE_WRITE_WIDTH_A(32),
    .READ_DATA_WIDTH_A(32),
    .READ_LATENCY_A(1),
    .RST_MODE_A("ASYNC"),
    .MEMORY_PRIMITIVE("block")
) block_0_data_1 (
    .clka(clk),
    .rsta(~nrst),
    .ena(1'b1), 
    .wea(wen & (write_idx == 1'b0)), 
    .addra(addr.set_index),
    .dina(wdata[1]),
    .douta(rdata0[1]),
    .injectdbiterra(1'b0),
    .injectsbiterra(1'b0),
    .regcea(1'b1),
    .sleep(1'b0)
);

// First block of set - data word 2
xpm_memory_spram #(
    .ADDR_WIDTH_A(ICACHE_SET_IDX_W),
    .MEMORY_SIZE(512 * 32),  // 512 sets
    .WRITE_DATA_WIDTH_A(32),
    .BYTE_WRITE_WIDTH_A(32),
    .READ_DATA_WIDTH_A(32),
    .READ_LATENCY_A(1),
    .RST_MODE_A("ASYNC"),
    .MEMORY_PRIMITIVE("block")
) block_0_data_2 (
    .clka(clk),
    .rsta(~nrst),
    .ena(1'b1), 
    .wea(wen & (write_idx == 1'b0)), 
    .addra(addr.set_index),
    .dina(wdata[2]),
    .douta(rdata0[2]),
    .injectdbiterra(1'b0),
    .injectsbiterra(1'b0),
    .regcea(1'b1),
    .sleep(1'b0)
);

// First block of set - data word 3
xpm_memory_spram #(
    .ADDR_WIDTH_A(ICACHE_SET_IDX_W),
    .MEMORY_SIZE(512 * 32),  // 512 sets
    .WRITE_DATA_WIDTH_A(32),
    .BYTE_WRITE_WIDTH_A(32),
    .READ_DATA_WIDTH_A(32),
    .READ_LATENCY_A(1),
    .RST_MODE_A("ASYNC"),
    .MEMORY_PRIMITIVE("block")
) block_0_data_3 (
    .clka(clk),
    .rsta(~nrst),
    .ena(1'b1), 
    .wea(wen & (write_idx == 1'b0)), 
    .addra(addr.set_index),
    .dina(wdata[3]),
    .douta(rdata0[3]),
    .injectdbiterra(1'b0),
    .injectsbiterra(1'b0),
    .regcea(1'b1),
    .sleep(1'b0)
);

// Second block of set - data word 0
xpm_memory_spram #(
    .ADDR_WIDTH_A(ICACHE_SET_IDX_W),
    .MEMORY_SIZE(512 * 32),  // 512 sets
    .WRITE_DATA_WIDTH_A(32),
    .BYTE_WRITE_WIDTH_A(32),
    .READ_DATA_WIDTH_A(32),
    .READ_LATENCY_A(1),
    .RST_MODE_A("ASYNC"),
    .MEMORY_PRIMITIVE("block")
) block_1_data_0 (
    .clka(clk),
    .rsta(~nrst),
    .ena(1'b1), 
    .wea(wen & (write_idx == 1'b1)), 
    .addra(addr.set_index),
    .dina(wdata[0]),
    .douta(rdata1[0]),
    .injectdbiterra(1'b0),
    .injectsbiterra(1'b0),
    .regcea(1'b1),
    .sleep(1'b0)
);

// Second block of set - data word 1
xpm_memory_spram #(
    .ADDR_WIDTH_A(ICACHE_SET_IDX_W),
    .MEMORY_SIZE(512 * 32),  // 512 sets
    .WRITE_DATA_WIDTH_A(32),
    .BYTE_WRITE_WIDTH_A(32),
    .READ_DATA_WIDTH_A(32),
    .READ_LATENCY_A(1),
    .RST_MODE_A("ASYNC"),
    .MEMORY_PRIMITIVE("block")
) block_1_data_1 (
    .clka(clk),
    .rsta(~nrst),
    .ena(1'b1), 
    .wea(wen & (write_idx == 1'b1)), 
    .addra(addr.set_index),
    .dina(wdata[1]),
    .douta(rdata1[1]),
    .injectdbiterra(1'b0),
    .injectsbiterra(1'b0),
    .regcea(1'b1),
    .sleep(1'b0)
);

// Second block of set - data word 2
xpm_memory_spram #(
    .ADDR_WIDTH_A(ICACHE_SET_IDX_W),
    .MEMORY_SIZE(512 * 32),  // 512 sets
    .WRITE_DATA_WIDTH_A(32),
    .BYTE_WRITE_WIDTH_A(32),
    .READ_DATA_WIDTH_A(32),
    .READ_LATENCY_A(1),
    .RST_MODE_A("ASYNC"),
    .MEMORY_PRIMITIVE("block")
) block_1_data_2 (
    .clka(clk),
    .rsta(~nrst),
    .ena(1'b1), 
    .wea(wen & (write_idx == 1'b1)), 
    .addra(addr.set_index),
    .dina(wdata[2]),
    .douta(rdata1[2]),
    .injectdbiterra(1'b0),
    .injectsbiterra(1'b0),
    .regcea(1'b1),
    .sleep(1'b0)
);

// Second block of set - data word 3
xpm_memory_spram #(
    .ADDR_WIDTH_A(ICACHE_SET_IDX_W),
    .MEMORY_SIZE(512 * 32),  // 512 sets
    .WRITE_DATA_WIDTH_A(32),
    .BYTE_WRITE_WIDTH_A(32),
    .READ_DATA_WIDTH_A(32),
    .READ_LATENCY_A(1),
    .RST_MODE_A("ASYNC"),
    .MEMORY_PRIMITIVE("block")
) block_1_data_3 (
    .clka(clk),
    .rsta(~nrst),
    .ena(1'b1), 
    .wea(wen & (write_idx == 1'b1)), 
    .addra(addr.set_index),
    .dina(wdata[3]),
    .douta(rdata1[3]),
    .injectdbiterra(1'b0),
    .injectsbiterra(1'b0),
    .regcea(1'b1),
    .sleep(1'b0)
);

endmodule