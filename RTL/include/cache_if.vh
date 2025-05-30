`ifndef CACHE_IF_VH
`define CACHE_IF_VH

`include "common_types.vh"
import common_types_pkg::*;

interface cache_if;
    
    // Memory control signals from datapath
    logic read;             // Read word
    logic [1:0] write;      // 0: no write, 1: byte, 2: halfword, 3: word
    word_t addr;            // Address to be accessed
    word_t store;           // Data to be stored
    logic done;             // Asserted when done (e.g. pipeline stage proceeding)

    // Memory control signals to datapath
    logic ready;            // Tell datapath that the read/write is done
    word_t load;            // Data loaded from memory

    // Other signals
    logic halt;
    logic flushed;

    modport cache (
        input read, write, addr, store, done, halt,
        output ready, load, flushed
    );

    modport datapath (
        output read, write, addr, store, done, halt,
        input ready, load, flushed
    );
    
    modport tb (
        output read, write, addr, store, done, halt,
        input ready, load, flushed
    );
endinterface

`endif // CACHE_IF_VH