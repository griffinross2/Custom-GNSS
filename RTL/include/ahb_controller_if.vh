`ifndef AHB_MASTER_IF_VH
`define AHB_MASTER_IF_VH

`include "common_types.vh"
import common_types_pkg::*;

interface ahb_controller_if;
    
    // Memory control signals from datapath
    logic iread;
    logic dread;
    logic [1:0] dwrite;
    word_t iaddr;
    word_t daddr;
    word_t dstore;

    // Memory control signals to datapath
    logic ihit, dhit;
    word_t iload;
    word_t dload;

    modport ahb_controller (
        input iread, dread, dwrite, iaddr, daddr, dstore,
        output ihit, dhit, iload, dload
    );
    
    modport tb (
        output iread, dread, dwrite, iaddr, daddr, dstore,
        input ihit, dhit, iload, dload
    );
endinterface

`endif // AHB_MASTER_IF_VH