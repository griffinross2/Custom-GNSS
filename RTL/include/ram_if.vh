`ifndef RAM_IF_VH
`define RAM_IF_VH

`include "common_types.vh"
import common_types_pkg::*;

interface ram_if;
    word_t addr;
    logic [3:0] wen;
    logic ren;
    word_t load;
    word_t store;
    ram_state_t state;

    modport ram (
        input addr, wen, ren, store,
        output load, state
    );

    modport ramctrl (
        input load, state,
        output addr, wen, ren, store
    );

endinterface

`endif // RAM_IF_VH