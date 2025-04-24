`ifndef REGISTER_FILE_IF_VH
`define REGISTER_FILE_IF_VH

`include "common_types.vh"
import common_types_pkg::*;

interface register_file_if;
    word_t wdat, rdat1, rdat2;
    reg_t wsel, rsel1, rsel2;
    logic wen;

    modport register_file (
        input wdat, wsel, wen, rsel1, rsel2,
        output rdat1, rdat2
    );
endinterface

`endif // REGISTER_FILE_IF_VH