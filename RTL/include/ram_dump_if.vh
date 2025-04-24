`ifndef RAM_DUMP_IF_VH
`define RAM_DUMP_IF_VH

`include "common_types.vh"
import common_types_pkg::*;

interface ram_dump_if;

  // CPU side
  logic iwait, dwait, iren, dren;
  logic [3:0] dwen;
  word_t iload, dload, dstore;
  word_t iaddr, daddr;
  logic override_ctrl;

  // Testbench
  modport tb (
    input override_ctrl, iren, dren, dwen, dstore, iaddr, daddr,
    output iwait, dwait, iload, dload  
  );

endinterface

`endif // RAM_DUMP_IF_VH
