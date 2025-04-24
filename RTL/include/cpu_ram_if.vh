`ifndef CPU_RAM_IF_VH
`define CPU_RAM_IF_VH

`include "common_types.vh"
import common_types_pkg::*;

interface cpu_ram_if;

  // CPU side
  logic iwait, dwait, iren, dren;
  logic [3:0] dwen;
  word_t iload, dload, dstore;
  word_t iaddr, daddr;

  // Memory controller side
  modport ramctrl (
    input iren, dren, dwen, dstore, iaddr, daddr,
    output iwait, dwait, iload, dload  
  );

  // CPU side 
  modport cpu (
    input iwait, dwait, iload, dload,
    output iren, dren, dwen, dstore, iaddr, daddr
  );

endinterface

`endif // CPU_RAM_IF_VH
