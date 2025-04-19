`ifndef COMMON_GNSS_TYPES_VH
`define COMMON_GNSS_TYPES_VH

// Common types package
package common_gnss_types_pkg;

    parameter ACC_W = 16;                       // Accumulator width

    typedef logic [4:0] sv_t;                   // SV number type
    typedef logic [10:1] l1ca_lfsr_t;           // LFSR type for G1 and G2
    typedef logic [9:0] gps_chip_t;             // Chip index type
    typedef logic signed [ACC_W-1:0] acc_t;     // Accumulator type
endpackage

`endif // COMMON_GNSS_TYPES_VH
