`define LC_MODE
`ifndef DATA_18
    `define PI_BY_2         18'h06488
    `define PI              18'h0c910
    `define THREE_PI_BY_2   18'h12d98
    `define ONE             18'b0001_0000_0000_0000_00
`else
    `define PI_BY_2         16'h1922
    `define PI              16'h3244
    `define THREE_PI_BY_2   16'h4b66
    `define ONE             16'b0001_0000_0000_0000
`endif
