`timescale 1ns/1ps
`include "../rtl/top_CORDIC_Engine_HC.v"
`include "../rtl/CORDIC_Engine_HC.v"
`include "../rtl/dynamic_atanh.v"

module tb_top_CORDIC_Engine_v2();

    reg i_clk;
    reg i_rst_n;
    
    reg i_rm_vm;
    reg signed [DATA_WIDTH - 1 : 0] in_x;
    reg signed [DATA_WIDTH - 1 : 0] in_y;
    reg signed [DATA_WIDTH - 1 : 0] in_alpha;
    reg [DATA_WIDTH -1 : 0] in_atan_0;
    reg i_valid_in;

    wire signed [DATA_WIDTH - 1 : 0] out_costheta;
    wire signed [DATA_WIDTH - 1 : 0] out_sintheta;
    wire signed [DATA_WIDTH - 1 : 0] out_alpha;
    wire o_valid_out;

    initial i_clk = 1'b1;
    always #5 i_clk = ~i_clk;

    localparam DATA_WIDTH = 20;
    localparam N_PE = 19;


    top_CORDIC_Engine_HC # (
        .DATA_WIDTH(DATA_WIDTH),
        .N_PE(N_PE)
    )
    top_CORDIC_Engine_HC_inst (
      .i_clk(i_clk),
      .i_rst_n(i_rst_n),
      .i_rm_vm(i_rm_vm),
      .in_x(in_x),
      .in_y(in_y),
      .in_alpha(in_alpha),
      .in_atan_0(in_atan_0),
      .i_valid_in(i_valid_in),
      .out_costheta(out_costheta),
      .out_sintheta(out_sintheta),
      .out_alpha(out_alpha),
      .o_valid_out(o_valid_out)
    );

    real real_x = 0;
    real real_y = 0;

    initial begin
        $dumpfile("tb_top_CORDIC_Engine_HC_v1.vcd");
        $dumpvars(0);

        i_rst_n = 1'b0;
        i_valid_in = 1'b0;
        i_rm_vm = 1'b1;
        in_x = 0;
        in_y = 0;
        in_alpha = 0;
        in_atan_0 = 0;

        $display(" ---- Simulation starts for HC mode ---- ");
        #10 i_rst_n = 1'b1;
            i_rm_vm = 1'b0;  // HC-Rotation Mode
            in_x = 20'h13520; // Scale input x & y with 0.82816
            in_y = 20'h0;
            in_alpha = 18'h0cccd; //0.8
            in_atan_0 = 20'h08ca0;

            i_valid_in = 1'b1;
        #10 i_valid_in = 1'b0;
        $display("--------------------------------------------");
        $display("x=%f , y=%f", (in_x/(2**16.0))/1.20751, (in_y/(2**16.0))/1.20751);
        
        wait(o_valid_out);     
        #20 
        
        #200 
        $display(" ---- Simulation Ends ---- ");
        $finish;       
    end
    
endmodule
