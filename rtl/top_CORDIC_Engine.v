module top_CORDIC_Engine#(
    parameter DATA_WIDTH = 18,
    parameter N_PE = 15
)
(
    input i_clk,
    input i_rst_n,
    
    input i_rm_vm,
    input signed [DATA_WIDTH - 1 : 0] in_x,
    input signed [DATA_WIDTH - 1 : 0] in_y,
    input signed [DATA_WIDTH - 1 : 0] in_alpha,
    input [DATA_WIDTH -1 : 0] in_atan_0,
    input i_valid_in,

    output reg signed [DATA_WIDTH - 1 : 0] out_costheta,
    output reg signed [DATA_WIDTH - 1 : 0] out_sintheta,
    output reg signed [DATA_WIDTH - 1 : 0] out_alpha,
    output reg o_valid_out
);

   


    /*  Pre-processing: 
            1. In Rotation mode: Mapping the input angle to appropriate quadrants
            2. In Vectoring mode: Mapping the input x & y to the quadrants
    */
    // Note: Angle should be in the radians in [0,2*pi]
    
    reg signed [DATA_WIDTH-1 : 0] r_i_alpha1, r_i_alpha2;
    reg signed [DATA_WIDTH-1 : 0] diff1, diff2, diff3;

    reg signed [DATA_WIDTH-1 : 0] r_x1,r_x2;
    reg signed [DATA_WIDTH-1 : 0] r_y1,r_y2;

    reg diff_valid;

    always@(posedge i_clk) begin
        case(i_rm_vm)

            // Rotation Mode
            1'b0: begin
                if(i_valid_in) begin
                    r_x1 <= in_x;
                    r_y1 <= in_y;
                    diff1 <= in_alpha - 16'h1922;
                    diff2 <= in_alpha - 16'h3244;
                    diff3 <= in_alpha - 16'h4b66;
                    r_i_alpha1 <= in_alpha;
                    diff_valid <= 1'b1;
                end
                else diff_valid <= 1'b0;
            end

            // Vectoring Mode
            1'b1: begin

                if(i_valid_in) begin
                    r_x1       <= in_x;
                    r_y1       <= in_y;
                    r_i_alpha1 <= in_alpha;
                    diff_valid <= 1'b1;
                end
                else diff_valid <= 1'b0;
            end
        
        endcase
    end

    wire v1, v2, v3;
    assign v1 = diff1[DATA_WIDTH-1];
    assign v2 = diff2[DATA_WIDTH-1];
    assign v3 = diff3[DATA_WIDTH-1];

    wire [1:0] w_i_quadrant;
    assign w_i_quadrant[1] = (i_rm_vm)? (r_y1[DATA_WIDTH-1]) : (~v1&~v2);
    assign w_i_quadrant[0] = (i_rm_vm)? (r_y1[DATA_WIDTH-1] ^ r_x1[DATA_WIDTH-1]) : (~v1&(~(v2^v3)));

    reg [1:0] quadrant;
    reg quadrant_valid;
    always@(posedge i_clk) begin
        if(diff_valid) begin
            case(w_i_quadrant)
                2'b00: begin 
                    quadrant <= 2'b00; // Q1
                    quadrant_valid <= 1'b1;
                    r_i_alpha2 <= r_i_alpha1;
                    r_x2 <= r_x1;
                    r_y2 <= r_y1;
                end

                2'b01: begin
                    quadrant <= 2'b01; // Q2
                    quadrant_valid <= 1'b1;
                    r_i_alpha2 <= (~i_rm_vm)? diff1 : r_i_alpha1;
                    r_x2 <= (~i_rm_vm)? r_x1 : r_y1;
                    r_y2 <= (~i_rm_vm)? r_y1 : -r_x1;
                end

                2'b10: begin
                    quadrant <= 2'b10; // Q2
                    quadrant_valid <= 1'b1;
                    r_i_alpha2 <= (~i_rm_vm)? diff2 : r_i_alpha1;
                    r_x2 <= (~i_rm_vm)? r_x1 : -r_x1;
                    r_y2 <= (~i_rm_vm)? r_y1 : -r_y1;
                end

                2'b11: begin
                    quadrant <= 2'b11;
                    quadrant_valid <= 1'b1;
                    r_i_alpha2 <= (~i_rm_vm)? diff3 : r_i_alpha1;
                    r_x2 <= (~i_rm_vm)? r_x1 : -r_y1;
                    r_y2 <= (~i_rm_vm)? r_y1 : r_x1;
                end

                default: quadrant_valid <= 1'b0;
            endcase
        end
        else quadrant_valid <= 1'b0;
    end

    reg r_quadrant_valid;
    always@(posedge i_clk) r_quadrant_valid <= quadrant_valid;

    wire [1:0] w_quadrant;

    wire [DATA_WIDTH-1 : 0] w_costheta, w_sintheta;

    wire [DATA_WIDTH-1 : 0] w_o_alpha;

    wire w_o_valid;

    wire [DATA_WIDTH-1 : 0] w_atan;

    /* ------------------ Dynamic_atan_coefficient_generator ------------ */
    dynamic_atan #(
        .N_PE(N_PE),
        .DATA_WIDTH(DATA_WIDTH)
    )
    dynamic_atan_inst (
        .i_clk(i_clk),
        .i_rstn(i_rst_n),
        .i_data(in_atan_0),
        .i_valid(quadrant_valid),
        .o_atan_data(w_atan),
        .o_valid(),
        .o_done()
    );
    
    
    /* ------------------ CORDIC ENGINE ----------------------- */
    CORDIC_Engine # (
        .DATA_WIDTH(DATA_WIDTH),
        .N_PE(N_PE)
    )
    CORDIC_Engine_inst (
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .i_rm_vm(i_rm_vm),
        .in_x(r_x2),
        .in_y(r_y2),
        .in_alpha(r_i_alpha2),
        .in_atan(w_atan),
        .i_quadrant(quadrant),
        .valid_in(r_quadrant_valid),
        .out_x(w_costheta),
        .out_y(w_sintheta),
        .out_alpha(w_o_alpha),
        .out_quadrant(w_quadrant),
        .valid_out(w_o_valid)
    );


    /* ---------------------- Post-processing the CORDIC Engine result ---------------- */
    wire [DATA_WIDTH-1 : 0] twos_comp_costheta, twos_comp_sintheta;
    assign twos_comp_costheta = ~w_costheta + 1;
    assign twos_comp_sintheta = ~w_sintheta + 1;

    always@(posedge i_clk) begin
        if(w_o_valid) begin
	        
            case(w_quadrant)
                2'b00: begin
                    out_costheta <= w_costheta;
                    out_sintheta <= w_sintheta;
                    out_alpha    <= w_o_alpha;
                    o_valid_out  <= 1'b1;
                end 

                2'b01: begin
                    out_costheta <= (~i_rm_vm)? twos_comp_sintheta : w_costheta;
                    out_sintheta <= (~i_rm_vm)? w_costheta : w_sintheta;
                    out_alpha    <= (~i_rm_vm)? w_o_alpha : w_o_alpha + 16'h1922;
                    o_valid_out  <= 1'b1;
                end

                2'b10: begin
                    out_costheta <= (~i_rm_vm)? twos_comp_costheta : w_costheta;
                    out_sintheta <= (~i_rm_vm)? twos_comp_sintheta : w_sintheta;
                    out_alpha    <= (~i_rm_vm)? w_o_alpha : w_o_alpha + 16'h3244;
                    o_valid_out  <= 1'b1;
                end

                2'b11: begin
                    out_costheta <= (~i_rm_vm)? w_sintheta : w_costheta;
                    out_sintheta <= (~i_rm_vm)? twos_comp_costheta : w_sintheta;
                    out_alpha    <= (~i_rm_vm)? w_o_alpha : w_o_alpha + 16'h4b66;
                    o_valid_out  <= 1'b1;
                end

                default: o_valid_out <= 1'b0;
            endcase
        end

        else o_valid_out <= 1'b0;
    end


endmodule
