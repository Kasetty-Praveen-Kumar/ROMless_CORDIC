// `include "../rtl/Arch_defines.vh"
`define ONE 20'b 0001_0000_0000_0000_0000
module dynamic_atanh#(
    parameter N_PE = 16,
    parameter DATA_WIDTH = 18
)(
    input i_clk,
    input i_rstn,

    input [DATA_WIDTH-1:0] i_data,
    input i_valid,

    output reg [DATA_WIDTH-1:0] o_atan_data,
    output reg o_valid,
    output reg o_done
);

    /* ------ Dynamic generation of atanh coefficients for CORDIC_PE ------ */
    /*
        It uses hybrid mechnaism to generate atan coefficients
        Iteration 0: Take the actaul value of atanh(2^1) from the input, since i = 0: atanh is infinite
        Iteration 1-5: Compute using Taylor series expansion
                    2^(-i) + ((2^(-3i)/4)+(2^(-3i)/16)+(2^(-3i)/64))  (Same as, x+(x^3)/3)
        Iteration 6-last: atan(2^(-i)) = 2^(-i)
    */

    reg [$clog2(N_PE) : 0] atan_counter = 1;
    reg [$clog2(N_PE) : 0] counter = 0;
    reg state;

    wire [DATA_WIDTH-1:0] inv_2_pow_i;
    wire [DATA_WIDTH-1:0] inv_2_pow_3i;
    
    wire [DATA_WIDTH-1:0] inv_2_pow_5i;

    assign inv_2_pow_i = `ONE >> atan_counter;
    assign inv_2_pow_3i = `ONE >> 3*atan_counter; 

    assign inv_2_pow_5i = `ONE >> 5*atan_counter; 

    reg count;

    always@(posedge i_clk) begin
        if(!i_rstn) begin
            atan_counter <= 1;
            counter <= 0;
            state <= 0;
            o_atan_data <= 0;
            o_valid <= 0;
            o_done <= 0;
            count <= 0;
        end else begin
            case(state)
                0: begin
                    count <= 0;
                    o_done <= 0;
                    if(i_valid) begin
                        o_atan_data <= i_data;
                        o_valid <= 1;
                        state <= 1;
                        atan_counter <= atan_counter + 1;
                        counter <= counter + 1;
                    end
                    else begin
                        o_valid <= 0;
                        state <= 0;
                        atan_counter <= atan_counter;
                        counter <= counter;
                    end
                end

                1: begin
                    if(counter>=1 && counter<5) begin
                        // atan_counter <= atan_counter + 1;
                        counter <= counter + 1;
                        if(atan_counter == 4) begin
                            if(~count) begin
                                atan_counter <= atan_counter;
                                count <= ~count;
                            end
                            else begin
                                atan_counter <= atan_counter + 1;
                                count <= ~count;
                            end
                        end
                        else begin
                            atan_counter <= atan_counter + 1;
                            count <= 0;
                        end
                        o_valid <= 1;
                        o_atan_data <= inv_2_pow_i + ((inv_2_pow_3i >> 2) + (inv_2_pow_3i >> 4) + (inv_2_pow_3i >> 6)) + 
                                                    ((inv_2_pow_5i >> 2) - (inv_2_pow_5i >> 5) - (inv_2_pow_5i >> 6)); 
                    end
                    else if(counter>=5) begin
                        if(counter == N_PE) begin
                            atan_counter <= 0; // Reset counter after reaching N_PE
                            counter <= 0;
                            state <= 0; // Go back to initial state
                            o_valid <= 0; // Reset valid signal
                            o_done <= 1; // Indicate completion
                        end else begin
                            o_atan_data <= inv_2_pow_i;
                            // atan_counter <= atan_counter + 1;
                            counter <= counter + 1;
                            if(atan_counter == 13) begin
                                if(~count) begin
                                    atan_counter <= atan_counter;
                                    count <= ~count;
                                end
                                else begin
                                    atan_counter <= atan_counter + 1;
                                    count <= ~count;
                                end
                            end
                            else begin
                                atan_counter <= atan_counter + 1;
                                count <= 0;
                            end
                            o_valid <= 1;
                        end
                    end
                end

                default: begin
                    o_valid <= 0;
                    state <= 0; // Reset to initial state
                    atan_counter <= 0; // Reset counter
                end

            endcase
        end
    end
endmodule