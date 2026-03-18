`timescale 1ns / 1ps

/*
Module: frequency_scaling for motor driver l298n
*/

module frequency_scaling (
    input clk_50M,
    output reg clk_3125KHz
);

initial begin
    clk_3125KHz = 0;
end

reg [2:0] counter = 0;

always @ (posedge clk_50M) begin
    if (!counter) clk_3125KHz <= ~clk_3125KHz; 
    counter <= counter + 1'b1; 
end

endmodule