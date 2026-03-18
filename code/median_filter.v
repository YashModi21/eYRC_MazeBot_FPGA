`timescale 1ns / 1ps

// Simple compare-and-swap module
module compare_swap (
    input  [15:0] a,
    input  [15:0] b,
    output [15:0] min,
    output [15:0] max
);
    assign min = (a < b) ? a : b;
    assign max = (a < b) ? b : a;
endmodule

// 5-sample median filter (returns median of d0..d4)
module median_filter_5 (
    input  [15:0] d0,
    input  [15:0] d1,
    input  [15:0] d2,
    input  [15:0] d3,
    input  [15:0] d4,
    output [15:0] median
);

    wire [15:0] a0,a1,a2,a3,a4;
    wire [15:0] b0,b1,b2,b3,b4;
    wire [15:0] c0,c1,c2,c3,c4;
    wire [15:0] d5,d6,d7,d8,d9;
    wire [15:0] d10; // temporary / unused min for final compare

    compare_swap s1(d0,d1,a0,a1);
    compare_swap s2(d2,d3,a2,a3);
    assign a4 = d4;

    compare_swap s3(a0,a2,b0,b2);
    compare_swap s4(a1,a3,b1,b3);
    assign b4 = a4;

    compare_swap s5(b1,b2,c1,c2);
    assign c0 = b0;
    assign c3 = b3;
    assign c4 = b4;

    compare_swap s6(c2,c4,d7,d9);
    compare_swap s7(c1,c3,d6,d8);
    assign d5 = c0;

    compare_swap s8(d6,d7,d10,median);

endmodule
