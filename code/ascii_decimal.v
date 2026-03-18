// ASCII decimal encoder
// Converts an 8-bit value (0-255) to three ASCII digits (hundreds, tens, ones)
module ascii_decimal(
    input  [7:0] value,
    output [7:0] ascii_hundreds,
    output [7:0] ascii_tens,
    output [7:0] ascii_ones
);

    wire [7:0] hundreds = value / 8'd100;
    wire [7:0] tens_raw  = value % 8'd100;
    wire [7:0] tens = tens_raw / 8'd10;
    wire [7:0] ones = value % 8'd10;

    assign ascii_hundreds = 8'd48 + hundreds;
    assign ascii_tens     = 8'd48 + tens;
    assign ascii_ones     = 8'd48 + ones;

endmodule
