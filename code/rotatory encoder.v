`timescale 1ns / 1ps

/*
Module: Rotatory Encoder Decoder (Signed Position)
Decodes quadrature signals from rotatory encoders on N20 motors
Tracks signed motor position and direction
*/

module rotatory_encoder (
    input  wire        clk_50M,
    input  wire        reset,                // active-low reset
    input  wire        encoder_a,
    input  wire        encoder_b,
    output reg signed [19:0] position_count, // SIGNED position
    output reg         direction
);

    // Quadrature states
    reg [1:0] current_state  = 2'b00;
    reg [1:0] previous_state = 2'b00;

    // Synchronization / debounce
    reg [3:0] encoder_a_sync = 4'b0000;
    reg [3:0] encoder_b_sync = 4'b0000;
    wire encoder_a_debounced;
    wire encoder_b_debounced;

    assign encoder_a_debounced = encoder_a_sync[3];
    assign encoder_b_debounced = encoder_b_sync[3];

    // Input synchronization
    always @(posedge clk_50M) begin
        encoder_a_sync <= {encoder_a_sync[2:0], encoder_a};
        encoder_b_sync <= {encoder_b_sync[2:0], encoder_b};
    end

    // Quadrature decode
    always @(posedge clk_50M) begin
        if (!reset) begin
            position_count <= 20'sd0;
            direction      <= 1'b0;
            current_state  <= 2'b00;
            previous_state <= 2'b00;
        end else begin
            previous_state <= current_state;
            current_state  <= {encoder_a_debounced, encoder_b_debounced};

            if (current_state != previous_state) begin
                case ({previous_state, current_state})

                    // Forward
                    4'b0001, 4'b0111, 4'b1110, 4'b1000: begin
                        position_count <= position_count + 1'sd1;
                        direction <= 1'b1;
                    end

                    // Reverse
                    4'b0010, 4'b1011, 4'b1101, 4'b0100: begin
                        position_count <= position_count - 1'sd1;
                        direction <= 1'b0;
                    end

                    default: begin
                   // invalid transition --> ignore
                    end
                endcase
            end
        end
    end

endmodule