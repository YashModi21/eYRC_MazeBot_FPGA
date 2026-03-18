`timescale 1ns / 1ps

// ultrasonic_refined.v
// Instantiates existing `ultrasonic` module and applies a 5-sample median filter

module ultrasonic_refined(
    input clk_50M,
    input echo_rx,
    input reset, // active low
    output trig,
    output [15:0] distance_out
);

    // raw distance from existing ultrasonic module
    wire [15:0] raw_distance;

    // instantiate original ultrasonic (includes built-in echo synchronizer)
    ultrasonic u0(
        .clk_50M(clk_50M),
        .echo_rx(echo_rx),
        .reset(reset),
        .trig(trig),
        .distance_out(raw_distance)
    );

    // 5-sample shift registers
    reg [15:0] s0, s1, s2, s3, s4;
    reg [15:0] prev_raw;
    reg [31:0] timeout_counter;
    localparam TIMEOUT_THRESHOLD = 32'd100000000; // ~2 seconds at 50 MHz

    // update shift register when a new raw sample is seen
    // also track timeout if no new samples arrive
    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            s0 <= 16'd0;
            s1 <= 16'd0;
            s2 <= 16'd0;
            s3 <= 16'd0;
            s4 <= 16'd0;
            prev_raw <= 16'd0;
            timeout_counter <= 32'd0;
        end else begin
            if (raw_distance != prev_raw) begin
                // New sample arrived, reset timeout and shift
                prev_raw <= raw_distance;
                s4 <= s3;
                s3 <= s2;
                s2 <= s1;
                s1 <= s0;
                s0 <= raw_distance;
                timeout_counter <= 32'd0;
            end else begin
                // No new sample, increment timeout counter
                if (timeout_counter < TIMEOUT_THRESHOLD) begin
                    timeout_counter <= timeout_counter + 1;
                end
            end
        end
    end

    wire [15:0] median_distance;

    median_filter_5 mf(
        .d0(s0),
        .d1(s1),
        .d2(s2),
        .d3(s3),
        .d4(s4),
        .median(median_distance)
    );

    // Output registration: register median output and handle timeout
    // On timeout (no new sample for ~2s), hold last valid value
    reg [15:0] distance_out_reg;

    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            distance_out_reg <= 16'd0;
        end else begin
            // Update output with median, or hold on timeout
            if (timeout_counter >= TIMEOUT_THRESHOLD) begin
                // Timeout: hold previous output
                distance_out_reg <= distance_out_reg;
            end else begin
                // Normal operation: output registered median
                distance_out_reg <= median_distance;
            end
        end
    end

    assign distance_out = distance_out_reg;

endmodule
