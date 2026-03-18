`timescale 1ns / 1ps


/*
Module HC_SR04 Ultrasonic Sensor
This module will detect objects present in front 
of the range, and give the distance in mm.
*/

module ultrasonic(
    input clk_50M, echo_rx, 
    input reset,                    //active low reset
    output reg trig,
    output [15:0] distance_out 
);

initial begin
    trig = 0;
end

// Echo synchronizer (2-stage flip-flop chain for metastability protection)
reg echo_sync_0, echo_sync_1;

// FSM states
localparam INITIAL_DELAY = 3'b000;
localparam TRIG_HIGH     = 3'b001;
localparam WAIT_ECHO     = 3'b010;
localparam COUNT_ECHO    = 3'b011;
localparam NEXT          = 3'b100;


reg [2:0]  state = INITIAL_DELAY;
reg [21:0] counter = 0;
reg [21:0] echo_count = 0;                     
reg [31:0] calculated_distance = 0;

always @(posedge clk_50M or negedge reset) begin
    if (!reset) begin
        echo_sync_0 <= 1'b0;
        echo_sync_1 <= 1'b0;
        trig <= 0;
        calculated_distance <= 0;
        counter <= 0;
        echo_count <= 0;
        state <= INITIAL_DELAY;
    end
    else begin
        // Update echo synchronizer
        echo_sync_0 <= echo_rx;
        echo_sync_1 <= echo_sync_0;
        
        case(state)
            INITIAL_DELAY: begin
                trig <= 0;
                counter <= counter + 1;
                if (counter >= 50) begin
                    counter <= 0;
                    state <= TRIG_HIGH;
                end
            end

            TRIG_HIGH: begin                // Sends a short sound pulse
                trig <= 1;
                counter <= counter + 1;
                if (counter >= 500) begin
                    trig <= 0;
                    counter <= 0;
                    state <= WAIT_ECHO;
                end
            end

            WAIT_ECHO: begin               // Wait till echo is received
                counter <= counter + 1;
                if (echo_sync_1) begin
                    echo_count <= 0;
                    state <= COUNT_ECHO;
                end
                if (counter > 3000000) begin
                    // Timeout if no echo received within 60ms
                    calculated_distance <= 0;
                    echo_count <= 0;
                    counter <= 0;
                    state <= INITIAL_DELAY;
                end
            end

            COUNT_ECHO: begin              // Calculate distance using echo count
                counter <= counter + 1;
                if (echo_sync_1)
                    echo_count <= echo_count + 1;
                else begin
                    calculated_distance <= (echo_count >300) ? 34 * echo_count / 10000 : calculated_distance; // to remove errors caused by ultrasonic sensors when it gives 0 distance
                    state <= NEXT;
                end
            end

            NEXT: begin
                counter <= counter + 1;
                if (counter > 3000000) begin
                    counter <= 0;
                    state <= INITIAL_DELAY;
                end
            end
        endcase
    end
end

assign distance_out = calculated_distance[15:0];

endmodule