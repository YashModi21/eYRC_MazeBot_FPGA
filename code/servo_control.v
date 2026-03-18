`timescale 1ns / 1ps

/*
    Servo Control Module
    This module controls two servos to move the soil moisture mechanism in a coordinated manner.
    The servos are controlled via PWM signals generated based on specified pulse widths.
    The module takes input signals to initiate downward and upward movements.
*/

module servo_control (
    input wire clk_50M,
    input wire reset,
    input wire move_down,      // Signal to move servo down
    input wire move_up,        // Signal to move servo up
    output reg servo1_pwm,
    output reg servo2_pwm
);

    // Servo positions (PWM pulse widths in cycles)
    parameter S1_INITIAL = 20'd90000;
    parameter S1_TARGET  = 20'd40000;
    
    parameter S2_INITIAL = 20'd155000;
    parameter S2_TARGET1 = 20'd80000; // Mid position for servo 2 
    parameter S2_TARGET2 = 20'd40000; // Final position for servo 2

    // Time configuration for PWM period
    parameter PWM_PERIOD = 20'd1000000;   // 20ms
    
    // Movement Speed
    parameter MOVE_UPDATE_RATE = 20'd500000; // Update every 10ms
    parameter STEP_SIZE        = 20'd1000;   // Change width by 1000

    // State Definitions
    localparam ST_IDLE      = 3'd0; // Idle
    localparam ST_MOVE_DOWN = 3'd1; // Move down
    localparam ST_MOVE_UP   = 3'd2; // Move up

    reg [19:0] pwm_counter;
    reg [19:0] s1_current;
    reg [19:0] s2_current;
    
    reg [27:0] timer;
    reg [2:0]  state;

    always @(posedge clk_50M) begin
        if (~reset) begin
            s1_current <= S1_INITIAL;
            s2_current <= S2_INITIAL;
            pwm_counter <= 0;
            timer <= 0;
            state <= ST_IDLE;
            servo1_pwm <= 0;
            servo2_pwm <= 0;
        end else begin
            
            // PWM Generator
            if (pwm_counter >= PWM_PERIOD - 1)
                pwm_counter <= 0;
            else
                pwm_counter <= pwm_counter + 1;

            // Output Logic
            servo1_pwm <= (pwm_counter < s1_current);
            servo2_pwm <= (pwm_counter < s2_current);

            // State Machine
            case (state)
                
                ST_IDLE: begin
                    s1_current <= S1_INITIAL;
                    s2_current <= S2_INITIAL;
                    timer <= 0;
                    if (move_down) begin
                        state <= ST_MOVE_DOWN;
                    end
                end

                // Move servo down to target position to record soil moisture
                ST_MOVE_DOWN: begin
                    if (timer >= MOVE_UPDATE_RATE) begin
                        timer <= 0;
                        
                        // Servo 2 moves first to the mid position 
                        if (s2_current > S2_TARGET1) begin
                            s2_current <= s2_current - STEP_SIZE;
                        end
                        // Servo 1 moves second to target position (down)
                        else if (s1_current > S1_TARGET) begin
                            s1_current <= s1_current - STEP_SIZE;
                        end
                        // Servo 2 moves to final target to take the measurement
                        else if (s2_current > S2_TARGET2) begin
                            s2_current <= s2_current - STEP_SIZE;
                        end
                        // All done moving down
                        else begin
                            if (move_up) begin
                                state <= ST_MOVE_UP;
                            end
                        end
                    end else begin
                        timer <= timer + 1;
                    end
                end

                // Move servo up back to initial position
                ST_MOVE_UP: begin
                    if (timer >= MOVE_UPDATE_RATE) begin
                        timer <= 0;
                        //servo 2 moves first to mid position
                        if (s2_current < S2_TARGET1) begin
                            s2_current <= s2_current + STEP_SIZE;
                        end
                        // Servo 1 moves up to its initial position
                        else if (s1_current < S1_INITIAL) begin
                            s1_current <= s1_current + STEP_SIZE;
                        end
                        // Then servo 2 moves to its initial position 
                        else if (s2_current < S2_INITIAL) begin
                            s2_current <= s2_current + STEP_SIZE;
                        end
                        // All done moving up
                        else begin
                            state <= ST_IDLE;
                        end
                    end else begin
                        timer <= timer + 1;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule