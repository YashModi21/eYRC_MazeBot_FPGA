`timescale 1ns / 1ps

/*
Module: Motor Driver L298N Controller
This module generates PWM and direction control signals for the L298N motor driver
Supports differential speed control for precise wall-following alignment
*/

module motor_driver_l298n (
    input clk_3125KHz,
    input [2:0] motor_command,      // Command type
    input [3:0] speed_left,         // PWM duty cycle for left motor (0-15)
    input [3:0] speed_right,        // PWM duty cycle for right motor (0-15)
    output pwm_left_motor,          // ENA - PWM signal for left motor
    output pwm_right_motor,         // ENB - PWM signal for right motor
    output reg in1,                 // IN1 - Direction control for left motor
    output reg in2,                 // IN2 - Direction control for left motor
    output reg in3,                 // IN3 - Direction control for right motor
    output reg in4                  // IN4 - Direction control for right motor
);

// Motor commands
localparam STOP        = 3'b000;
localparam FORWARD     = 3'b001;
localparam BACKWARD    = 3'b010;
localparam LEFT_TURN   = 3'b011;
localparam RIGHT_TURN  = 3'b100;

//debug speed
wire [3:0] left_motor_debug = speed_left;
wire [3:0] right_motor_debug = speed_right;

// PWM generators for left motor for speed control 
pwm_generator pwm_left (
    .clk_3125KHz(clk_3125KHz),
    .duty_cycle(speed_left),
    .pwm_signal(pwm_left_motor)
);

// PWM generators for right motor for speed control
pwm_generator pwm_right (
    .clk_3125KHz(clk_3125KHz),
    .duty_cycle(speed_right),
    .pwm_signal(pwm_right_motor)
);

always @(posedge clk_3125KHz) begin
    case(motor_command)
        STOP: begin
            in1 <= 0;
            in2 <= 0;
            in3 <= 0;
            in4 <= 0;
        end
        
        FORWARD: begin
            in1 <= 1;
            in2 <= 0;
            in3 <= 1;
            in4 <= 0;
        end
        
        BACKWARD: begin
            in1 <= 0;
            in2 <= 1;
            in3 <= 0;
            in4 <= 1;
        end
        
        LEFT_TURN: begin
            in1 <= 0;
            in2 <= 1;
            in3 <= 1;
            in4 <= 0;
        end
        
        RIGHT_TURN: begin
            in1 <= 1;
            in2 <= 0;
            in3 <= 0;
            in4 <= 1;
        end
        
        default: begin
            in1 <= 0;
            in2 <= 0;
            in3 <= 0;
            in4 <= 0;
        end
    endcase
end

endmodule