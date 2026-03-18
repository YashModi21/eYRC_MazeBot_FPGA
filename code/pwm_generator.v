`timescale 1ns / 1ps

/*
Module: pwm_generator generates pwm 
signals which are sent to the motor driver
to control speed of the motors.
*/



module pwm_generator(
    input clk_3125KHz,
    input [3:0] duty_cycle,
    output reg  pwm_signal
);

initial begin
     pwm_signal = 1;
end
//////////////////DO NOT MAKE ANY CHANGES ABOVE THIS LINE //////////////////

reg [2:0] counter = 0;
reg [3:0] counter_pwm = 0;


always @ (posedge clk_3125KHz) begin
	if (duty_cycle > counter_pwm)
	pwm_signal <= 1;
	else begin
	pwm_signal <= 0;
	end
	counter_pwm <= counter_pwm + 1;

end

//////////////////DO NOT MAKE ANY CHANGES BELOW THIS LINE //////////////////

endmodule
