/*
 Moisture Sensor Interface Module
 Generates ADC serial clock by dividing the 50MHz input clock by 8 (creates ~6.25MHz SPI clock)
 Instantiates ADC controller to read analog moisture sensor values from MCP3004 channel 0
 Provides 12-bit digital output representing soil moisture level (0-4095 ADC counts)
*/

module moisture_sensor(
    input dout, clk50,
    output adc_cs_n, din, adc_sck,
    output [11:0] d_out_ch0,
    output [7:0] led_ind
);


reg [3:0] counter;

always @(posedge clk50) begin
    counter <= counter + 1;
end

assign adc_sck = counter[3]; // dividing clk50 by 8 to get adc_sck

adc_controller adc_inst(
    .dout(dout),
    .adc_sck(adc_sck),
    .adc_cs_n(adc_cs_n),
    .din(din),
    .d_out_ch0(d_out_ch0),
    .led_ind(led_ind)
);

endmodule