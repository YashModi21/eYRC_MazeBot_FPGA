/*
 ADC Controller Module
 Interfaces with MCP3004 8-channel ADC via SPI protocol
 Manages chip select, serial data input/output, and clock signals
 Returns 12-bit digital conversion result for selected channel (CH0)
*/

module adc_controller(
    input dout, adc_sck,
    output adc_cs_n, din, 
    output reg [11:0] d_out_ch0,
    output reg [7:0] led_ind
);
    parameter MIN = 600;
    parameter MAX = 1400;
    parameter STEP = (MAX - MIN) / 8;
/*
one read write cycle of adc is of 16 bits so we have named them 0, 2...15
out of that we have to update the address in MSB first format on 2, 3, 4 falling edges
we have to read from 4th to 15th rising edges
*/

    reg [3:0] din_counter = 0; // mod-15
    reg [3:0] sp_counter = 0;
    reg adc_cs = 1;
    reg din_temp = 0;       // default 0 since we are reading from adc channel 0
    reg [11:0] dout_chx = 0;
    reg adc_clk_reg = 0;
	 
    // data writing on negedge.
    always @(negedge adc_sck) begin
        din_counter <= din_counter + 1;
        // chip select
        if(din_counter == 0) begin
            adc_cs <= !adc_cs;
        end

        // adc channel selection. if all 0 -> channel 0, pin 24
        // case(din_counter)
        //     2: din_temp = 0; //ADD2
        //     3: din_temp = 0; //ADD1
        //     4: din_temp = 0; //ADD0
        //     default: din_temp = 0;
        // endcase
    end

    always @(posedge adc_sck) begin
        // read the adc value in between 4th sclk cycle and 15th sclk cylce.
        if((sp_counter >= 4) && (sp_counter <= 15)) begin
            dout_chx[15 - sp_counter] <= dout; // fill in the data
        end else begin
            dout_chx <= 0; // reset the dout_chx
        end
        sp_counter <= sp_counter + 1'b1;
    end

    always @(posedge adc_sck ) begin
        if ((sp_counter == 15)&& (!adc_cs)) begin
            d_out_ch0 <= dout_chx;
        end
    end

    // // led always block
    // always @(d_out_ch0) begin
    //     if (d_out_ch0 < MIN) begin
    //         led_ind = 8'b0000_0001;
    //     end else if (d_out_ch0 >= MAX) begin
    //         led_ind = 8'b1111_1111;
    //     end else begin
    //         led_ind = 8'b01111_1111 >> MAX-((d_out_ch0 - MIN) / STEP);
    //     end
    // end 
    always @(*) begin
        integer count;

        if (d_out_ch0 < MIN)
            count = 1;
        else if (d_out_ch0 >= MAX)
            count = 8;
        else
            count = (d_out_ch0 - MIN) / STEP + 1;

        led_ind = (8'hFF >> (8 - count));  // generates 0000_0001 → 1111_1111
    end


    // output.
    // assign d_out_ch0 = dout_chx;        // since our adc_sclk is a continous clock, won't the values in here keep changing? should we only assign dout_chx to d_out_ch0 when the sp_counter is max/done or adc_cs is high
    assign adc_cs_n = adc_cs;
    assign din = din_temp;

endmodule