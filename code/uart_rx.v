`timescale 1ns / 1ps

module uart_rx(
    input clk_3125,
    input rx,
    output reg [7:0] rx_msg,
    output reg rx_parity,
    output reg rx_complete,
    output reg rx_detect_A    // HIGH when 'A' (0x41) is received
);

initial begin
    rx_msg = 8'b0;
    rx_parity = 1'b0;
    rx_complete = 1'b0;
    rx_detect_A = 1'b0;
end

    localparam IDLE  = 2'd0;
    localparam START = 2'd1;
    localparam DATA  = 2'd2;
    localparam STOP  = 2'd3;

    reg [5:0] counter = 0;
    reg [3:0] bit_count = 0;
    reg [1:0] state = IDLE;

    always @(posedge clk_3125) begin
        case (state)

            IDLE: begin
                rx_complete <= 1'b0;
                counter <= 0;
                bit_count <= 0;
                if (rx == 1'b0) begin
                    state <= START;
                end
            end

            START: begin
                if (counter == 6'd13) begin
                    if (rx == 1'b0) begin
                        counter <= 0;
                        state <= DATA;
                    end
                    else begin
                        state <= IDLE;
                    end
                end
                else begin
                    counter <= counter + 1;
                end
            end

            DATA: begin
                if (counter == 6'd26) begin
                    counter <= 0;
                    rx_msg[bit_count] <= rx;

                    if (bit_count == 4'd7) begin
                        state <= STOP;
                    end
                    else begin
                        bit_count <= bit_count + 1;
                    end
                end
                else begin
                    counter <= counter + 1;
                end
            end

            STOP: begin
                if (counter == 6'd26) begin
                    rx_complete <= 1'b1;
                    rx_parity <= ^rx_msg;
                    // Detect if received byte is 'A' (0x41)
                    rx_detect_A <= (rx_msg == 8'h41) ? 1'b1 : 1'b0;
                    state <= IDLE;
                end
                else begin
                    counter <= counter + 1;
                end
            end

        endcase
    end

endmodule