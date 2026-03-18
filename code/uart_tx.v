/*
Module UART Transmitter sends 8-bit data serially using UART at 115200 baud.
It adds start, parity, and stop bits and tells when the data is fully sent.
*/


module uart_tx(
    input clk_3125,
    input parity_type,tx_start,
    input [7:0] data,
    output reg tx, tx_done
);

initial begin
    tx = 1'b1;
    tx_done = 1'b0;
end
 
 
// FSM states
localparam	IDLE 			= 3'b000,
				SEND_START 	= 3'b001,
				SEND_BITS	= 3'b010,
				SEND_PARITY = 3'b011,
		      SEND_STOP	= 3'b100;
				
// Clock and baudrate parameters
localparam integer BAUD_TICKS = 27; // ≈ 3125kHz / 115200 baud

reg [2:0] state;              
reg [7:0] data_buf;           //stores data to be sent
reg [2:0] bit_index;          //selects which data bit to send	
reg [4:0] tick_count;         //counts clock cycles for baud timing
reg parity_bit;               //calculated parity bit (even/odd)

initial begin
    state = IDLE;
    data_buf   = 8'd0;
	 bit_index  = 3'd0;
	 parity_bit = 1'd0;
	 tick_count = 5'd0;
end

always @(posedge clk_3125) begin
	case(state)
		// IDLE: waiting for start signal
		IDLE: begin
			tx         <= 1'b1;                              // idle line high
			tx_done    <= 1'b0;
			tick_count <= 5'd0;
			if (tx_start) begin
				data_buf   <= data;
				parity_bit <= parity_type ? ~(^data) : ^data; // even/odd parity
				bit_index  <= 3'd0;
				tx         <= 1'b0;
				state      <= SEND_START;
			end 
		end
		
		// SEND_START: send start bit (0)
		SEND_START: begin
			tick_count <= tick_count + 5'd1;
			if (tick_count == BAUD_TICKS-1) begin
				tick_count <= 5'd0;
				state      <= SEND_BITS;
			end
		end
	 
	   // SEND_BITS: transmit 8 data bits (MSB first)
		SEND_BITS: begin
			tx         <= data_buf[bit_index];
         tick_count <= tick_count + 5'd1;
			if (tick_count == BAUD_TICKS-1) begin
				tick_count <= 5'd0;
				bit_index  <= bit_index + 3'd1;
				if (bit_index == 7) begin
					state <= SEND_PARITY;
				end
			end
		end
		
		// SEND_PARITY: send parity bit
		SEND_PARITY: begin
			 tx <= parity_bit;
			 tick_count <= tick_count + 5'd1;
			 if (tick_count == BAUD_TICKS-1) begin
				tick_count <= 5'd0;
				state      <= SEND_STOP;
			 end
		end
		
		// SEND_STOP: send stop bit (1)
		SEND_STOP: begin
			tx         <= 1'b1;
			tick_count <= tick_count + 5'd1;
			if (tick_count == BAUD_TICKS-1) begin
				tick_count <= 5'd0;
				tx_done    <= 1'b1;
				state      <= IDLE;
			end	
		end
		
      default: state <= IDLE;	
		
	endcase
end


endmodule