/*
 DHT Temperature & Humidity Sensor Module (DHT11/DHT22)
 Performs continuous polling via 1-wire protocol: host pull-down → sensor response → 40-bit data transmission
 Extracts temperature (integral + decimal) and humidity (integral + decimal) with checksum validation
 Outputs data_valid flag when new reading is complete; runs asynchronously at clk_50M
*/

module dht(
    input clk_50M,
    input reset,
    inout sensor,

    output reg [7:0] T_integral,
    output reg [7:0] RH_integral,
    output reg [7:0] T_decimal,
    output reg [7:0] RH_decimal,
    output reg [7:0] Checksum,
    output reg data_valid
);

// ----------------------------------------------------
// Bidirectional I/O and Synchronization
// ----------------------------------------------------
reg sensor_out;
reg sensor_dir;    // 1 = drive, 0 = receive
assign sensor = sensor_dir ? sensor_out : 1'bz;

// Three-stage synchronizer to prevent metastability
reg [2:0] sync_chain;
always @(posedge clk_50M) sync_chain <= {sync_chain[1:0], sensor};
wire s_safe = sync_chain[2];

// Edge detection logic
reg s_prev;
always @(posedge clk_50M) s_prev <= s_safe;

wire neg_edge = (s_prev == 1'b1 && s_safe == 1'b0);
wire pos_edge = (s_prev == 1'b0 && s_safe == 1'b1);

// ----------------------------------------------------
// State Machine Definitions
// ----------------------------------------------------
localparam IDLE           = 3'd0;
localparam START          = 3'd1;
localparam WAIT_RESP_LOW  = 3'd2;
localparam WAIT_RESP_HIGH = 3'd3;
localparam READ_LOW       = 3'd4;
localparam READ_HIGH      = 3'd5;
localparam CHECK          = 3'd6;
localparam DONE           = 3'd7;

reg [2:0] state;
reg [25:0] cnt;    // 26 bits required for 1s delay
reg [5:0] bit_index;
reg [39:0] data_out;

// Checksum logic
wire [7:0] calc_sum = data_out[39:32] + data_out[31:24] + data_out[23:16] + data_out[15:8];

// ----------------------------------------------------
// RESET + MAIN FSM
// ----------------------------------------------------
always @(posedge clk_50M) begin
    if (!reset) begin
        state       <= IDLE;
        sensor_dir  <= 1'b0;
        sensor_out  <= 1'b1;
        cnt         <= 26'd0;
        bit_index   <= 6'd0;
        data_out    <= 40'd0;
        data_valid  <= 1'b0;
    end else begin
	 
	 
	     if (state != IDLE && cnt > 26'd1000000) begin
                state <= IDLE;
                cnt <= 26'd0;
                sensor_dir <= 1'b0;
            end
				
				
        case(state)

        // 1s stabilization + 18ms Start Pulse
        IDLE: begin
            data_valid <= 1'b0;
            cnt <= cnt + 26'd1;
				
				if (cnt < 26'd50000000) begin
					  // -------- 1 second gap --------
					  sensor_dir <= 1'b0;   // release bus (Hi-Z)
					  sensor_out <= 1'b1;   // don't care
            end
				else begin
						sensor_dir <= 1'b1;
                  sensor_out <= 1'b0;   // drive low
					   if (cnt > 26'd50950000) begin   // >18 ms
						    sensor_dir <= 1'b0;   // release bus (Hi-Z)
							 sensor_out <= 1'b1;   // don't care
							 cnt <= 26'd0;
							 state <= START;
                  end
						
				end
        end

        // Wait for the DHT11 to pull the line LOW
        START: begin
		      cnt <= cnt + 26'd1;
            if (neg_edge) begin
                cnt <= 26'd0;
                state <= WAIT_RESP_LOW;
            end
        end

        // Wait for 80us LOW to end (positive edge)
        WAIT_RESP_LOW: begin
		      cnt <= cnt + 26'd1;
            if (pos_edge) begin
                cnt <= 26'd0;
                state <= WAIT_RESP_HIGH;
            end
        end

        // Wait for 80us HIGH to end (negative edge)
        WAIT_RESP_HIGH: begin
		      cnt <= cnt + 26'd1;
            if (neg_edge) begin
                cnt <= 26'd0;
                bit_index <= 6'd0;
                state <= READ_LOW;
            end
        end

        // Wait for start of bit data pulse
        READ_LOW: begin
		      cnt <= cnt + 26'd1;
            if (pos_edge) begin
                cnt <= 26'd0;
                state <= READ_HIGH;
            end
        end

        // Measure HIGH width to determine 0 or 1
        READ_HIGH: begin
            cnt <= cnt + 26'd1;
            if (neg_edge) begin
                // 50us (2500 cycles) is the optimal threshold
                data_out[39-bit_index] <= (cnt > 26'd2500) ? 1'b1 : 1'b0;
                bit_index <= bit_index + 6'd1;

                if (bit_index >= 6'd39) begin // Check after 40 bits
					     cnt <= 26'd0;
                    state <= CHECK;
					 end	  
                else begin
					     cnt <= 26'd0;
                    state <= READ_LOW;
					 end	  
            end
        end

        // Implement the 50us End Signal from datasheet
        CHECK: begin
            cnt <= cnt + 26'd1;
            if (cnt >= 26'd3000) begin // ~60us buffer
                state <= DONE;
            end
        end

        DONE: begin
            RH_integral <= data_out[39:32];
            RH_decimal  <= data_out[31:24];
            T_integral  <= data_out[23:16];
            T_decimal   <= data_out[15:8]; 
            Checksum    <= data_out[7:0];  

            data_valid <= (calc_sum == data_out[7:0]);
            
            state <= IDLE; // Loop back to start 1s gap
            cnt <= 26'd0;
        end

        endcase
    end
end

endmodule
