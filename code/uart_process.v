/*
 UART Process Module - Soil Sensing & Data Transmission
 Triggered by U-turn detection (MPI state): Servo down → capture min moisture & sensor values → servo up → transmit data over UART
 Uses latched sensor values from hold state: MPIM-1-#, MM-1-M/D-#, TH-1-XX-XX-# format
 Coordinates logged via simple change detection on x,y inputs
*/

`timescale 1ns / 1ps

module uart_process(
    input clk_50M,
    input clk_3125KHz,
    input reset,
    input mpi_trigger,           // Signal from MPI state to start soil sensing
    input exit,                  // Signal from movement_logic when maze is complete
    input uart_rx_in,
    input dout,                  // Moisture sensor data output
    inout dht_sensor,
    input [31:0] x, y,           // DFS1 coordinate inputs
    output adc_cs_n,             // Moisture sensor control
    output din,
    output adc_sck,
    output servo1_pwm,           // Servo control outputs
    output servo2_pwm,
    output uart_tx_out,
    output mpi_complete,         // Signal to MPI state when process is complete
    output debug_rx_start_cmd,   // Debug: HIGH when 'A' received
    output [7:0] debug_rx_byte   // Debug: last received byte
);


    // DHT Module Instantiation
    wire [7:0] T_integral, RH_integral, T_decimal, RH_decimal, Checksum;
    wire dht_valid; 
    
    dht dht0(
        .clk_50M(clk_50M),
        .reset(reset),
        .sensor(dht_sensor),
        .T_integral(T_integral),
        .RH_integral(RH_integral),
        .T_decimal(T_decimal),
        .RH_decimal(RH_decimal),
        .Checksum(Checksum),
        .data_valid(dht_valid)
    );
    
    // Moisture Sensor Module Instantiation
    wire [11:0] moisture_value;
    wire [7:0] moisture_led;
    
    moisture_sensor moisture_inst(
        .dout(dout),
        .clk50(clk_50M),
        .adc_cs_n(adc_cs_n),
        .din(din),
        .adc_sck(adc_sck),
        .d_out_ch0(moisture_value),
        .led_ind(moisture_led)
    );
    
    // Servo Control Signals
    reg servo_move_down = 1'b0;
    reg servo_move_up = 1'b0;
    
    // Servo Control Module Instantiation
    servo_control servo_inst(
        .clk_50M(clk_50M),
        .reset(reset),
        .move_down(servo_move_down),
        .move_up(servo_move_up),
        .servo1_pwm(servo1_pwm),
        .servo2_pwm(servo2_pwm)
    );
    
    // ASCII Decimal Encoders for Latched Sensor Values
    // (Only latched tens and ones are transmitted)
    wire [7:0] latched_t_int_t, latched_t_int_o;
    wire [7:0] latched_rh_int_t, latched_rh_int_o;
    
    // Encoders for latched sensor values (only tens and ones used)
    ascii_decimal ascii_latched_t_int(.value(latched_T_integral), .ascii_tens(latched_t_int_t), .ascii_ones(latched_t_int_o));
    ascii_decimal ascii_latched_rh_int(.value(latched_RH_integral), .ascii_tens(latched_rh_int_t), .ascii_ones(latched_rh_int_o));
    
    // Latched Moisture Sensor Classification
    // Stores minimum ADC value captured during hold state
    // Below 1330 = Moist (M), Above 1330 = Dry (D)
    
    // Latched sensor values (captured during hold state)
    reg [7:0] latched_T_integral = 0;
    reg [7:0] latched_RH_integral = 0;
    reg [11:0] latched_moisture_value = 12'hFFF;  // Initialize to max (4095) to find minimum
    reg latched_soil_moist = 0;

    // Synchronize mpi_trigger from movement_logic to clk_3125KHz
    reg mpi_trigger_meta = 1'b0;
    reg mpi_trigger_sync = 1'b0;
    reg mpi_trigger_last = 1'b0;
    
    always @(posedge clk_3125KHz) begin
        mpi_trigger_meta <= mpi_trigger;
        mpi_trigger_sync <= mpi_trigger_meta;
        mpi_trigger_last <= mpi_trigger_sync;
    end
    
    wire mpi_trigger_edge = mpi_trigger_sync && !mpi_trigger_last;
    reg mpi_done = 1'b0;
    assign mpi_complete = mpi_done;
    
    // Exit edge detection (similar to mpi_trigger)
    reg exit_meta = 1'b0;
    reg exit_sync = 1'b0;
    reg exit_last = 1'b0;
    
    always @(posedge clk_3125KHz) begin
        exit_meta <= exit;
        exit_sync <= exit_meta;
        exit_last <= exit_sync;
    end
    
    wire exit_edge = exit_sync && !exit_last;

    // FSM in clk_3125KHz domain 
    
    localparam S_IDLE = 4'd0, S_HOLD_SERVO = 4'd1, S_MPI = 4'd2, S_MOISTURE = 4'd3, S_TEMP_HUMID = 4'd4, S_COUNTER = 4'd5;
    localparam S_SEND_TRIGGER = 4'd6, S_SEND_START = 4'd7, S_SEND_WAIT = 4'd8, S_DONE = 4'd9, S_MAZE_END = 4'd10;
    localparam S_COORD_LOG = 4'd11;  // Coordinate logging state
    
    reg [3:0] state = S_IDLE;
    reg [7:0] tx_data = 8'd0;
    reg tx_start = 1'b0;
    reg [3:0] byte_counter = 0;
    
    // Coordinate tracking registers
    reg [31:0] prev_coord_x = 32'd0;
    reg [31:0] prev_coord_y = 32'd0;
    reg is_coord_packet = 1'b0;  // Flag: HIGH when sending coordinate packet
    reg is_maze_end_packet = 1'b0;  // Flag: HIGH when sending maze end packet

    
    // Packet buffers - variable length lines (max 14 bytes)
    reg [7:0] packet_line [0:3][0:13];  // 4 lines, up to 14 bytes each
    reg [1:0] line_counter = 0;  // which line to send
    reg [4:0] line_lengths [0:3];  // track length of each line
    
    // UART signals
    wire [7:0] rx_msg;
    wire rx_parity;
    wire rx_complete;
    wire rx_detect_A;      // 'A' detection signal from uart_rx module
    wire tx_done;
    wire parity_type = 1'b0;  // Even parity
    
    // Latch for 'A' detection - holds HIGH once rx_detect_A pulses
    reg rx_detect_A_latch = 1'b0;
    always @(posedge clk_3125KHz or negedge reset) begin
        if (!reset) begin
            rx_detect_A_latch <= 1'b0;
        end else if (rx_detect_A) begin
            rx_detect_A_latch <= 1'b1;  // Latch stays HIGH once set
        end
    end
    
    // MPI Position Mapping (uart_packet_manager style)
    localparam MPI_1 = 7'd45,
               MPI_2 = 7'd11,
               MPI_3 = 7'd19,
               MPI_4 = 7'd0,
               MPI_5 = 7'd64,
               MPI_6 = 7'd48,
               MPI_7 = 7'd81,
               MPI_8 = 7'd34,
               MPI_9 = 7'd5;
    
    // Calculate actual position from coordinates
    wire [6:0] actual_pos = y[6:0] * 7'd9 + x[6:0];
    
    // MPI ID selection based on current position
    reg [7:0] mpi_id;
    always @(*) begin
        case (actual_pos)
            MPI_1 : mpi_id = "1";
            MPI_2 : mpi_id = "2";
            MPI_3 : mpi_id = "3";
            MPI_4 : mpi_id = "4";
            MPI_5 : mpi_id = "5";
            MPI_6 : mpi_id = "6";
            MPI_7 : mpi_id = "7";
            MPI_8 : mpi_id = "8";
            MPI_9 : mpi_id = "9";
            default : mpi_id = "0";
        endcase
    end
    
    // Hold servo counter (10 seconds at clk_50M)
    reg [28:0] hold_servo_counter = 0;
    localparam HOLD_SERVO_TIME = 29'd250000000;  // 5 seconds at 50MHz (2 sec × 5)
    
    
    always @(posedge clk_3125KHz or negedge reset) begin
        if (!reset) begin
            state <= S_IDLE;
            tx_data <= 8'd0;
            tx_start <= 1'b0;
            byte_counter <= 0;
            line_counter <= 0;
        end
        else begin
            case (state)
                S_IDLE: begin
                    tx_start <= 1'b0;
                    servo_move_down <= 1'b0;
                    servo_move_up <= 1'b0;
                    hold_servo_counter <= 0;
                    latched_moisture_value <= 12'hFFF;  // Reset to max for next minimum capture
                    mpi_done <= 1'b0;
                    is_coord_packet <= 1'b0;  // Clear flag
                    is_maze_end_packet <= 1'b0;  // Clear flag
                    
                    // Detect coordinate changes
                    if (x != prev_coord_x || y != prev_coord_y) begin
                        prev_coord_x <= x;
                        prev_coord_y <= y;
                        state <= S_COORD_LOG;
                    end
                    else if (exit_edge) begin
                        // Exit signal received - prepare end message
                        state <= S_MAZE_END;
                    end
                    else if (mpi_trigger_edge) begin
                        // Start servo moving down on MPI trigger
                        servo_move_down <= 1'b1;
                        state <= S_HOLD_SERVO;
                    end
                end
                
                S_COORD_LOG: begin
                    // Prepare coordinate packet: "POS-X-Y-#\r\n" (e.g., "POS-3-7-#\r\n")
                    // X and Y are 0-8 (9x9 grid)
                    is_coord_packet <= 1'b1;  // Mark as coordinate packet
                    packet_line[0][0]  <= 8'h50;           // P
                    packet_line[0][1]  <= 8'h4F;           // O
                    packet_line[0][2]  <= 8'h53;           // S
                    packet_line[0][3]  <= 8'h2D;           // -
                    packet_line[0][4]  <= (x % 10) + 8'h30;  // X ones digit (use fresh x, will be latched this cycle)
                    packet_line[0][5]  <= 8'h2D;           // -
                    packet_line[0][6]  <= (y % 10) + 8'h30;  // Y ones digit (use fresh y, will be latched this cycle)
                    packet_line[0][7]  <= 8'h2D;           // -
                    packet_line[0][8]  <= 8'h23;           // # (hash character)
                    packet_line[0][9]  <= 8'h0D;           // CR
                    packet_line[0][10] <= 8'h0A;           // LF
                    
                    line_lengths[0] <= 5'd11;
                    line_counter <= 0;
                    byte_counter <= 0;
                    state <= S_SEND_TRIGGER;
                end
                
                S_HOLD_SERVO: begin
                    // Hold servo for 5 seconds (reading DHT and moisture sensor)
                    // Counter increments at clk_3125KHz, so divide by 16 to get clk_50M equivalent
                    // Increment by 16 per clk_3125KHz cycle to match 50MHz timing
                    hold_servo_counter <= hold_servo_counter + 28'd16;
                    
                    // Capture sensor values during hold state - store minimum moisture value
                    latched_T_integral <= T_integral;
                    latched_RH_integral <= RH_integral;
                    // Store minimum moisture value encountered during hold state
                    if (moisture_value < latched_moisture_value) begin
                        latched_moisture_value <= moisture_value;
                    end
                    latched_soil_moist <= (latched_moisture_value < 12'd1330);  // Classify based on latched minimum
                    
                    if (hold_servo_counter >= HOLD_SERVO_TIME) begin
                        hold_servo_counter <= 0;
                        state <= S_MPI;
                    end
                end
                
                S_MPI: begin
                    // Clear servo down signal and start moving servo up
                    servo_move_down <= 1'b0;
                    servo_move_up <= 1'b1;
                    
                    // Prepare line 0: "MPIM-<id>-X-Y-#\r\n" (includes coordinates)
                    packet_line[0][0]  <= 8'h4D;           // M
                    packet_line[0][1]  <= 8'h50;           // P
                    packet_line[0][2]  <= 8'h49;           // I
                    packet_line[0][3]  <= 8'h4D;           // M
                    packet_line[0][4]  <= 8'h2D;           // -
                    packet_line[0][5]  <= mpi_id;          // MPI ID (dynamic based on position)
                    packet_line[0][6]  <= 8'h2D;           // -
                    packet_line[0][7]  <= ((x[6:0] / 10) % 10) + "0";  // X tens
                    packet_line[0][8]  <= (x[6:0] % 10) + "0";         // X ones
                    packet_line[0][9]  <= 8'h2D;           // -
                    packet_line[0][10] <= ((y[6:0] / 10) % 10) + "0"; // Y tens
                    packet_line[0][11] <= (y[6:0] % 10) + "0";        // Y ones
                    packet_line[0][12] <= 8'h2D;           // -
                    packet_line[0][13] <= 8'h23;           // # (hash character)
                    line_lengths[0] <= 5'd14;
                    
                    line_counter <= 0;
                    byte_counter <= 0;
                    state <= S_SEND_TRIGGER;
                end
                
                S_MOISTURE: begin
                    // Start servo moving up while sending moisture
                    servo_move_up <= 1'b1;
                    
                    // Prepare line 1: "MM-<id>-M/D-#\r\n" (Moisture message - uses latched value)
                    packet_line[1][0]  <= 8'h4D;           // M
                    packet_line[1][1]  <= 8'h4D;           // M
                    packet_line[1][2]  <= 8'h2D;           // -
                    packet_line[1][3]  <= mpi_id;          // MPI ID (dynamic)
                    packet_line[1][4]  <= 8'h2D;           // -
                    packet_line[1][5]  <= latched_soil_moist ? 8'h4D : 8'h44;  // M or D (from latched value)
                    packet_line[1][6]  <= 8'h2D;           // -
                    packet_line[1][7]  <= 8'h23;           // # (hash character)
                    packet_line[1][8]  <= 8'h0D;           // CR
                    packet_line[1][9]  <= 8'h0A;           // LF
                    line_lengths[1] <= 4'd10;
                    
                    line_counter <= 1;
                    byte_counter <= 0;
                    state <= S_SEND_TRIGGER;
                end
                
                S_TEMP_HUMID: begin
                    // Prepare line 2: "TH-<id>-XX-XX-#\r\n" (Temperature-Humidity message using latched values)
                    packet_line[2][0]  <= 8'h54;           // T
                    packet_line[2][1]  <= 8'h48;           // H
                    packet_line[2][2]  <= 8'h2D;           // -
                    packet_line[2][3]  <= mpi_id;          // MPI ID (dynamic)
                    packet_line[2][4]  <= 8'h2D;           // -
                    packet_line[2][5]  <= latched_t_int_t;         // Temperature tens (from latched value)
                    packet_line[2][6]  <= latched_t_int_o;         // Temperature ones (from latched value)
                    packet_line[2][7]  <= 8'h2D;           // -
                    packet_line[2][8]  <= latched_rh_int_t;        // Humidity tens (from latched value)
                    packet_line[2][9]  <= latched_rh_int_o;        // Humidity ones (from latched value)
                    packet_line[2][10] <= 8'h2D;           // -
                    packet_line[2][11] <= 8'h23;           // # (hash character)
                    packet_line[2][12] <= 8'h0D;           // CR
                    packet_line[2][13] <= 8'h0A;           // LF
                    line_lengths[2] <= 4'd14;
                    
                    line_counter <= 2;
                    byte_counter <= 0;
                    state <= S_SEND_TRIGGER;
                end
                
                S_COUNTER: begin
                    // Final blank lines for spacing
                    packet_line[3][0]  <= 8'h0D;           // CR (extra line)
                    packet_line[3][1]  <= 8'h0A;           // LF
                    packet_line[3][2]  <= 8'h0D;           // CR
                    packet_line[3][3]  <= 8'h0A;           // LF
                    line_lengths[3] <= 4'd4;
                    
                    line_counter <= 3;
                    byte_counter <= 0;
                    state <= S_SEND_TRIGGER;
                end
                
                S_SEND_TRIGGER: begin
                    // Set data first, then tx_start in next state
                    tx_data <= packet_line[line_counter][byte_counter];
                    state <= S_SEND_START;
                end
                
                S_SEND_START: begin
                    // Assert tx_start for one cycle
                    tx_start <= 1'b1;
                    state <= S_SEND_WAIT;
                end
                
                S_SEND_WAIT: begin
                    // Clear tx_start and wait for tx_done
                    tx_start <= 1'b0;
                    
                    if (tx_done) begin
                        if (byte_counter < (line_lengths[line_counter] - 1)) begin
                            // Send next byte in same line
                            byte_counter <= byte_counter + 1;
                            state <= S_SEND_TRIGGER;
                        end
                        else begin
                            // Line complete, check packet type
                            if (is_coord_packet) begin
                                // Coordinate packet complete - return to IDLE
                                is_coord_packet <= 1'b0;
                                state <= S_IDLE;
                            end else if (is_maze_end_packet) begin
                                // Maze end packet complete - return to IDLE
                                is_maze_end_packet <= 1'b0;
                                state <= S_IDLE;
                            end else begin
                                // MPI packet - follow normal multi-line flow
                                case (line_counter)
                                    2'd0: state <= S_MOISTURE;
                                    2'd1: state <= S_TEMP_HUMID;
                                    2'd2: state <= S_COUNTER;
                                    2'd3: state <= S_DONE;
                                    default: state <= S_DONE;
                                endcase
                            end
                        end
                    end
                end
                
                S_DONE: begin
                    // Clear servo signals and signal MPI completion
                    servo_move_down <= 1'b0;
                    servo_move_up <= 1'b0;
                    mpi_done <= 1'b1;
                    
                    // Wait for MPI trigger to release before returning to IDLE
                    if (!mpi_trigger_sync) state <= S_IDLE;
                end
                
                S_MAZE_END: begin
                    // Prepare line for maze end message: "END-#\r\n"
                    is_maze_end_packet <= 1'b1;  // Mark as maze end packet
                    packet_line[0][0]  <= 8'h45;           // E
                    packet_line[0][1]  <= 8'h4E;           // N
                    packet_line[0][2]  <= 8'h44;           // D
                    packet_line[0][3]  <= 8'h2D;           // -
                    packet_line[0][4]  <= 8'h23;           // # (hash character)
                    packet_line[0][5]  <= 8'h0D;           // CR
                    packet_line[0][6]  <= 8'h0A;           // LF
                    
                    line_lengths[0] <= 4'd7;
                    line_counter <= 0;
                    byte_counter <= 0;
                    state <= S_SEND_TRIGGER;
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end
    
    // UART RX instantiation (receive from HC-05)
    uart_rx uart_rx0(
        .clk_3125(clk_3125KHz),
        .rx(uart_rx_in),
        .rx_msg(rx_msg),
        .rx_parity(rx_parity),
        .rx_complete(rx_complete),
        .rx_detect_A(rx_detect_A)
    );
    
    // UART TX instantiation (send to HC-05)
    uart_tx uart_tx0(
        .clk_3125(clk_3125KHz),
        .parity_type(parity_type),
        .tx_start(tx_start),
        .data(tx_data),
        .tx(uart_tx_out),
        .tx_done(tx_done)
    );

    // Debug outputs - wire directly from uart_rx module
    assign debug_rx_start_cmd = rx_detect_A_latch;  // Latched: stays HIGH once 'A' detected
    assign debug_rx_byte = rx_msg;                  // Current received byte from uart_rx

endmodule


