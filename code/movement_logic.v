`timescale 1ns / 1ps
/*
Module: Movement Logic Controller

- Movement controller FSM for a maze-solving / wall-following robot.

Inputs used:
1) IR Sensors (active-low, debounced):
   - ir_left, ir_right, ir_front  -> wall presence detection

2) Ultrasonic Distance Sensors (mm):
   - distance_left, distance_right, distance_front -> alignment + obstacle check

3) Wheel Encoders:
   - left_encoder_count, right_encoder_count -> forward distance + turn angle control

Outputs:
- motor_command (STOP / FORWARD / LEFT TURN / RIGHT TURN)
- speed_left, speed_right (PWM duty values for motor speed control)
- state_debug, turn_debug, op_debug for debugging

Core Working:
- Uses a main FSM (STOP_STATE, MOVE_STATE, LEFT_TURN, RIGHT_TURN, UTURN, FORWARD, WALL_FOLLOWING)
- Turns are performed in 3 phases using a sub-FSM (FORWARD_BEFORE_TURN -> TURN -> FORWARD_AFTER_TURN)
- Wall-following adjusts motor speeds using ultrasonic distance imbalance for better alignment.

--> Module is shortened for the purpose of demonstrating soil mechanism and uart process, adding a MPI state, to initiate soil sensing, temperature and humidity sensing and uart transmission
	
*/


module movement_logic (
    input clk_50M,
    input clk_3125KHz,
    input reset,
    
    // IR sensor inputs (wall presence detection)
    input ir_left,              // Left IR sensor
    input ir_right,             // Right IR sensor
	input ir_front,             // Front IR sensor
    
    // Ultrasonic sensor distance inputs (in mm) for alignment
    input [15:0] distance_left,
    input [15:0] distance_right,
    input [15:0] distance_front,
    
    // Encoder inputs for turning
    input [19:0] left_encoder_count,
    input [19:0] right_encoder_count,
    
    // Motor control outputs
    output reg [2:0] motor_command,// command to motor driver
    output reg [3:0] speed_left,   // left motor speed
    output reg [3:0] speed_right,  // right motor speed
    
    // Soil sensing and UART control
    output reg mpi_trigger,         // Signal to trigger MPI soil sensing
    output reg exit_maze,           // Signal when maze is complete (STOP_STATE reached)
    input mpi_complete,             // Signal from MPI when sensing is complete
    input debug_rx_start_cmd,       // 'A' command received from UART - trigger movement
    
    // Debug outputs
    output reg [3:0] state_debug,
	output reg [2:0] turn_debug,
	output op_debug,
	output [3:0] debug_dfs_pos_x,  // Debug: DFS X coordinate
	output [3:0] debug_dfs_pos_y   // Debug: DFS Y coordinate
);

// State definitions
localparam STOP_STATE     = 4'b0000;
localparam MOVE_STATE     = 4'b0001;
localparam LEFT_TURN      = 4'b0010;
localparam RIGHT_TURN     = 4'b0011;
localparam UTURN          = 4'b0100;
localparam FORWARD        = 4'b0101;
localparam WALL_FOLLOWING = 4'b0110;

// Turn state definitions
localparam FORWARD_BEFORE_TURN = 3'b000;
localparam TURN                = 3'b001;
localparam MPI                 = 3'b010;
localparam FORWARD_AFTER_TURN  = 3'b011;

// Motor commands
localparam MOTOR_STOP        = 3'b000;
localparam MOTOR_FORWARD     = 3'b001;
localparam MOTOR_BACKWARD    = 3'b010;
localparam MOTOR_LEFT_TURN   = 3'b011;
localparam MOTOR_RIGHT_TURN  = 3'b100;

// Thresholds and parameters                      
localparam FRONT_THRESHOLD        = 16'd260;    // threshold for front wall detection 
localparam BEFORE_TURN_COUNT      = 16'd3250;   // encoder count before turning
localparam AFTER_TURN_COUNT       = 16'd2350;   // encoder count after turning
localparam TURN_90_COUNT          = 16'd1900;   // encoder count to measure turning     
localparam TURN_180_COUNT         = 16'd3225;   //encoder count for u-turn
localparam FORWARD_COUNT          = 16'd4650;   // encoder count for straight movement of one cell 
localparam SAFETY_THRESHOLD       = 16'd80;     // ultrasonic sensor safety stop
localparam OPEN_FRONT_BEFORE_CORRECTION  = 16'd500;    // correction for open path turns (experimentally determined)
localparam OPEN_SIDE_BEFORE_CORRECTION   = 16'd900;    // correction for open side turns (experimentally determined)
localparam OPEN_FRONT_AFTER_CORRECTION   = 16'd300;    // correction for forward after turn when front is open	
localparam OPEN_SIDE_AFTER_CORRECTION    = 16'd600;    // correction for forward after turn when side is open
localparam SIDE_OPEN_TURN_CORRECTION = 16'd450;   // correction for turn angle when side is open
localparam AFTER_UTURN_DISTANCE = 16'd1700; // distance to move forward after UTURN

// State and turn counters
reg [3:0]  current_state = STOP_STATE;
reg [29:0] state_counter = 0;
reg [2:0]  turn_state = FORWARD_BEFORE_TURN;
reg [19:0] left_enc_start = 0;
reg [19:0] right_enc_start = 0;
reg [7:0]  left_turn_count = 0 ;
reg [7:0]  right_turn_count = 0;
reg mpi_detected = 0;  // Flag for MPI wall detection during UTURN FORWARD_BEFORE_TURN
reg uturn_flag = 0;
reg front_flag = 0;
reg side_flag =0;
reg [3:0] mpi_count = 0;  // Counter for MPI sensing events

// IR state tracking for change detection
reg ref_ir_left_debounced = 0;
reg ref_ir_right_debounced = 0;
reg ref_ir_front_debounced = 0;
wire ir_state_changed;  // Flag when any IR debounced value changes

// Debounce IR sensors which indicate presence of wall
reg ir_left_debounced = 0;
reg ir_right_debounced = 0;
reg ir_front_debounced = 0;
reg [19:0] ir_left_counter = 0;
reg [19:0] ir_right_counter = 0;
reg [19:0] ir_front_counter = 0;
localparam DEBOUNCE_THRESHOLD = 20'd100000;  // ~2ms debounce
localparam AVG_DISTANCE = 9'd200;
localparam TILE_DISTANCE = 20'd4960;  // One cell forward movement
localparam TILE_TOLERANCE = 20'd500;
localparam TURN_SPEED = 4'd15;
localparam DIST_BUFFER = 1000;       // buffer for encoder stability
localparam TIME_BUFFER = 40000000;   // time buffer (0.8s) for sensor stabilization


// Wall detection flags
wire wall_front_detected = (distance_front < FRONT_THRESHOLD && distance_front > 4) ? 1 : 0;
wire side_open = (distance_left >= 175 && distance_right >= 175) ? 1 : 0;
assign op_debug = wall_front_detected;
wire front_open = !wall_front_detected;
wire ultrasonic_safety_stop = (distance_front <= SAFETY_THRESHOLD ) ? 1 : 0;

// Speed 
wire [3:0] speed_left_align, speed_right_align, speed_leftturn_left, speed_leftturn_right, speed_rightturn_left, speed_rightturn_right, wf_left, wf_right;

//Apparent values to implement wall following during turns ( experimentally calculated , sum of distances calculated by both ir is 145mm
wire [15:0] apparent_left = (200 - 45- distance_right)>0 ? (200 - 45 - distance_right) : distance_right;
wire [15:0] apparent_right = (200 - 45 - distance_left)>0  ? (200 - 45 - distance_left) : distance_left;

//PWM values for wall following
// assign speed_left_align = (distance_left < AVG_DISTANCE ) ? (15 - distance_left * 15/AVG_DISTANCE) : 6;
// assign speed_right_align = (distance_right < AVG_DISTANCE ) ? (15 - distance_right * 15/AVG_DISTANCE) : 6;

// //PWM values for left turn using apparent wall logic
// assign speed_leftturn_left = (apparent_left < AVG_DISTANCE) ? (15 - apparent_left * 15/AVG_DISTANCE) : 6;
// assign speed_leftturn_right= (distance_right < AVG_DISTANCE) ? (15 - distance_right * 15/AVG_DISTANCE) : 6;

// //PWM values for right turn using apparent wall logic
// assign speed_rightturn_left = (distance_left < AVG_DISTANCE) ? (15 - distance_left * 15/AVG_DISTANCE) : 6;
// assign speed_rightturn_right= (apparent_right < AVG_DISTANCE) ? (15 - apparent_right * 15/AVG_DISTANCE) : 6;

// IR state change detection
assign ir_state_changed = (ref_ir_left_debounced != ir_left_debounced || 
                           ref_ir_right_debounced != ir_right_debounced || 
                           ref_ir_front_debounced != ir_front_debounced);

//assign wf_left = (distance_left < AVG_DISTANCE ) ? (16 - distance_left * 15/AVG_DISTANCE) : (distance_left >= AVG_DISTANCE && distance_right >= AVG_DISTANCE) ? TURN_SPEED : (wf_left> 15) ? 15 : (16 - apparent_left * 15/AVG_DISTANCE) ;
//assign wf_right = (distance_right < AVG_DISTANCE ) ? (16 - distance_right * 15/AVG_DISTANCE) : (distance_left >= AVG_DISTANCE && distance_right >= AVG_DISTANCE) ? TURN_SPEED : (wf_right> 15) ? 15 : (16 - apparent_right * 15/AVG_DISTANCE) ;
	
assign wf_left =
    (distance_left < (AVG_DISTANCE / 3)) ? 15 :
    (distance_left < AVG_DISTANCE) ?
        (18 - distance_left * 15 / AVG_DISTANCE) :
    (distance_left >= AVG_DISTANCE && distance_right >= AVG_DISTANCE) ?
        TURN_SPEED :
    ((18 - apparent_left * 15 / AVG_DISTANCE) > 15) ?
        15 :
        (18 - apparent_left * 15 / AVG_DISTANCE);


assign wf_right =
    (distance_right < (AVG_DISTANCE / 3)) ? 15 :
    (distance_right < AVG_DISTANCE) ?
        (18 - distance_right * 15 / AVG_DISTANCE) :
    (distance_left >= AVG_DISTANCE && distance_right >= AVG_DISTANCE) ?
        TURN_SPEED :
    ((18 - apparent_right * 15 / AVG_DISTANCE) > 15) ?
        15 :
        (18 - apparent_right * 15 / AVG_DISTANCE);
	
// Encoder difference
wire [19:0] left_delta = ((left_encoder_count - left_enc_start)>0) ? left_encoder_count - left_enc_start : 0;
wire [19:0] right_delta = ((right_encoder_count - right_enc_start)>0) ? right_encoder_count - right_enc_start : 0;

	// if ir_debounced value = 1 that implies wall detected
	// Debounce IR sensors
	always @(posedge clk_50M) begin
		 // Debounce left IR sensor
		 if (!ir_left) begin
			  if (ir_left_counter < DEBOUNCE_THRESHOLD) begin
					ir_left_counter <= ir_left_counter + 1;
			  end else begin
					ir_left_debounced <= 1;
			  end
		 end else begin
			  ir_left_counter <= 0;
			  ir_left_debounced <= 0;
		 end
		 
		 // Debounce right IR sensor
		 if (!ir_right) begin
			  if (ir_right_counter < DEBOUNCE_THRESHOLD) begin
					ir_right_counter <= ir_right_counter + 1;
			  end else begin
					ir_right_debounced <= 1;
			  end
		 end else begin
			  ir_right_counter <= 0;
			  ir_right_debounced <= 0;
		 end
		 
		 // Debounce front
		 if (!ir_front) begin
			 if (ir_front_counter < DEBOUNCE_THRESHOLD) begin
					ir_front_counter <= ir_front_counter + 1;
			  end else begin
					ir_front_debounced <= 1;
			  end
		 end else begin
			  ir_front_counter <= 0;
			  ir_front_debounced <= 0;
		 end
	end

// DFS Module Instantiation
wire [2:0] dfs_move;        // DFS output: movement decision
wire [3:0] dfs_pos_x;       // DFS debug: X coordinate
wire [3:0] dfs_pos_y;       // DFS debug: Y coordinate
wire [31:0] dfs_x, dfs_y;   // DFS1 coordinate outputs
wire dfs_back_tracking;     // Backtracking flag from dfs1
reg dfs_clk = 0;            // DFS clock - toggled in MOVE_STATE
reg [31:0] dfs_clk_count = 0;    // Counter for DFS clock toggles (0-3)
reg [31:0] dfs_time_count = 0;   // Time counter for sensor stabilization
reg dfs_decision_valid = 0;  // Flag when DFS decision is ready
reg [2:0] dfs_move_latched; // Latched DFS decision

// Encoder difference calculation for MOVE_STATE buffer logic
wire [19:0] encoder_diff = (left_encoder_count > left_enc_start) ? 
                            (left_encoder_count - left_enc_start) : 
                            (left_enc_start - left_encoder_count);

// New DFS1 instantiation with proper signal mapping
dfs1 dfs1_module (
    .clk(dfs_clk),               // Use toggled DFS clock from MOVE_STATE
    .rst_n(reset),
    .left(ir_left_debounced),
    .mid(wall_front_detected),
    .right(ir_right_debounced),
    .mpi_count(mpi_count),       // MPI counter from movement_logic
    .mpi_total(4'd3),            // MPI total: 3 sensing events
    .move(dfs_move),
    .x(dfs_x),                   // X coordinate output
    .y(dfs_y),                   // Y coordinate output
    .back_tracking_n(dfs_back_tracking), // Backtracking flag
    .debug_clk(),                // TODO: Connect debug clock
    .pos()                       // TODO: Connect position output
);

// Main control logic with hardcoding logic
always @(posedge clk_50M) begin
    if (!reset) begin
        current_state <= STOP_STATE;
        turn_state <= FORWARD_BEFORE_TURN;
        motor_command <= MOTOR_STOP;
        speed_left <= 4'd0;
        speed_right <= 4'd0;
        state_counter <= 0;
        mpi_trigger <= 1'b0;
        exit_maze <= 1'b0;
		dfs_clk <= 0;
		dfs_clk_count <= 0;
		dfs_time_count <= 0;
        state_debug <= STOP_STATE;
    end
    else begin
        state_debug <= current_state;
        state_counter <= state_counter + 1;

        case (current_state)
            STOP_STATE: begin
                motor_command <= MOTOR_STOP;
                speed_left <= 4'd0;
                speed_right <= 4'd0;
                exit_maze <= 1'b1;  // Signal maze completion
                
                // Transition to MOVE_STATE when 'A' command is received (debug_rx_start_cmd)
                if (debug_rx_start_cmd) begin
                    current_state <= MOVE_STATE;
                    state_counter <= 0;
                    exit_maze <= 1'b0;  // Clear exit signal
                end
            end 
            
				/*
				MOVE_STATE:
				- Main decision-making state (navigation brain).
				- Reads debounced IR sensors + ultrasonic front distance to detect:
					 * wall on left/right
					 * wall ahead (wall_front_detected)
					 * maze exit (all open)
				- Based on wall combination, it selects next action:
					 * LEFT_TURN / RIGHT_TURN (with encoder start reset)
					 * WALL_FOLLOWING (when both side walls present)
					 * FORWARD (special straight run case)
					 * STOP (exit condition)
				- Uses lf_count / lr_count / rf_count for hardcoded turn sequences
				  (experiment-based tuning for specific maze paths).
				*/

            MOVE_STATE: begin
                // Pulse clock logic - similar to motor_driver PULSE_CLOCK state
                // Wait for encoder buffer and time buffer to stabilize, then pulse DFS clock 4 times
                if (encoder_diff < DIST_BUFFER && dfs_time_count < TIME_BUFFER && dfs_clk_count == 0) begin
                    // Buffer phase: wait for encoder/sensor stabilization
                    dfs_clk <= 1'b0;
                    dfs_time_count <= dfs_time_count + 1;
                    dfs_decision_valid <= 0;
                end
                else if (dfs_clk_count <= 32'd3) begin
                    // Pulse DFS clock 4 times
                    dfs_time_count <= 0;
                    dfs_clk <= ~dfs_clk;  // Toggle DFS clock
                    dfs_clk_count <= dfs_clk_count + 1;
                    dfs_decision_valid <= 0;
                end
                else begin
                    // After 4 clock toggles, hold clock stable and latch decision
                    dfs_move_latched <= dfs_move;
                    dfs_decision_valid <= 1;
                    dfs_clk_count <= 32'd0;  // Move to "holding" state
                end
                
                // Use DFS decision when ready
                if (dfs_decision_valid) begin
                    case (dfs_move_latched)
                        3'b000: begin  // STOP
                            motor_command <= MOTOR_STOP;
                            speed_left <= 4'd0;
                            speed_right <= 4'd0;
                            state_counter <= 0;
                            current_state <= STOP_STATE;
                        end
                        
                        3'b001: begin  // FORWARD -> WALL_FOLLOWING
                            motor_command <= MOTOR_STOP;
                            speed_left <= 4'd0;
                            speed_right <= 4'd0;
                            state_counter <= 0;
                            left_enc_start <= left_encoder_count;  // Track encoder for tile distance
                            right_enc_start <= right_encoder_count;
                            // Capture reference IR states when entering WALL_FOLLOWING
                            ref_ir_left_debounced <= ir_left_debounced;
                            ref_ir_right_debounced <= ir_right_debounced;
                            ref_ir_front_debounced <= ir_front_debounced;
                            current_state <= WALL_FOLLOWING;
                        end
                        
                        3'b010: begin  // LEFT -> LEFT_TURN
                            motor_command <= MOTOR_STOP;
                            speed_left <= 4'd0;
                            speed_right <= 4'd0;
                            state_counter <= 0;
                            left_enc_start  <= left_encoder_count;
                            right_enc_start <= right_encoder_count;
                            current_state <= LEFT_TURN;
                            left_turn_count <= left_turn_count + 1;
                            turn_state <= FORWARD_BEFORE_TURN;
                        end
                        
                        3'b011: begin  // RIGHT -> RIGHT_TURN
                            motor_command <= MOTOR_STOP;
                            speed_left <= 4'd0;
                            speed_right <= 4'd0;
                            state_counter <= 0;
                            left_enc_start  <= left_encoder_count;
                            right_enc_start <= right_encoder_count;
                            current_state <= RIGHT_TURN;
                            right_turn_count <= right_turn_count + 1;
                            turn_state <= FORWARD_BEFORE_TURN;
                        end
                        
                        3'b100: begin  // UTURN -> UTURN
                            motor_command <= MOTOR_STOP;
                            speed_left <= 4'd0;
                            speed_right <= 4'd0;
                            state_counter <= 0;
                            current_state <= UTURN;
							turn_state <= FORWARD_BEFORE_TURN;
							left_enc_start  <= left_encoder_count;
                            right_enc_start <= right_encoder_count;
                        end
                        
                        default: begin  // Safety fallback
                            motor_command <= MOTOR_STOP;
                            speed_left <= 4'd0;
                            speed_right <= 4'd0;
                            state_counter <= 0;
                        end
                        endcase
                    dfs_decision_valid <= 1'b0;
                end 
            end
				
				
			  /*
				LEFT_TURN:
				- Performs a full left turn using turn_state FSM:
				  1) FORWARD_BEFORE_TURN: move forward (encoder-based) to reach turning point
				  2) TURN: rotate ~90° left using encoder count (TURN_90_COUNT)
				  3) FORWARD_AFTER_TURN: move forward to stabilize and return to MOVE_STATE
				- Uses ultrasonic-based PWM correction during forward phases for wall alignment.
				- front_flag / side_flag slightly adjust turn + forward distances for smoother turns.
				*/

           LEFT_TURN: begin
                case (turn_state) 
                    FORWARD_BEFORE_TURN: begin
						      if ( front_open) begin
								      front_flag <= 1;
										if ( right_delta <= BEFORE_TURN_COUNT - OPEN_FRONT_BEFORE_CORRECTION ) begin
											motor_command <= MOTOR_FORWARD;
											speed_left <= wf_left;
											speed_right <= wf_right;
												 
											if (ultrasonic_safety_stop) begin
												 motor_command <= MOTOR_STOP;
												 speed_left <= 4'd0;
												 speed_right <= 4'd0;
												 turn_state <= TURN;
												 left_enc_start  <= left_encoder_count;
												 right_enc_start <= right_encoder_count;
												 state_counter <= 0;
											end

										end
										else begin
											motor_command <= MOTOR_STOP;
											speed_left <= 4'd0;
											speed_right <= 4'd0;
											turn_state <= TURN;
											left_enc_start  <= left_encoder_count;
											right_enc_start <= right_encoder_count;
											state_counter <= 0;
										end
								end
								else if ( side_open ) begin
								      side_flag <= 1;
										if ( right_delta <= BEFORE_TURN_COUNT - OPEN_SIDE_BEFORE_CORRECTION  ) begin
											motor_command <= MOTOR_FORWARD;
											speed_left <= TURN_SPEED;
											speed_right <= TURN_SPEED;
												 
											if (ultrasonic_safety_stop) begin
												 motor_command <= MOTOR_STOP;
												 speed_left <= 4'd0;
												 speed_right <= 4'd0;
												 turn_state <= TURN;
												 left_enc_start  <= left_encoder_count;
												 right_enc_start <= right_encoder_count;
												 state_counter <= 0;
											end

										end
										else begin
											motor_command <= MOTOR_STOP;
											speed_left <= 4'd0;
											speed_right <= 4'd0;
											turn_state <= TURN;
											left_enc_start  <= left_encoder_count;
											right_enc_start <= right_encoder_count;
											state_counter <= 0;
										end
								end
								else begin
										if ( right_delta <= BEFORE_TURN_COUNT ) begin
											motor_command <= MOTOR_FORWARD;
											speed_left <= wf_left;
											speed_right <= wf_right;
											  
											if (ultrasonic_safety_stop) begin
												 motor_command <= MOTOR_STOP;
												 speed_left <= 4'd0;
												 speed_right <= 4'd0;
												 turn_state <= TURN;
												 left_enc_start  <= left_encoder_count;
												 right_enc_start <= right_encoder_count;
												 state_counter <= 0;
											end

										end
										else begin
											motor_command <= MOTOR_STOP;
											speed_left <= 4'd0;
											speed_right <= 4'd0;
											turn_state <= TURN;
											left_enc_start  <= left_encoder_count;
											right_enc_start <= right_encoder_count;
											state_counter <= 0;
										end
								end
                    end
                    TURN: begin
						    if (side_flag) begin
									if (state_counter > 27'd10000000) begin
											if (right_delta <= TURN_90_COUNT + SIDE_OPEN_TURN_CORRECTION) begin
												 motor_command <= MOTOR_LEFT_TURN;
												 speed_left <= TURN_SPEED;
												 speed_right <= TURN_SPEED;
											end
											else begin
												 turn_state <= FORWARD_AFTER_TURN;
												 left_enc_start  <= left_encoder_count;
												 right_enc_start <= right_encoder_count;
												 state_counter <= 0;
										   end 
								   end			
	                   end
							 else begin
									 if (state_counter > 27'd10000000) begin
											if (right_delta <= TURN_90_COUNT) begin
												 motor_command <= MOTOR_LEFT_TURN;
												 speed_left <= TURN_SPEED;
												 speed_right <= TURN_SPEED;
											end
											else begin
												 turn_state <= FORWARD_AFTER_TURN;
												 left_enc_start  <= left_encoder_count;
												 right_enc_start <= right_encoder_count;
												 state_counter <= 0;
										   end
									  end		
	                   end						 
                    end  
                    FORWARD_AFTER_TURN: begin
							  if (front_flag) begin
									if (right_delta <= AFTER_TURN_COUNT + OPEN_FRONT_AFTER_CORRECTION) begin
										 motor_command <= MOTOR_FORWARD;
	                                     speed_left <= wf_left	;
	                                     speed_right <= wf_right;
									end
									else begin
										 current_state <= MOVE_STATE;
										 motor_command <= MOTOR_STOP;
										 speed_left <= 4'd0;
										 speed_right <= 4'd0;
										 state_counter <= 0;
										 front_flag <= 0;
									end
							  end
							  else if (side_flag) begin
									if (right_delta <= AFTER_TURN_COUNT + OPEN_SIDE_AFTER_CORRECTION ) begin
										 motor_command <= MOTOR_FORWARD;
	                                     speed_left <= wf_left;
	                                     speed_right <= wf_right;
									end
									else begin
										 current_state <= MOVE_STATE;
										 motor_command <= MOTOR_STOP;
										 speed_left <= 4'd0;
										 speed_right <= 4'd0;
										 state_counter <= 0;
										 side_flag <= 0;
									end
							  end
							  else begin
									  if (right_delta <= AFTER_TURN_COUNT) begin
										 motor_command <= MOTOR_FORWARD;
										 speed_left <= wf_left;
										 speed_right <= wf_right;
									end
									else begin
										 current_state <= MOVE_STATE;
										 motor_command <= MOTOR_STOP;
										 speed_left <= 4'd0;
										 speed_right <= 4'd0;
										 state_counter <= 0;
									end
							  end 
                    end  
                endcase    
           end    
                
					 
					 /*
					RIGHT_TURN:
					- Performs a full right turn using turn_state FSM:
					  1) FORWARD_BEFORE_TURN: move forward (encoder-based) to reach turning point
					  2) TURN: rotate ~90° right using encoder count (TURN_90_COUNT)
					  3) FORWARD_AFTER_TURN: move forward to stabilize and return to MOVE_STATE
					- Uses ultrasonic-based PWM correction during forward phases for wall alignment.
					- front_flag / side_flag slightly adjust turn + forward distances for smoother turns.
					*/

                RIGHT_TURN: begin
                    case (turn_state) 
                        FORWARD_BEFORE_TURN: begin
                            if ( front_open) begin
									       front_flag <= 1;
											 if ( left_delta <= BEFORE_TURN_COUNT - OPEN_FRONT_BEFORE_CORRECTION) begin
												  motor_command <= MOTOR_FORWARD;
												  speed_left <= wf_left;
												  speed_right <= wf_right;
												  
													if (ultrasonic_safety_stop) begin
														 motor_command <= MOTOR_STOP;
														 speed_left <= 4'd0;
														 speed_right <= 4'd0;
														 turn_state <= TURN;
														 left_enc_start  <= left_encoder_count;
														 right_enc_start <= right_encoder_count;
														 state_counter <= 0; 
													end
													
											 end
											 else begin
													  motor_command <= MOTOR_STOP;
													  speed_left <= 4'd0;
													  speed_right <= 4'd0;
													  turn_state <= TURN;
													  left_enc_start  <= left_encoder_count;
													  right_enc_start <= right_encoder_count;	
													  state_counter <= 0;  
											 end
									 end
									 else if ( side_open) begin
									       side_flag <= 1;
											 if ( left_delta <= BEFORE_TURN_COUNT - OPEN_SIDE_BEFORE_CORRECTION) begin
												  motor_command <= MOTOR_FORWARD;
												  speed_left <= wf_left;
											      speed_right <= wf_right;
												  
													if (ultrasonic_safety_stop) begin
														 motor_command <= MOTOR_STOP;
														 speed_left <= 4'd0;
														 speed_right <= 4'd0;
														 turn_state <= TURN;
														 left_enc_start  <= left_encoder_count;
														 right_enc_start <= right_encoder_count;
														 state_counter <= 0; 
													end
													
											 end
											 else begin
													  motor_command <= MOTOR_STOP;
													  speed_left <= 4'd0;
													  speed_right <= 4'd0;
													  turn_state <= TURN;
													  left_enc_start  <= left_encoder_count;
													  right_enc_start <= right_encoder_count;	
													  state_counter <= 0;  
											 end
									 end
									 else begin
											 if ( left_delta <= BEFORE_TURN_COUNT ) begin
												  motor_command <= MOTOR_FORWARD;
												  speed_left <= wf_left;
												  speed_right <= wf_right;
												  
													if (ultrasonic_safety_stop	) begin
														 motor_command <= MOTOR_STOP;
														 speed_left <= 4'd0;
														 speed_right <= 4'd0;
														 turn_state <= TURN;
														 left_enc_start  <= left_encoder_count;
														 right_enc_start <= right_encoder_count;
														 state_counter <= 0; 
													end
													
											 end
											 else begin
												 
													  motor_command <= MOTOR_STOP;
													  speed_left <= 4'd0;
													  speed_right <= 4'd0;
													  turn_state <= TURN;
													  left_enc_start  <= left_encoder_count;
													  right_enc_start <= right_encoder_count;	
													  state_counter <= 0;  
											 end
									 end
                        end
                        TURN: begin
								     if (side_flag == 1) begin
											if (state_counter > 27'd10000000) begin
												 if (left_delta <= TURN_90_COUNT + SIDE_OPEN_TURN_CORRECTION) begin
													  motor_command <= MOTOR_RIGHT_TURN;
													  speed_left <= TURN_SPEED;
													  speed_right <= TURN_SPEED;
												 end
												 else begin
													  turn_state <= FORWARD_AFTER_TURN;
													  left_enc_start  <= left_encoder_count;
													  right_enc_start <= right_encoder_count;
													  state_counter <= 0; 
												 end    
											end
							        end
							        else begin
											if (state_counter > 27'd10000000) begin
												 if (left_delta <= TURN_90_COUNT) begin
													  motor_command <= MOTOR_RIGHT_TURN;
													  speed_left <= TURN_SPEED;
													  speed_right <= TURN_SPEED;
												 end
												 else begin
													  turn_state <= FORWARD_AFTER_TURN;
													  left_enc_start  <= left_encoder_count;
													  right_enc_start <= right_encoder_count;
													  state_counter <= 0; 
												 end    
											end
							        end		  
                        end  
                        FORWARD_AFTER_TURN: begin
                            if (front_flag) begin
											 if (left_delta <= AFTER_TURN_COUNT + OPEN_FRONT_AFTER_CORRECTION) begin
												  motor_command <= MOTOR_FORWARD;
	                                              speed_left <= wf_left;
												  speed_right <= wf_right;
											 end
											 else begin
												  current_state <= MOVE_STATE;
												  motor_command <= MOTOR_STOP;
										        speed_left <= 4'd0;
										        speed_right <= 4'd0;
												  state_counter <= 0;
												  front_flag <= 0;
											 end
									 end
									 else if (side_flag) begin
											 if (left_delta <= AFTER_TURN_COUNT + OPEN_SIDE_AFTER_CORRECTION) begin
												  motor_command <= MOTOR_FORWARD;
	                                              speed_left <= wf_left;
												  speed_right <= wf_right;
											 end
											 else begin
												  current_state <= MOVE_STATE;
												  motor_command <= MOTOR_STOP;
											 	  speed_left <= 4'd0;
											  	  speed_right <= 4'd0;
												  state_counter <= 0;
												  side_flag <= 0;
											 end
									 end
									 else begin
											 if (left_delta <= AFTER_TURN_COUNT) begin
												  motor_command <= MOTOR_FORWARD;
												  speed_left <= wf_left;
												  speed_right <= wf_right;
											 end
											 else begin
												  current_state <= MOVE_STATE;
												  motor_command <= MOTOR_STOP;
										        speed_left <= 4'd0;
										        speed_right <= 4'd0;
												  state_counter <= 0;
											 end
									 end
                        end  
                    endcase			  
						 
            end
				
				/*
				UTURN:
				- Executes a 180° turn when front + side walls are blocked (dead end).
				- uturn_flag decides turning direction based on side distances:
					 uturn_flag = 0 -> rotate left
					 uturn_flag = 1 -> rotate right
				- turn_state flow:
				  1) FORWARD_BEFORE_TURN: short stop/delay for stability
				  2) TURN: rotate ~180° using encoder count (TURN_180_COUNT)
				  3) MPI: Initiates soil sensing, temperature and humidity sensing and sends through uart
				  4) FORWARD_AFTER_TURN: move forward slightly, then return to MOVE_STATE
				*/

				
				UTURN: begin
					case (turn_state) 
                    FORWARD_BEFORE_TURN: begin
								if (right_delta > BEFORE_TURN_COUNT || ultrasonic_safety_stop) begin
										if( state_counter < 27'd10000000) begin
														motor_command <= MOTOR_STOP;
														speed_left <= 4'd0;
														speed_right <= 4'd0;
														
														// Detect MPI (wall in front) while waiting
														if (ir_front_debounced) begin
															mpi_detected <= 1'b1;
														end
										end
										else begin
													turn_state <= TURN;
													state_counter <= 0;
													left_enc_start  <= left_encoder_count;
													right_enc_start <= right_encoder_count;
													uturn_flag <= (distance_left <= distance_right) ? 1 : 0;
										end
								end
								else begin
										motor_command <= MOTOR_FORWARD;
										speed_left <= wf_left;
										speed_right <= wf_right;
								end
								
                    end
                    TURN: begin
                       if (!uturn_flag) begin
									if ( right_delta <= TURN_180_COUNT ) begin
											  motor_command <= MOTOR_LEFT_TURN;
											  speed_left <= TURN_SPEED ;
											  speed_right <= TURN_SPEED ;
									end	  
									else begin
										  motor_command <= MOTOR_STOP;
										  speed_left <= 4'd0;
										  speed_right <= 4'd0;
										  // Conditional transition: MPI if detected, else FORWARD_AFTER_TURN
										  if (mpi_detected) begin
										      turn_state <= MPI;
										  end else begin
										      turn_state <= FORWARD_AFTER_TURN;
										      left_enc_start  <= left_encoder_count;
										      right_enc_start <= right_encoder_count;
										  end
										  state_counter <= 0;
									end
							  end	
							  else begin
									if ( left_delta <= TURN_180_COUNT ) begin
											  motor_command <= MOTOR_RIGHT_TURN;
											  speed_left <= TURN_SPEED ;
											  speed_right <= TURN_SPEED ;
									end	  
									else begin
										   motor_command <= MOTOR_STOP;
										   speed_left <= 4'd0;
											speed_right <= 4'd0;
										   // Conditional transition: MPI if detected, else FORWARD_AFTER_TURN
										   if (mpi_detected) begin
										       turn_state <= MPI;
										   end else begin
										       turn_state <= FORWARD_AFTER_TURN;
										       left_enc_start  <= left_encoder_count;
										       right_enc_start <= right_encoder_count;
										   end
										   state_counter <= 0;
									end
							  end	          
                    end 
					 MPI: begin
						   // MPI state - trigger soil sensing and wait for completion
						   mpi_trigger <= 1'b1;
							
							if(mpi_complete) begin
							      if (state_counter >= 29'd300000000) begin
										  mpi_trigger <= 1'b0;
										  mpi_count <= mpi_count + 1;  // Increment MPI counter
										  turn_state <= FORWARD_AFTER_TURN;
										  state_counter <= 0;
										  left_enc_start  <= left_encoder_count;
										  right_enc_start <= right_encoder_count;
										  mpi_detected <= 0;  // Reset MPI detected flag for next time
								   end
							       else begin	
										 motor_command <= MOTOR_STOP;
										 speed_left <= 4'd0;
										 speed_right <= 4'd0;
									end
							end			
										 
					 end
						 FORWARD_AFTER_TURN: begin
						   if (uturn_flag) begin
								if ( left_delta <= AFTER_UTURN_DISTANCE)begin	
								     motor_command <= MOTOR_FORWARD;
									 speed_left <= wf_left;
									 speed_right <= wf_right;
								end
								else begin
										motor_command <= MOTOR_STOP;
										speed_left <= 4'd0;
										speed_right <= 4'd0;
                                        current_state <= MOVE_STATE;
										state_counter <= 0;
								end
							end	
							else begin
								if ( right_delta <= AFTER_UTURN_DISTANCE)begin
									 motor_command <= MOTOR_FORWARD;
									 speed_left <= wf_left;
									 speed_right <= wf_right;
							    end

								else begin
										motor_command <= MOTOR_STOP;
										speed_left <= 4'd0;
										speed_right <= 4'd0;
										current_state <= MOVE_STATE;
										state_counter <= 0;
								end
							end
						 end 
                endcase  
 
				end 
				
				/*
				FORWARD:
				- Moves straight for one complete cell distance using encoder count (FORWARD_COUNT).
				- Uses TURN_SPEED for both motors.
				- After reaching the encoder target, stops and returns to MOVE_STATE.
				*/

			   
			//    FORWARD: begin
			// 			if(uturn_flag) begin
			// 			   if ( left_delta <= FORWARD_COUNT ) begin
			// 						motor_command <= MOTOR_FORWARD;
			// 						speed_left <= TURN_SPEED;
            //                         speed_right <= TURN_SPEED;
			// 				end
			// 				else begin
			// 						motor_command <= MOTOR_STOP;
			// 					   speed_left <= 4'd0;
			// 						speed_right <= 4'd0;
			// 						current_state <= MOVE_STATE;
			// 						state_counter <= 0;
			// 				end
			// 			end
			// 			else begin
			// 				 if ( right_delta <= FORWARD_COUNT ) begin
			// 						motor_command <= MOTOR_FORWARD;
			// 						speed_left <= TURN_SPEED;
            //                speed_right <= TURN_SPEED;
			// 				end
			// 				else begin
			// 						motor_command <= MOTOR_STOP;
			// 					    speed_left <= 4'd0;
			// 						speed_right <= 4'd0;
			// 						current_state <= MOVE_STATE;
			// 						state_counter <= 0;
			// 				end
			// 			end
			//    end
			   
				/*
				WALL_FOLLOWING:
				- Activated when both left and right walls are detected by IR sensors.
				- Robot moves forward while aligning between walls using ultrasonic PWM correction:
					 speed_left  = speed_left_align
					 speed_right = speed_right_align
				- If front + left + right walls are detected (dead end), robot stops and enters UTURN.
				- If side-wall condition breaks, robot stops wall-following and returns to MOVE_STATE.
				*/

            WALL_FOLLOWING: begin
						  motor_command <= MOTOR_FORWARD;
						  speed_left <= wf_left;
						  speed_right <= wf_right;
						  state_counter <= 0;
						  
						  // Exit if tile distance reached OR IR state changes
						  if (right_delta > TILE_DISTANCE + TILE_TOLERANCE || ir_state_changed) begin
								current_state <= MOVE_STATE;
								motor_command <= MOTOR_STOP;
								speed_left <= 4'd0;
								speed_right <= 4'd0;
								state_counter <= 0;
						  end
				end
        endcase    
    end        
end

// Debug output assignments
assign debug_dfs_pos_x = dfs_x[3:0];   // Map dfs1 X coordinate (convert from 32-bit to 4-bit)
assign debug_dfs_pos_y = dfs_y[3:0];   // Map dfs1 Y coordinate (convert from 32-bit to 4-bit)

endmodule