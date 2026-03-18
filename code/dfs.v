module dfs(
    input wire clk,
    input wire rst_n,
    input wire left,
    input wire mid,
    input wire right,
    output reg [2:0] move,
    output wire [3:0] debug_pos_x,  // Debug: Current X coordinate
    output wire [3:0] debug_pos_y   // Debug: Current Y coordinate
);
    // Move commands
    localparam STOP    = 3'b000;
    localparam FORWARD = 3'b001;
    localparam LEFT    = 3'b010;
    localparam RIGHT   = 3'b011;
    localparam UTURN   = 3'b100;

    // Direction encoding
    localparam NORTH = 2'b00;
    localparam EAST  = 2'b01;
    localparam SOUTH = 2'b10;
    localparam WEST  = 2'b11;

    reg [2:0] state;
    reg [1:0] current_dir;
    reg [3:0] pos_x, pos_y;
    reg [6:0] cell_index;
    reg [2:0] move_set;
    
    // DFS tracking - mark which cells have been visited
    reg visited [0:80];
    // reg end_reached;
    reg [7:0] visit_marker;
    
    reg [7:0] stack_coords [0:80];
    reg [6:0] sp;               // Stack pointer
    reg is_tracking;            // High after reaching goal the first time
    reg returning_home;         // High when final exploration is done
    reg returning_junction;     // High when returning from deadend
    
    // Decision variables
    reg [1:0] left_dir, front_dir, right_dir;
    reg can_go_left, can_go_front, can_go_right;
    reg [6:0] next_left_cell, next_front_cell, next_right_cell;
    reg left_visited, front_visited, right_visited;
    wire [3:0] target_x;
    wire [3:0] target_y;

    assign {target_y, target_x} = (sp > 0) ? stack_coords[sp-1] : 8'h48; // Default to start
    integer i, j;

    // FSM states
    localparam IDLE   = 3'b000;
    localparam SENSE1 = 3'b001;
   // localparam SENSE2 = 3'b010;
    localparam DECIDE = 3'b011;
    localparam EXEC   = 3'b100;

    // Calculate cell index from position
    always @(*) begin
        cell_index = pos_y * 7'd9 + pos_x; 
	    can_go_left = !left;
     	can_go_front = !mid;
     	can_go_right = !right;    

        //Combinational logic for returning back
         if (returning_home) begin
            if (sp == 0) begin
                move_set = FORWARD; // Final Exit
            end 
            else begin                
                if (target_x > pos_x) begin // Need to go EAST
                    if (current_dir == EAST) move_set = FORWARD;
                    else if (current_dir == NORTH) move_set = RIGHT;
                    else if (current_dir == SOUTH) move_set = LEFT;
                    else move_set = UTURN;
                end else if (target_x < pos_x) begin // Need to go WEST
                    if (current_dir == WEST) move_set= FORWARD;
                    else if (current_dir == NORTH) move_set = LEFT;
                    else if (current_dir == SOUTH) move_set = RIGHT;
                    else move_set = UTURN;
                end else if (target_y < pos_y) begin // Need to go NORTH
                    if (current_dir == NORTH) move_set = FORWARD;
                    else if (current_dir == EAST) move_set = LEFT;
                    else if (current_dir == WEST) move_set = RIGHT;
                    else move_set = UTURN;
                end else if (target_y > pos_y) begin // Need to go SOUTH
                    if (current_dir == SOUTH) move_set = FORWARD;
                    else if (current_dir == EAST) move_set = RIGHT;
                    else if (current_dir == WEST) move_set = LEFT;
                    else move_set = UTURN;
                end                     
        end
                end

            case (current_dir)
                    NORTH: begin
                        left_dir = WEST;
                        front_dir = NORTH;
                        right_dir = EAST;
                    end
                    EAST: begin
                        left_dir = NORTH;
                        front_dir = EAST;
                        right_dir = SOUTH;
                    end
                    SOUTH: begin
                        left_dir = EAST;
                        front_dir = SOUTH;
                        right_dir = WEST;
                    end
                    WEST: begin
                        left_dir = SOUTH;
                        front_dir = WEST;
                        right_dir = NORTH;
                    end
                endcase
                        
            // Calculate next cell indices for each direction
            case (left_dir)
                NORTH: next_left_cell = cell_index - 7'd9;
                SOUTH: next_left_cell = cell_index + 7'd9;
                EAST:  next_left_cell = cell_index + 7'd1;
                WEST:  next_left_cell = cell_index - 7'd1;
            endcase
            
            case (front_dir)
                NORTH: next_front_cell = cell_index - 7'd9;
                SOUTH: next_front_cell = cell_index + 7'd9;
                EAST:  next_front_cell = cell_index + 7'd1;
                WEST:  next_front_cell = cell_index - 7'd1;
            endcase
            
            case (right_dir)
                NORTH: next_right_cell = cell_index - 7'd9;
                SOUTH: next_right_cell = cell_index + 7'd9;
                EAST:  next_right_cell = cell_index + 7'd1;
                WEST:  next_right_cell = cell_index - 7'd1;
            endcase
            
            // Check if next cells are visited and if we've tried this direction
            left_visited = visited[next_left_cell];
            front_visited = visited[next_front_cell];
            right_visited = visited[next_right_cell];

    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            current_dir <= NORTH;
            pos_x <= 4'd4;
            pos_y <= 4'd8;
            move <= STOP;
            //move_set <= STOP;
            visit_marker <= 8'd0;
            sp <= 0;
            is_tracking <= 1'b0;
            returning_home <= 1'b0;
            returning_junction <= 1'b0;
            // can_go_left <= 0;
            // can_go_front <= 1;
            // can_go_right <= 0;
            
            
            // Initialize visited and tried arrays
            for (i = 0; i < 81; i = i + 1) begin
                visited[i] <= 1'b0;
            end
            
        end else begin
            case (state)
                IDLE: begin
                    state <= SENSE1;
                   // move <= STOP;
                end
                
                SENSE1: begin
                    state <= DECIDE;
                    move <= STOP;
                    
                    // Mark current cell as visited
                    if (!visited[cell_index])begin
                    visited[cell_index] <= 1'b1;
                    visit_marker <= visit_marker + 1'b1;
                    end   
                    
                end
                
                DECIDE: begin
                    // 1. Arm Tracking when goal is first reached
                    if (pos_x == 4'd4 && pos_y == 4'd0 && !is_tracking) begin
                    is_tracking <= 1'b1;
                    sp <= 0; 
                    move <= UTURN;
                    end

                    if (visit_marker == 8'd81) begin
                    returning_home <= 1'b1;
                    end

                    if (is_tracking && !returning_home)begin
                    if (!can_go_front && !can_go_left && !can_go_right )
                    returning_junction <= 1'b1;

                     if (returning_junction && ((can_go_front && can_go_left ) || 
                     (can_go_front && can_go_right) || (can_go_left && can_go_right)))begin                           
                         returning_junction <= 1'b0;
                        end
                     end                        

                    if (returning_home) begin
                        if (sp == 0) begin
                            move <= FORWARD; // Final Exit
                        end 
			            else begin
                           
                            if (target_x > pos_x) begin // Need to go EAST
                                move <= move_set;
                            end else if (target_x < pos_x) begin // Need to go WEST
                                move <= move_set;
                            end else if (target_y < pos_y) begin // Need to go NORTH
                                move <= move_set;
                            end else if (target_y > pos_y) begin // Need to go SOUTH
                                move <= move_set;
                                end
                            state <= EXEC;                      
                    end
                    end
                    else begin

                        if (can_go_left && !left_visited ) begin
                            move <= LEFT;
                        end
                        else if (can_go_front && !front_visited) begin
                            move <= FORWARD;
                        end
                        else if (can_go_right && !right_visited ) begin
                            move <= RIGHT;
                        end
                        // All unvisited paths explored, now allow revisiting for backtracking
                        else if (can_go_left) begin
                            move <= LEFT;
                        end
                        else if (can_go_front) begin
                            move <= FORWARD;
                        end
                        else if (can_go_right) begin
                            move <= RIGHT;
                        end
                        // All directions tried from this cell, turn around
                        else begin
                            move <= UTURN;                           
                        end
                        
                        state <= EXEC;
                    end
                end
                
                EXEC: begin
                    // --- Stack Management (Pruning) ---
                    if (is_tracking && !returning_home) begin
                        if (move == UTURN) begin
                            if (sp > 0) sp <= sp - 1; // On hitting a deadend pop the current cell
                        end
                        else if (returning_junction)begin
                            if (sp > 0) sp <= sp - 1;
                        end
                        else if (move == FORWARD || move == LEFT || move == RIGHT) begin
                            stack_coords[sp] <= {pos_y, pos_x};
                            sp <= sp + 1;
                        end
                           
                    end
                        
                        
                else if (returning_home) begin
                        if ((sp > 0) && (move == FORWARD 
                        || move == LEFT 
                        || move == RIGHT 
                        || move == UTURN))
                         sp <= sp - 1;
                    end

                    
                    // Execute the move
                    case (move)
                        STOP: begin
                            pos_x <= pos_x;
                            pos_y <= pos_y;
                            current_dir <= current_dir;
                        end
                        LEFT: begin
                            case (current_dir)
                                NORTH: begin current_dir <= WEST;  pos_x <= pos_x - 4'd1; end
                                EAST:  begin current_dir <= NORTH; pos_y <= pos_y - 4'd1; end
                                SOUTH: begin current_dir <= EAST;  pos_x <= pos_x + 4'd1; end
                                WEST:  begin current_dir <= SOUTH; pos_y <= pos_y + 4'd1; end
                            endcase
                        end
                        RIGHT: begin
                            case (current_dir)
                                NORTH: begin current_dir <= EAST;  pos_x <= pos_x + 4'd1; end
                                EAST:  begin current_dir <= SOUTH; pos_y <= pos_y + 4'd1; end
                                SOUTH: begin current_dir <= WEST;  pos_x <= pos_x - 4'd1; end
                                WEST:  begin current_dir <= NORTH; pos_y <= pos_y - 4'd1; end
                            endcase
                        end
                        UTURN: begin
                            case (current_dir)
                                NORTH: begin current_dir <= SOUTH; pos_y <= pos_y + 4'd1; end
                                EAST:  begin current_dir <= WEST;  pos_x <= pos_x - 4'd1; end
                                SOUTH: begin current_dir <= NORTH; pos_y <= pos_y - 4'd1; end
                                WEST:  begin current_dir <= EAST;  pos_x <= pos_x + 4'd1; end
                            endcase
                        end
                        FORWARD: begin
                            case (current_dir)
                                NORTH: pos_y <= pos_y - 4'd1;
                                SOUTH: pos_y <= pos_y + 4'd1;
                                EAST:  pos_x <= pos_x + 4'd1;
                                WEST:  pos_x <= pos_x - 4'd1;
                            endcase
                        end
                    endcase
                    
                    state <= IDLE; // After executing, go back to sensing state
                end
                
                default: state <= IDLE;
            endcase
        end
    end

    // Debug outputs - wire coordinates
    assign debug_pos_x = pos_x;
    assign debug_pos_y = pos_y;

endmodule