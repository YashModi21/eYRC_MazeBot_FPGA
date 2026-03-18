module dfs1 (
    input clk,
    input rst_n,
    input left,
    input mid,
    input right,
    input [3:0] mpi_count,
    input [3:0] mpi_total,
    output reg [3:0] move,
    output reg [31:0] x,
    output reg [31:0] y,
	output reg back_tracking_n,
    output reg [3:0] debug_clk,
    output reg [6:0] pos
);

localparam  NORTH = 2'b00,
            EAST  = 2'b01,
            SOUTH = 2'b10,
            WEST  = 2'b11;

localparam STOP    = 3'd0, //movement output
           FORWARD = 3'd1,
           LEFT    = 3'd2,
           RIGHT   = 3'd3,
           UTURN   = 3'd4;

reg [1:0] dir = NORTH;
// reg [6:0] pos = 7'd76;
reg [1:0] rst_delay = 2'd0;

wire signed [6:0] delta [3:0];
assign delta[0] = -7'd9;
assign delta[1] = 7'd1;
assign delta[2] = 7'd9;
assign delta[3] = -7'd1;

reg [6:0] next_pos;
reg [1:0] next_dir;
reg [3:0] next_move;
integer i, j;

reg [6:0] left_pos, forward_pos, right_pos, back_pos;
reg [1:0] left_dir, right_dir, back_dir;
reg buffer = 0;

reg [80:0] visited = (81'd1 << 76);
reg [80:0] exit_visited = (81'd1 << 4);
reg exit_path = 0;

reg [6:0] stack [127:0], exit_stack[127:0];
reg [6:0] stack_ptr = 0, exit_stack_ptr = 0;

reg stack_op_push = 0, exit_stack_op_push = 0;
reg [6:0] target_backtrack_pos = 0;

wire find_exit;
assign find_exit = &visited || (mpi_count >= mpi_total && exit_path);

initial begin
    pos <= 7'd76;
end

always @(posedge clk) begin
    buffer <= buffer + 1;
end

always @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
        rst_delay <= 2'd0;
    end
    else begin
        if (rst_delay != 0) begin
            rst_delay <= rst_delay - 2'd1;
            move <= STOP;
        end
        else begin
                if (buffer) begin
                    debug_clk <= debug_clk + 1;
                    move <= next_move;
                    dir <= next_dir;
                    pos <= next_pos;

                    x <= pos % 9;
                    y <= pos / 9;

                    visited[next_pos] <= 1'b1;
                    if (exit_path)
                        exit_visited[next_pos] <= 1'b1;

                    if (~exit_path && pos == 7'd4)
                        exit_path <= 1;

                    if (stack_op_push) begin
                        stack[stack_ptr] <= pos;
                        back_tracking_n <= 1;
                        stack_ptr <= stack_ptr + 1;
                    end
                        else begin
                        if (stack_ptr > 0) stack_ptr <= stack_ptr - 1;
                        back_tracking_n <= 0;
                    end

                    if (exit_stack_op_push && (exit_path || pos == 4)) begin
                        exit_stack[exit_stack_ptr] <= pos;
                        exit_stack_ptr <= exit_stack_ptr + 1;
                    end
                        else if (exit_path) begin
                        if (exit_stack_ptr > 0)
                            exit_stack_ptr <= exit_stack_ptr - 1;
                    end
//                    end
                end
            end
        end
    end

    always @(*) begin
//        if (buffer) begin
            left_dir = (dir == 2'b00) ? 2'b11 : dir - 2'd1;
            right_dir = (dir == 2'b11) ? 2'b00 : dir + 2'd1;
            back_dir = (dir >= 2'b10) ? dir - 2'd2 : dir + 2'd2;

            left_pos = pos + delta[left_dir] ;
            forward_pos = pos + delta[dir];
            right_pos = pos + delta[right_dir];
            back_pos = pos + delta[back_dir];

//            next_move = FORWARD;
//            stack_op_push = 0;
//            exit_stack_op_push = 0;

            if (!(find_exit)) begin
						if (stack_ptr > 0)
                    target_backtrack_pos = stack[stack_ptr - 1];
						else
                    target_backtrack_pos = pos;
                        if (pos == 7'd123)
                                 next_move = STOP;
						else if ((!left && !visited[left_pos] && left_pos != 7'd123) || (!left && left_pos == 7'd123 && mpi_count >= mpi_total)) begin
								  next_move = LEFT;
								  stack_op_push = 1;
						end
						else if ((!mid && !visited[forward_pos] && forward_pos != 7'd123) || (!mid && forward_pos == 7'd123 && mpi_count >= mpi_total)) begin
								  next_move = FORWARD;
								  stack_op_push = 1;
						end
						else if ((!right && !visited[right_pos] && right_pos != 7'd123) || (!right && right_pos == 7'd123 && mpi_count >= mpi_total)) begin
								  next_move = RIGHT;
								  stack_op_push = 1;
						end 
						else if (stack_ptr > 0) begin
								  stack_op_push = 0;
								  if (target_backtrack_pos == forward_pos)
										next_move = FORWARD;
								  else if (target_backtrack_pos == left_pos)
										next_move = LEFT;
								  else if (target_backtrack_pos == right_pos)
										next_move = RIGHT;
								  else if (target_backtrack_pos == back_pos)
										next_move = UTURN;
								  else
										next_move = STOP;
						end
						else begin
										next_move = STOP;
						end
            end
				else begin
                 
				if (pos == 7'd4) begin
                    if (dir == NORTH)      next_move = FORWARD;
                    else if (dir == WEST)  next_move = RIGHT;
                    else if (dir == EAST)  next_move = LEFT;
                    else                   next_move = UTURN;  
                end
                else if (pos == 7'd123) begin
                    next_move = STOP;
                end
                else begin
					if (exit_stack_ptr > 0)
                        target_backtrack_pos = exit_stack[exit_stack_ptr - 1];
                    else if (stack_ptr > 0)
                        target_backtrack_pos = stack[stack_ptr - 1];

                    exit_stack_op_push = 0;
                    if (target_backtrack_pos == forward_pos)
                        next_move = FORWARD;
                    else if (target_backtrack_pos == left_pos)
                        next_move = LEFT;
                    else if (target_backtrack_pos == right_pos)
                        next_move = RIGHT;
                    else if (target_backtrack_pos == back_pos)
                        next_move = UTURN;
                    else
                        next_move = STOP;
                end
            end

            next_pos = pos;
            next_dir = dir;

            case (next_move)
                FORWARD: begin
                    next_pos = forward_pos;
                    next_dir = dir;
                    exit_stack_op_push = (exit_path && !exit_visited[forward_pos] && !(find_exit)) ? 1'b1 : 1'd0;
                end
                LEFT: begin
                    next_pos = left_pos;
                    next_dir = left_dir;
                    exit_stack_op_push = (exit_path && !exit_visited[left_pos] && !(find_exit)) ? 1'b1 : 1'd0;
                end
                RIGHT: begin
                    next_pos = right_pos;
                    next_dir = right_dir;
                    exit_stack_op_push = (exit_path && !exit_visited[right_pos] && !(find_exit)) ? 1'b1 : 1'd0;
                end
                UTURN: begin
                    next_pos = back_pos;
                    next_dir = back_dir;
                    exit_stack_op_push = (exit_path && !exit_visited[back_pos] && !(find_exit)) ? 1'b1 : 1'd0;
                end
                STOP: begin
                    next_pos = pos;
                    next_dir = dir;
                end
            endcase
//        end
    end

endmodule
