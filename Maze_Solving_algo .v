module t2c_maze_explorer (
    input clk,
    input rst_n,
    input left, mid, right,
    output reg [2:0] move
);

    // -------------------- PARAMETERS --------------------
    parameter DEBUG = 1;
    localparam NORTH = 2'd0, EAST = 2'd1, SOUTH = 2'd2, WEST = 2'd3;

    // -------------------- DECLARATIONS --------------------
    reg [1:0] bot_facing;
    reg [3:0] pos_x, pos_y;
    reg [3:0] prev_x, prev_y;
    reg [6:0] idx;
    reg [6:0] k;

    // Visit map
    reg [1:0] visit_map [0:80];
    reg [6:0] i;

	   // Helper task for checking stack top
                        reg is_stack_top;
	 
	                     // Local vars for combinational logic
                    reg eff_left, eff_mid, eff_right;
                    reg [1:0] eff_open_paths;

	 
    // Stacks
    reg [3:0] junction_x[0:15];
    reg [3:0] junction_y[0:15];
    reg [3:0] junction_prev_x[0:15];
    reg [3:0] junction_prev_y[0:15];
    reg [4:0] stack_ptr;

    reg [3:0] return_exit_junction_x[0:15];
    reg [3:0] return_exit_junction_y[0:15];
    reg [3:0] return_exit_junction_prev_x[0:15];
    reg [3:0] return_exit_junction_prev_y[0:15];
    reg [4:0] return_exit_stack_ptr;

    // Control Flags
    reg storing_return_path;
    reg return_active;
    reg backtrack_active;
    reg [3:0] target_x, target_y; // Keep registers to hold state
    reg all_visited;
    reg [1:0] current_facing;

    // Logic Wires (Reduced Registers where possible)
    reg [1:0] left_count, mid_count, right_count;
    
    // These hold the indices calculated for wall checking
    reg [6:0] left_idx, mid_idx, right_idx;
    
    // Temporary variable for the target calculation
    reg [6:0] target_idx_calc; 

    reg use_left_wall;
    reg [15:0] cycle_count;
    reg [6:0] r, c;

    // -------------------- HELPER FUNCTIONS --------------------
    function [7:0] facing_to_char;
        input [1:0] f;
        begin
            case (f)
                NORTH: facing_to_char = "N";
                EAST:  facing_to_char = "E";
                SOUTH: facing_to_char = "S";
                WEST:  facing_to_char = "W";
                default: facing_to_char = "?";
            endcase
        end
    endfunction

    // 1. Calculate Map Index (Coordinate -> Scalar)
    function [6:0] get_index;
        input [3:0] x, y;
        reg [6:0] y_mul9;
        begin
            y_mul9 = (y << 3) + y; // Shift is "free" in hardware
            get_index = y_mul9 + x;
        end
    endfunction

    // 2. Calculate Neighbor Index (Coordinate + Dir + Move -> Neighbor Index)
    function [6:0] get_next_cell;
        input [3:0] x, y;
        input [1:0] f;
        input [2:0] mv;
        reg [3:0] nx, ny;
        begin
            nx = x; ny = y;
            case (f)
                NORTH: begin
                    if (mv == 3'b001) ny = y - 1;
                    else if (mv == 3'b010) nx = x - 1;
                    else if (mv == 3'b011) nx = x + 1;
                end
                EAST: begin
                    if (mv == 3'b001) nx = x + 1;
                    else if (mv == 3'b010) ny = y - 1;
                    else if (mv == 3'b011) ny = y + 1;
                end
                SOUTH: begin
                    if (mv == 3'b001) ny = y + 1;
                    else if (mv == 3'b010) nx = x + 1;
                    else if (mv == 3'b011) nx = x - 1;
                end
                WEST: begin
                    if (mv == 3'b001) nx = x - 1;
                    else if (mv == 3'b010) ny = y + 1;
                    else if (mv == 3'b011) ny = y - 1;
                end
            endcase
            get_next_cell = get_index(nx, ny);
        end
    endfunction

    // -------------------- FSM --------------------
    reg [1:0] state;
    localparam [1:0] IDLE=0, UPDATE_POS=1, MOVE_FWD=2;

    always @(posedge clk) begin
        if (!rst_n) begin
            for (i = 0; i < 81; i = i + 1) visit_map[i] <= 2'b00;
            move <= 3'b000;
            bot_facing <= NORTH;
            pos_x <= 4'd4; pos_y <= 4'd8;
            prev_x <= 4'd4; prev_y <= 4'd8;
            stack_ptr <= 0; return_exit_stack_ptr <= 0;
            backtrack_active <= 0; storing_return_path <= 0;
            return_active <= 0; all_visited <= 0;
            target_x <= 0; target_y <= 0;
            cycle_count <= 0;
            state <= IDLE;
        end else begin
            cycle_count <= cycle_count + 1;
            case (state)
                IDLE: state <= UPDATE_POS;
                
                MOVE_FWD: begin

                    // 1. Guardrails
                    eff_left = left; eff_mid = mid; eff_right = right;
                    if (pos_x == 4 && pos_y == 8) begin
                        if (bot_facing == SOUTH) eff_mid = 1;
                        else if (bot_facing == EAST) eff_right = 1;
                        else if (bot_facing == WEST) eff_left = 1;
                    end
                    eff_open_paths = (!eff_left) + (!eff_mid) + (!eff_right);
                    idx = get_index(pos_x, pos_y);

                    // 2. Update Map
                    if (visit_map[idx] < 2'b11) visit_map[idx] <= visit_map[idx] + 1;

                    // 3. Check All Visited
                    if (!all_visited) begin
                        all_visited = 1;
                        for (k = 0; k < 81; k = k + 1)
                            if (visit_map[k] == 0 && k != idx) all_visited = 0;
                    end

                    // 4. Triggers
                    if (pos_x == 4 && pos_y == 0 && !all_visited) storing_return_path <= 1;
                    if (all_visited && !return_active) return_active <= 1;

                    // 5. Stack Pushes
                    if (storing_return_path && eff_open_paths >= 2) begin
                          if (return_exit_stack_ptr == 0 || 
                             (return_exit_junction_x[return_exit_stack_ptr-1] != pos_x || 
                              return_exit_junction_y[return_exit_stack_ptr-1] != pos_y)) begin
                              return_exit_junction_x[return_exit_stack_ptr] <= pos_x;
                              return_exit_junction_y[return_exit_stack_ptr] <= pos_y;
                              return_exit_junction_prev_x[return_exit_stack_ptr] <= prev_x;
                              return_exit_junction_prev_y[return_exit_stack_ptr] <= prev_y;
                              return_exit_stack_ptr <= return_exit_stack_ptr + 1;
                          end
                    end
                    if (eff_open_paths >= 2 && visit_map[idx] == 0) begin
                        junction_x[stack_ptr] <= pos_x;
                        junction_y[stack_ptr] <= pos_y;
                        junction_prev_x[stack_ptr] <= prev_x;
                        junction_prev_y[stack_ptr] <= prev_y;
                        stack_ptr <= stack_ptr + 1;
                    end

                    // 6. Calculate Neighbor Indices (REUSED FOR MOVING LOGIC)
                    left_idx = get_next_cell(pos_x, pos_y, bot_facing, 3'b010);
                    mid_idx = get_next_cell(pos_x, pos_y, bot_facing, 3'b001);
                    right_idx = get_next_cell(pos_x, pos_y, bot_facing, 3'b011);
                    
                    left_count = eff_left ? 2'b11 : visit_map[left_idx];
                    mid_count = eff_mid ? 2'b11 : visit_map[mid_idx];
                    right_count = eff_right ? 2'b11 : visit_map[right_idx];

                    // 7. Move Selection
                    if (pos_x == 4 && pos_y == 0 && all_visited) begin
                        // Exit Logic
                        case (bot_facing)
                             NORTH: move <= 3'b001; 
                             EAST:  move <= 3'b010; 
                             WEST:  move <= 3'b011; 
                             SOUTH: move <= 3'b100; 
                        endcase
                    end else if (return_active) begin
                        // RETURN MODE
                        if (return_exit_stack_ptr > 0 && 
                            pos_x == return_exit_junction_x[return_exit_stack_ptr-1] && 
                            pos_y == return_exit_junction_y[return_exit_stack_ptr-1]) begin
                            
                            // CALCULATE TARGET INDEX ONCE
                            target_idx_calc = get_index(return_exit_junction_prev_x[return_exit_stack_ptr-1],
                                                        return_exit_junction_prev_y[return_exit_stack_ptr-1]);
                            
                            // LOGIC REDUCTION: Compare Target Index to Neighbors
                            if (target_idx_calc == mid_idx) move <= 3'b001;      // Target is Forward
                            else if (target_idx_calc == left_idx) move <= 3'b010;// Target is Left
                            else if (target_idx_calc == right_idx) move <= 3'b011;// Target is Right
                            else move <= 3'b100;                                 // Target is Behind

                            return_exit_stack_ptr <= return_exit_stack_ptr - 1; 
                        end else begin
                             if (!eff_left) move <= 3'b010;
                            else if (!eff_mid) move <= 3'b001;
                            else if (!eff_right) move <= 3'b011;
                            else move <= 3'b100;
                        end
                    end else if (eff_open_paths == 0) begin
                        move <= 3'b100; 
                    end else if (eff_open_paths == 1) begin
                        if (!eff_left) move <= 3'b010;
                        else if (!eff_mid) move <= 3'b001;
                        else if (!eff_right) move <= 3'b011;
                    end else begin
                        // JUNCTION EXPLORATION
                        use_left_wall = (bot_facing == NORTH || bot_facing == WEST);
                        
                      
                        is_stack_top = (stack_ptr > 0 && junction_x[stack_ptr-1] == pos_x && junction_y[stack_ptr-1] == pos_y);

                        if (use_left_wall) begin
                            if (!eff_left && left_count == 0) move <= 3'b010;
                            else if (!eff_mid && mid_count == 0) move <= 3'b001;
                            else if (!eff_right && right_count == 0) move <= 3'b011;
                            else begin
                                if (is_stack_top) begin
                                    stack_ptr <= stack_ptr - 1;
                                    target_x <= junction_prev_x[stack_ptr-1]; 
                                    target_y <= junction_prev_y[stack_ptr-1];
                                    
                                    // LOGIC REDUCTION: Calculate Index and Compare
                                    target_idx_calc = get_index(junction_prev_x[stack_ptr-1], junction_prev_y[stack_ptr-1]);
                                    
                                    if (target_idx_calc == mid_idx) move <= 3'b001;
                                    else if (target_idx_calc == left_idx) move <= 3'b010;
                                    else if (target_idx_calc == right_idx) move <= 3'b011;
                                    else move <= 3'b100;

                                    backtrack_active <= 1;
                                end else move <= 3'b100;
                            end
                        end else begin
                            if (!eff_right && right_count == 0) move <= 3'b011;
                            else if (!eff_mid && mid_count == 0) move <= 3'b001;
                            else if (!eff_left && left_count == 0) move <= 3'b010;
                            else begin
                                if (is_stack_top) begin
                                    stack_ptr <= stack_ptr - 1;
                                    target_x <= junction_prev_x[stack_ptr-1]; 
                                    target_y <= junction_prev_y[stack_ptr-1];
                                    
                                    // LOGIC REDUCTION: Calculate Index and Compare
                                    target_idx_calc = get_index(junction_prev_x[stack_ptr-1], junction_prev_y[stack_ptr-1]);
                                    
                                    if (target_idx_calc == mid_idx) move <= 3'b001;
                                    else if (target_idx_calc == left_idx) move <= 3'b010;
                                    else if (target_idx_calc == right_idx) move <= 3'b011;
                                    else move <= 3'b100;

                                    backtrack_active <= 1;
                                end else move <= 3'b100;
                            end
                        end
                    end
						                $display("\n---- VISIT MAP (9x9) ----");
for (r = 0; r < 9; r = r + 1) begin
    $write("Row %0d: ", r);
    for (c = 0; c < 9; c = c + 1) begin
        $write("%0d ", visit_map[r*9 + c]);
    end
    $write("\n");
end
$display("-------------------------");
                    state <= UPDATE_POS;
                end

                UPDATE_POS: begin
                    prev_x <= pos_x; prev_y <= pos_y;
                    current_facing = bot_facing;
                    case (move)
                        3'b010: bot_facing <= bot_facing - 1;
                        3'b011: bot_facing <= bot_facing + 1;
                        3'b100: bot_facing <= bot_facing + 2;
                    endcase
                    case (move)
                        3'b001: begin
                            if (current_facing == NORTH) pos_y <= pos_y - 1;
                            else if (current_facing == EAST) pos_x <= pos_x + 1;
                            else if (current_facing == SOUTH) pos_y <= pos_y + 1;
                            else if (current_facing == WEST) pos_x <= pos_x - 1;
                        end
                        3'b010: begin
                            if (current_facing == NORTH) pos_x <= pos_x - 1;
                            else if (current_facing == EAST) pos_y <= pos_y - 1;
                            else if (current_facing == SOUTH) pos_x <= pos_x + 1;
                            else if (current_facing == WEST) pos_y <= pos_y + 1;
                        end
                        3'b011: begin
                            if (current_facing == NORTH) pos_x <= pos_x + 1;
                            else if (current_facing == EAST) pos_y <= pos_y + 1;
                            else if (current_facing == SOUTH) pos_x <= pos_x - 1;
                            else if (current_facing == WEST) pos_y <= pos_y - 1;
                        end
                        3'b100: begin
                            if (current_facing == NORTH) pos_y <= pos_y + 1;
                            else if (current_facing == EAST) pos_x <= pos_x - 1;
                            else if (current_facing == SOUTH) pos_y <= pos_y - 1;
                            else if (current_facing == WEST) pos_x <= pos_x + 1;
                        end
                    endcase
                    if (backtrack_active && pos_x == target_x && pos_y == target_y) backtrack_active <= 0;
                    state <= MOVE_FWD;
                end
            endcase
        end
    end
endmodule