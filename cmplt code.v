module mb_2805_task5b (
input wire clk_50M,
input wire rst_n,
input wire ir_sensor,

// Ultrasonic Sensor Pins
input wire echo_L, echo_C, echo_R,
output wire trig_L, trig_C, trig_R,

inout wire dht_dat,           // DHT11 Temperature/Humidity Sensor
input wire adc_dout,          // Moisture Sensor ADC Data Out
output wire adc_cs_n,         // ADC Chip Select
output wire adc_din,          // ADC Data In
output wire adc_sck,          // ADC Serial Clock
input wire bt_rx_pin,         // Bluetooth UART RX
output wire bt_tx_pin,        // Bluetooth UART TX        // Bluetooth UART TX
output wire [7:0] led_moisture, // Moisture Level LEDs

// Servo Outputs
output wire servo_pwm1,
output wire servo_pwm2,

// Motor Encoder Inputs
input wire left_enc_a, left_enc_b,
input wire right_enc_a, right_enc_b,

// Motor Driver Outputs
output wire left_in1, left_in2, left_en,
output wire right_in3, right_in4, right_en,

// Debug LEDs
output wire [7:0] led
);

// ---------------------------------------------------------
// 1. SAFETY STARTUP TIMER
// ---------------------------------------------------------
reg [31:0] startup_cnt;
reg system_ready;

always @(posedge clk_50M or negedge rst_n) begin
if (!rst_n) begin
startup_cnt <= 0;
system_ready <= 0;
end else begin
if (startup_cnt < 32'd50_000_000) begin
startup_cnt <= startup_cnt + 1;
system_ready <= 0;
end else begin
system_ready <= bt_start_received;  // Only ready after "start" command
end
end
end

// ---------------------------------------------------------
// 1.5 BLUETOOTH START COMMAND
// ---------------------------------------------------------
wire bt_start_received;

// ---------------------------------------------------------
// 2.5 MASTER CONTROL STATE MACHINE
// ---------------------------------------------------------
localparam MODE_MAZE_SOLVING = 2'd0;
localparam MODE_SERVO_OPERATION = 2'd1;
localparam MODE_UTURN_AFTER_SERVO = 2'd2;

reg [1:0] master_mode;
reg [1:0] master_mode_next;

// IR Edge Detection
// IR Edge Detection
reg ir_prev;
always @(posedge clk_50M) ir_prev <= ir_sensor;
wire ir_rising_edge = (ir_sensor && !ir_prev);

// Servo operation status
wire servo_operation_complete;
wire uturn_complete;
wire op_L, op_C, op_R;

// U-turn completion edge detection from motor controller
reg motor_uturn_done_prev;
always @(posedge clk_50M) motor_uturn_done_prev <= motor_uturn_done;
wire uturn_rising_edge = (motor_uturn_done && !motor_uturn_done_prev);

// Master Mode FSM
always @(posedge clk_50M or negedge rst_n) begin
    if (!rst_n) begin
        master_mode <= MODE_MAZE_SOLVING;
    end else if (system_ready)begin
        master_mode <= master_mode_next;
    end
end

always @(*) begin
    master_mode_next = master_mode;
    
    case (master_mode)
        MODE_MAZE_SOLVING: begin
            if (system_ready && ir_rising_edge&&(op_L && op_C && op_R)) begin
                master_mode_next = MODE_SERVO_OPERATION;
            end
        end
        
        MODE_SERVO_OPERATION: begin
            if (servo_operation_complete) begin
                master_mode_next = MODE_UTURN_AFTER_SERVO;
            end
        end
        
        MODE_UTURN_AFTER_SERVO: begin
            if (uturn_complete) begin
                master_mode_next = MODE_MAZE_SOLVING;
            end
        end
        
        default: master_mode_next = MODE_MAZE_SOLVING;
    endcase
end
// ---------------------------------------------------------
// 2. Ultrasonic Sensor Array
// ---------------------------------------------------------
wire [15:0] dist_L, dist_C, dist_R;

ultrasonic_array_top u_sensors (
.clk_50M(clk_50M),
.reset(rst_n),
.echo_L(echo_L), .echo_C(echo_C), .echo_R(echo_R),
.trig_L(trig_L), .trig_C(trig_C), .trig_R(trig_R),
.op_L(op_L), .op_C(op_C), .op_R(op_R),
.dist_L(dist_L), .dist_C(dist_C), .dist_R(dist_R)
);

// ---------------------------------------------------------
// 3. Maze Explorer Logic (Brain)
// ---------------------------------------------------------
wire [2:0] brain_cmd;

t2c_maze_explorer u_brain (
.clk(clk_50M),
.rst_n(rst_n),
.left(op_L),
.mid(op_C),
.right(op_R),
.move(brain_cmd)
);

// ---------------------------------------------------------
// 4. MOTOR COMMAND MUX WITH MODE CONTROL
// ---------------------------------------------------------
wire [2:0] uturn_cmd;
wire uturn_active;
reg [2:0] final_move_cmd;

always @(*) begin
    if (!system_ready) begin
        final_move_cmd = 3'b000;
    end else begin
        case (master_mode)
            MODE_MAZE_SOLVING: begin
                final_move_cmd = brain_cmd;
            end
            
            MODE_SERVO_OPERATION: begin
                final_move_cmd = 3'b000;  // STOP during servo operation
            end
            
            MODE_UTURN_AFTER_SERVO: begin
                final_move_cmd = 3'b101;  // ← SPECIAL CODE: Post-servo U-turn (no adjust)
            end
            
            default: final_move_cmd = 3'b000;
        endcase
    end
end

// ---------------------------------------------------------
// 4.5 SERVO GATE CONTROLLER
// ---------------------------------------------------------
wire servo_trigger;
assign servo_trigger = system_ready && (master_mode == MODE_SERVO_OPERATION);

task5noservo u_servo (
    .clk(clk_50M),
    .rst_n(rst_n),
    .ir_sensor(servo_trigger),
    .dht_dat(dht_dat),              // Connect DHT11
    .adc_dout(adc_dout),            // Connect ADC
    .adc_cs_n(adc_cs_n),
    .adc_din(adc_din),
    .adc_sck(adc_sck),
    .bt_rx_pin(bt_rx_pin),          // Connect Bluetooth RX
    .bt_tx_pin(bt_tx_pin),          // Connect Bluetooth TX
    .led_moisture(led_moisture),    // Connect moisture LEDs
    .servo_pwm1(servo_pwm1),
    .servo_pwm2(servo_pwm2),
    .servo_done(servo_operation_complete),
    .start_received(bt_start_received),  // Output start signal
    .uturn_edge(uturn_rising_edge)       // U-turn completion trigger
);

// ---------------------------------------------------------
// 4.6 U-TURN CONTROLLER
// ---------------------------------------------------------
uturn_controller u_uturn (
    .clk(clk_50M),
    .rst_n(rst_n),
    .start(master_mode == MODE_UTURN_AFTER_SERVO),
    .move_cmd(uturn_cmd),
    .active(uturn_active),
    .complete(uturn_complete)
);

// ---------------------------------------------------------
// 5. Motor Controller with Self-Alignment
// ---------------------------------------------------------
wire [31:0] dbg_left_ticks, dbg_right_ticks;

wire motor_uturn_done;

motor_controller u_motors (
.clk(clk_50M),
.rst_n(rst_n),
.move_cmd(final_move_cmd),

// Add distance inputs for alignment
.dist_L(dist_L),
.dist_R(dist_R),
.dist_C(dist_C),
.op_L(op_L),
.op_R(op_R),

.left_enc_a(left_enc_a), .left_enc_b(left_enc_b),
.right_enc_a(right_enc_a), .right_enc_b(right_enc_b),

.left_in1(left_in1), .left_in2(left_in2), .left_enable(left_en),
.right_in3(right_in3), .right_in4(right_in4), .right_enable(right_en),

.left_tick_count(dbg_left_ticks),
.right_tick_count(dbg_right_ticks),
.led(led),
.uturn_done(motor_uturn_done)
);

endmodule

module motor_controller (
input wire clk,
input wire rst_n,
input wire [2:0] move_cmd,

// Distance inputs for wall alignment
input wire [15:0] dist_L,
input wire [15:0] dist_R,
input wire [15:0] dist_C,
input wire op_L,
input wire op_R,

// Encoder Inputs
input wire left_enc_a, left_enc_b,
input wire right_enc_a, right_enc_b,

// Motor Outputs
output reg left_in1, left_in2,
output wire left_enable,
output reg right_in3, right_in4,
output wire right_enable,

// Debug
// Debug
output reg signed [31:0] left_tick_count,
output reg signed [31:0] right_tick_count,
output wire [7:0] led,
output reg uturn_done  // Signal when U-turn completes
);

// ==========================================
// 1. PWM & CONFIGURATION
// ==========================================
parameter PWM_PERIOD = 16'd5000;
parameter PWM_DUTY = 16'd4300; // 90% Base Duty

// Wall alignment parameters
localparam signed WALL_GAIN = 4;
localparam signed WALL_MAX = 600;
localparam signed RIGHT_WALL_BIAS = 0;

reg [15:0] pwm_cnt;


// Internal Reset
reg sys_rst_n;
reg [3:0] rst_cnt = 0;
always @(posedge clk) begin
if (rst_cnt < 4'd10) begin
rst_cnt <= rst_cnt + 1;
sys_rst_n <= 0;
end
else sys_rst_n <= rst_n;
end

// PWM Generator
always @(posedge clk) begin
if (!sys_rst_n) pwm_cnt <= 0;
else if (pwm_cnt < PWM_PERIOD - 1) pwm_cnt <= pwm_cnt + 1;
else pwm_cnt <= 0;
end

// ==========================================
// 2. STATE & TICK DEFINITIONS
// ==========================================
localparam S_IDLE = 3'd0;
localparam S_FWD_ONLY = 3'd1;
localparam S_TURN_PHASE = 3'd2;
localparam S_POST_TURN_FWD = 3'd3;
localparam S_WAIT_SENSORS = 3'd4;
localparam ADJUST = 3'd5;

localparam TICKS_TURN_LEFT = 32'd390;
localparam TICKS_TURN_RIGHT = 32'd390;
localparam TICKS_TURN_UTURN = 32'd730;

// Forward Normal (Split)
// NOTE: In Right-Only mode, TICKS_FWD_NORMAL_L is effectively ignored for control
localparam TICKS_FWD_NORMAL_L = 32'd1300;
localparam TICKS_FWD_NORMAL_R = 32'd1200;

// Forward Post-Turn (Split)
localparam TICKS_FWD_POST_LEFT_L = 32'd1290;
localparam TICKS_FWD_POST_LEFT_R = 32'd1100;

localparam TICKS_FWD_POST_RIGHT_L = 32'd1290;
localparam TICKS_FWD_POST_RIGHT_R = 32'd1200;

localparam TICKS_FWD_POST_UTURN_L = 32'd900;
localparam TICKS_FWD_POST_UTURN_R = 32'd900;
localparam DELAY_STABILIZE = 32'd25_000_000;

reg [2:0] state;
reg [2:0] latched_cmd;
reg [31:0] target_ticks_L;
reg [31:0] target_ticks_R;
reg [31:0] delay_timer;
reg reset_counters;

// ==========================================
// 3. UPDATED FLAGS (RIGHT ENCODER MASTER)
// ==========================================
wire left_done, right_done, all_done;

// Check Right Encoder against Target
assign right_done = (right_tick_count >= $signed(target_ticks_R)) || (right_tick_count <= -$signed(target_ticks_R));

// ---> CONTROL CHANGE: Force Left to follow Right status <---
// Since we are ignoring left encoder values for control, "left_done" is true whenever "right_done" is true.
assign left_done = right_done;

// System is done when Right is done
assign all_done = right_done;

// ==========================================
// 4. WALL ALIGNMENT CORRECTION
// ==========================================
// ==========================================
// 4. SINGLE-WALL ALIGNMENT CORRECTION
// ==========================================
// ==========================================
// 4. SINGLE-WALL ALIGNMENT WITH DEADBAND
// ==========================================
localparam WALL_MIN = 16'd55;      // Minimum acceptable distance (9cm)
localparam WALL_MAX1 = 16'd65;     // Maximum acceptable distance (11cm)
localparam DEADBAND = 16'd10;      // ±2cm tolerance when centering between walls

reg signed [16:0] alignment_error;
reg wall_following_active;
reg signed [16:0] wall_diff;

// Determine which wall to follow and calculate error
always @(posedge clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        alignment_error <= 0;
        wall_following_active <= 0;
    end
    else if (state == S_FWD_ONLY || state == S_POST_TURN_FWD) begin
        if (op_L && !op_R) begin
            // ===== LEFT WALL ONLY =====
            if (dist_L < WALL_MIN) begin
                // Too close to left wall - move RIGHT
                alignment_error <= $signed({1'b0, dist_L}) - $signed(WALL_MIN);
                wall_following_active <= 1'b1;
            end
            else if (dist_L > WALL_MAX1) begin
                // Too far from left wall - move LEFT
                alignment_error <= $signed({1'b0, dist_L}) - $signed(WALL_MAX1);
                wall_following_active <= 1'b1;
            end
            else begin
                // Within range - no correction needed!
                alignment_error <= 17'd0;
                wall_following_active <= 1'b0;
            end
        end
        else if (!op_L && op_R) begin
            // ===== RIGHT WALL ONLY =====
            if (dist_R < WALL_MIN) begin
                // Too close to right wall - move LEFT
                alignment_error <= $signed(WALL_MIN) - $signed({1'b0, dist_R});
                wall_following_active <= 1'b1;
            end
            else if (dist_R > WALL_MAX1) begin
                // Too far from right wall - move RIGHT
                alignment_error <= $signed(WALL_MAX1) - $signed({1'b0, dist_R});
                wall_following_active <= 1'b1;
            end
            else begin
                // Within range - no correction needed!
                alignment_error <= 17'd0;
                wall_following_active <= 1'b0;
            end
        end
        else if (op_L && op_R) begin
            // ===== BOTH WALLS DETECTED =====
            // Center between them with deadband
            wall_diff = $signed({1'b0, dist_L}) - $signed({1'b0, dist_R});
            
            if (wall_diff > $signed(DEADBAND)) begin
                // Too close to right - move LEFT
                alignment_error <= wall_diff;
                wall_following_active <= 1'b1;
            end
            else if (wall_diff < -$signed(DEADBAND)) begin
                // Too close to left - move RIGHT
                alignment_error <= wall_diff;
                wall_following_active <= 1'b1;
            end
            else begin
                // Centered well enough - no correction!
                alignment_error <= 17'd0;
                wall_following_active <= 1'b0;
            end
        end
        else begin
            // No walls detected - no alignment correction
            alignment_error <= 17'd0;
            wall_following_active <= 1'b0;
        end
    end
    else begin
        alignment_error <= 0;
        wall_following_active <= 0;
    end
end

// Calculate correction with gain and clamping
wire signed [31:0] wall_corr_raw;
assign wall_corr_raw = alignment_error * WALL_GAIN;

wire signed [31:0] wall_corr;
assign wall_corr = (wall_corr_raw > WALL_MAX) ? WALL_MAX :
                   (wall_corr_raw < -WALL_MAX) ? -WALL_MAX : wall_corr_raw;

// Apply correction when ANY wall is detected during forward motion
wire wall_align_active;
assign wall_align_active = (state == S_FWD_ONLY || state == S_POST_TURN_FWD) && wall_following_active;

// Differential PWM duty cycles
wire [15:0] left_duty, right_duty;
wire [31:0] right_bias;
assign right_bias = (!op_L && op_R && wall_align_active) ? RIGHT_WALL_BIAS : 0;
assign left_duty = wall_align_active ? (PWM_DUTY + wall_corr) : PWM_DUTY;
assign right_duty = wall_align_active ? (PWM_DUTY - wall_corr+right_bias) : PWM_DUTY;

// Generate individual motor PWM signals
wire left_pwm, right_pwm;
assign left_pwm = (pwm_cnt < left_duty);
assign right_pwm = (pwm_cnt < right_duty);

// Enable outputs with alignment
assign left_enable = left_pwm;
assign right_enable = right_pwm;

// ==========================================
// 5. MAIN STATE MACHINE
// ==========================================
always @(posedge clk or negedge sys_rst_n) begin
if (!sys_rst_n) begin
state <= S_IDLE;
target_ticks_L <= 0;
target_ticks_R <= 0;
latched_cmd <= 0;
reset_counters <= 1;
delay_timer <= 0;
uturn_done <= 0;
end else begin
case (state)
S_IDLE: begin
delay_timer <= 0;
uturn_done <= 0;  // Clear the signal
if (move_cmd == 3'b000) begin
reset_counters <= 1;
end else begin
latched_cmd <= move_cmd;
reset_counters <= 0;

if (move_cmd == 3'b001) begin
target_ticks_L <= TICKS_FWD_NORMAL_L;
target_ticks_R <= TICKS_FWD_NORMAL_R;
state <= S_FWD_ONLY;
end
else if ((dist_C > 40 && dist_C < 180)&&(move_cmd != 3'b101)) begin
target_ticks_L <= 15;
target_ticks_R <= 15;
state <= ADJUST;
end
else begin
// Turn Phase
if ((move_cmd == 3'b100)||(move_cmd == 3'b101)) begin
target_ticks_L <= TICKS_TURN_UTURN;
target_ticks_R <= TICKS_TURN_UTURN;
end
else if (move_cmd == 3'b010) begin
target_ticks_L <= TICKS_TURN_LEFT;
target_ticks_R <= TICKS_TURN_LEFT;
end
else begin
target_ticks_L <= TICKS_TURN_RIGHT;
target_ticks_R <= TICKS_TURN_RIGHT;
end

state <= S_TURN_PHASE;
end
end
end

ADJUST:begin
if (all_done)begin
reset_counters <= 1;
state <= S_WAIT_SENSORS;
end
else reset_counters <=0;
end

S_FWD_ONLY: begin
if (all_done) begin
reset_counters <= 1;
state <= S_WAIT_SENSORS;
end
else reset_counters <= 0;
end

S_TURN_PHASE: begin
if (all_done) begin
reset_counters <= 1;

if (latched_cmd == 3'b100) begin
target_ticks_L <= TICKS_FWD_POST_UTURN_L;
target_ticks_R <= TICKS_FWD_POST_UTURN_R;
end
else if (latched_cmd == 3'b010) begin
target_ticks_L <= TICKS_FWD_POST_LEFT_L;
target_ticks_R <= TICKS_FWD_POST_LEFT_R;
end
else begin
target_ticks_L <= TICKS_FWD_POST_RIGHT_L;
target_ticks_R <= TICKS_FWD_POST_RIGHT_R;
end

state <= S_POST_TURN_FWD;
end else reset_counters <= 0;
end

S_POST_TURN_FWD: begin
if (all_done) begin
reset_counters <= 1;
// Signal U-turn completion if it was a U-turn
if (latched_cmd == 3'b100 || latched_cmd == 3'b101) begin
    uturn_done <= 1;
end
state <= S_WAIT_SENSORS;
end else reset_counters <= 0;
end

S_WAIT_SENSORS: begin
reset_counters <= 1;

if (delay_timer < DELAY_STABILIZE) begin
delay_timer <= delay_timer + 1;
end else begin
delay_timer <= 0;
state <= S_IDLE;
end
end

default: state <= S_IDLE;
endcase
end
end

// ==========================================
// 6. MOTOR DRIVER (Outputs)
// ==========================================
always @(posedge clk) begin
if (!sys_rst_n) begin
left_in1 <= 0; left_in2 <= 0;
right_in3 <= 0; right_in4 <= 0;
end else begin

if (state == S_IDLE || state == S_WAIT_SENSORS || (all_done && !reset_counters)) begin
left_in1 <= 1; left_in2 <= 1;
right_in3 <= 1; right_in4 <= 1;
end

else if (state == S_FWD_ONLY || state == S_POST_TURN_FWD || state == ADJUST) begin
// Left Motor mimics Right Status
if (!right_done) {left_in1, left_in2} <= 2'b10;
else {left_in1, left_in2} <= 2'b11;

if (!right_done) {right_in3, right_in4} <= 2'b10;
else {right_in3, right_in4} <= 2'b11;
end

else if (state == S_TURN_PHASE) begin
case (latched_cmd)
3'b010: begin // LEFT
if (!right_done) begin
{left_in1, left_in2} <= 2'b10;
{right_in3, right_in4} <= 2'b01;
end else begin
{left_in1, left_in2} <= 2'b11;
{right_in3, right_in4} <= 2'b11;
end
end
3'b011: begin // RIGHT
if (!right_done) begin
{left_in1, left_in2} <= 2'b01;
{right_in3, right_in4} <= 2'b10;
end else begin
{left_in1, left_in2} <= 2'b11;
{right_in3, right_in4} <= 2'b11;
end
end
3'b100 , 3'b101 : begin // UTURN
if (!right_done) begin
{left_in1, left_in2} <= 2'b01;
{right_in3, right_in4} <= 2'b10;
end else begin
{left_in1, left_in2} <= 2'b11;
{right_in3, right_in4} <= 2'b11;
end
end
default: begin
{left_in1, left_in2} <= 2'b11;
{right_in3, right_in4} <= 2'b11;
end
endcase
end
end
end

// ==========================================
// 7. ENCODER LOGIC (Kept for Debugging Only)
// ==========================================
// Note: The left logic below generates 'left_tick_count' for debug
// but this variable is no longer used in state machine logic.
reg [1:0] la_sync, lb_sync;
reg la_last;

always @(posedge clk or negedge sys_rst_n) begin
if (!sys_rst_n) begin
la_sync<=0; lb_sync<=0; la_last<=0; left_tick_count <= 0;
end else begin
if (reset_counters) left_tick_count <= 0;
else begin
la_sync <= {la_sync[0], left_enc_a};
lb_sync <= {lb_sync[0], left_enc_b};
if (la_sync[1] && !la_last) begin
if (!lb_sync[1]) left_tick_count <= left_tick_count + 1;
else left_tick_count <= left_tick_count - 1;
end
la_last <= la_sync[1];
end
end
end

reg [1:0] ra_sync, rb_sync;
reg ra_last;

always @(posedge clk or negedge sys_rst_n) begin
if (!sys_rst_n) begin
ra_sync<=0; rb_sync<=0; ra_last<=0; right_tick_count <= 0;
end else begin
if (reset_counters) right_tick_count <= 0;
else begin
ra_sync <= {ra_sync[0], right_enc_a};
rb_sync <= {rb_sync[0], right_enc_b};
if (ra_sync[1] && !ra_last) begin
if (!rb_sync[1]) right_tick_count <= right_tick_count + 1;
else right_tick_count <= right_tick_count - 1;
end
ra_last <= ra_sync[1];
end
end
end

// ==========================================
// 8. DEBUG DISPLAY (LEDs)
// ==========================================
assign led[0] = (state == S_TURN_PHASE);
assign led[1] = (state == S_POST_TURN_FWD);
assign led[2] = right_done; // Debug: Lights up when Right Target Reached
assign led[3] = wall_align_active;
assign led[7] = (move_cmd != 0); // Active indicator

endmodule
// [Rest of modules remain unchanged]
module frequency_scaling (
input clk_50M,
output reg clk_3125KHz
);

initial begin
clk_3125KHz = 0;
end

reg [2:0]counter = 0;

always @ (posedge clk_50M) begin
if (!counter) clk_3125KHz = ~clk_3125KHz;
counter <= counter + 1'd1;
end

endmodule

// ============================================================================
// MODULE: pwm_generator
// ----------------------------------------------------------------------------
// DESCRIPTION:
// This module generates two distinct signals from a 3.125 MHz input clock:
// 1. A secondary reference clock of 195 KHz.
// 2. A Pulse Width Modulated (PWM) signal with a duty cycle controlled by
//    a 4-bit input (0 to 15).
// ============================================================================

module pwm_generator(
input clk_3125KHz,             // High-frequency input clock (3.125 MHz)
input [3:0] duty_cycle,        // 4-bit duty cycle setting (0-15)
output reg clk_195KHz,         // Divided clock output (~195.3 KHz)
output reg pwm_signal          // Resulting PWM output for motor speed control
);

// Initial state for registers on power-up
initial begin
clk_195KHz = 0; pwm_signal = 1;
end

// Internal counters
reg [2:0] counter_195 = 0;     // Counter for clock division (3-bit)
reg [3:0] counter_pwm = 0;     // Counter for PWM period (4-bit, 0-15)

// ---------------------------------------------------------
// MAIN CLOCK LOGIC (Synchronous to 3.125 MHz)
// ---------------------------------------------------------
always @(posedge clk_3125KHz) begin
// 1. CLOCK DIVISION: Generates 195KHz by toggling every 8 cycles (2^3)
if (!counter_195)
clk_195KHz <= ~clk_195KHz;
counter_195 <= counter_195 + 1'd1;

// 2. PWM PERIOD COUNTER: Cycles from 0 up to 15
if (counter_pwm == 4'd15)
counter_pwm <= 0;
else
counter_pwm <= counter_pwm + 1'd1;

// 3. DUTY CYCLE COMPARATOR:
// pwm_signal is HIGH while counter is less than the duty_cycle input

if (counter_pwm < duty_cycle)
pwm_signal <= 1'b1;
else
pwm_signal <= 1'b0;
end

endmodule

// ============================================================================
// MODULE: t2c_maze_explorer
// ----------------------------------------------------------------------------
// DESCRIPTION:
// This module serves as the "Decision Brain" of the robot. 
// It takes binary obstacle detection flags from three ultrasonic sensors 
// and outputs a specific movement command based on the maze environment.
// ============================================================================

module t2c_maze_explorer (
input clk,               // System Clock
input rst_n,             // Active-low Reset
input left, mid, right,  // Obstacle flags (1 = Wall detected, 0 = Path clear)
output reg [2:0] move    // 3-bit movement command to the Motor Controller
);

// ---------------------------------------------------------
// MOVEMENT DECISION LOGIC
// ---------------------------------------------------------
// The case statement evaluates a 3-bit vector: {Left, Mid, Right}
// ---------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
if (!rst_n)
move <= 3'b000;         // Reset to IDLE state
else begin
case ({left,mid ,right})
// Format: {Left_Wall, Center_Wall, Right_Wall}
// 000: No walls detected -> Stay in IDLE or continue current path
3'b000: move <= 3'b000; 

// 001: Wall on Right -> Turn LEFT
3'b001: move <= 3'b010; 

// 010: Wall in Center -> Turn LEFT (Typical maze-solving priority)
3'b010: move <= 3'b010; 

// 011: Wall in Center & Right -> Turn LEFT
3'b011: move <= 3'b010; 

// 100: Wall on Left -> Move FORWARD
3'b100: move <= 3'b001; 

// 101: Walls on Left & Right -> Move FORWARD (Tunnel mode)
3'b101: move <= 3'b001; 

// 110: Walls on Left & Center -> Turn RIGHT
3'b110: move <= 3'b011; 

// 111: Dead end (Walls on all sides) -> Perform U-TURN
3'b111: move <= 3'b100; 

// Default safety case
default: move <= 3'b000;
endcase
end
end

endmodule

// ============================================================================
// MODULE: ultrasonic_array_top
// ----------------------------------------------------------------------------
// DESCRIPTION:
// A wrapper module that instantiates three ultrasonic sensors (Left, Center, Right).
// It manages the parallel operation of the HC-SR04 sensor array.
// ============================================================================

module ultrasonic_array_top (
input clk_50M,                          // 50 MHz System Clock
input reset,                            // Active-low Reset
input echo_L, echo_C, echo_R,           // Echo inputs from physical sensors
output trig_L, trig_C, trig_R,          // Trigger outputs to physical sensors
output op_L, op_C, op_R,                // Obstacle Present flags (Boolean)
output [15:0] dist_L, dist_C, dist_R    // Calculated distance in millimeters
);

// ---------------------------------------------------------
// Left Sensor Instance
// ---------------------------------------------------------
t1b_ultrasonic #(.DELAY_CYCLES(20'd500000)) sensor_left (
.clk_50M(clk_50M), .reset(reset),
.echo_rx(echo_L), .trig(trig_L),
.op(op_L), .distance_out(dist_L)
);

// ---------------------------------------------------------
// Center Sensor Instance
// ---------------------------------------------------------
t1b_ultrasonic #(.DELAY_CYCLES(20'd1000000)) sensor_center (
.clk_50M(clk_50M), .reset(reset),
.echo_rx(echo_C), .trig(trig_C),
.op(op_C), .distance_out(dist_C)
);

// ---------------------------------------------------------
// Right Sensor Instance
// ---------------------------------------------------------
t1b_ultrasonic #(.DELAY_CYCLES(20'd500000)) sensor_right (
.clk_50M(clk_50M), .reset(reset),
.echo_rx(echo_R), .trig(trig_R),
.op(op_R), .distance_out(dist_R)
);

endmodule

// ============================================================================
// MODULE: t1b_ultrasonic
// ----------------------------------------------------------------------------
// DESCRIPTION:
// Individual driver for the HC-SR04 ultrasonic sensor.
// 1. Generates a 10us trigger pulse.
// 2. Measures the duration of the High-level echo signal.
// 3. Converts time to distance using the speed of sound.
// ============================================================================

module t1b_ultrasonic#(
    parameter DELAY_CYCLES = 20'd1000000  // Default 20ms
)(
input clk_50M, reset, echo_rx,          // System inputs
output reg trig,                        // Physical Trigger pulse
output op,                              // Obstacle flag (True if < 180mm)
output wire [15:0] distance_out         // Final distance value
);

// Initialize trigger state
initial begin
trig = 0;
end

// State machine parameters for the Trigger sequence
localparam IDLE=0, WAIT=1, TRIG_ON=2, DELAY=3;
reg [1:0] state = IDLE;
reg [10:0] trig_count=0;                // Counter for 10us pulse width
reg [20:0] echo_count=0;                // Counter for echo high duration
reg [5:0] wait_count=0;                 // Small setup delay
reg [19:0] delay=0;                     // Sampling interval delay
reg [15:0] out_clone=0;                 // Internal register for distance

// Continuous assignments for outputs
assign distance_out=out_clone;
assign op=(out_clone<180);              // Threshold detection at 180mm

// ---------------------------------------------------------
// TRIGGER FINITE STATE MACHINE (FSM)
// ---------------------------------------------------------
always@(posedge clk_50M or negedge reset) begin
if(!reset) begin
state<=IDLE;
trig_count<=0;
trig<=0;
end
else begin
case(state)
IDLE:begin
trig_count<=0;
trig<=0;
wait_count<=0;
state<=WAIT;
end

// Brief wait state to ensure stability
WAIT:begin
wait_count<=wait_count+1;
if(wait_count>=49) begin
wait_count<=0;
state<=TRIG_ON;
end
end

// Generates the required 10us pulse (500 cycles at 50MHz)
TRIG_ON:begin
if(trig_count<500) begin
trig<=1;
trig_count<=trig_count+1;
end
else begin
trig<=0;
trig_count<=0;
state<=DELAY;
end
end

// Delay state to prevent overlapping signals (approx 20ms)
DELAY:begin
if (delay<= DELAY_CYCLES) begin
delay<=delay+1;
end
else begin
delay<=0;
state<=IDLE;
end
end
endcase
end
end

// ---------------------------------------------------------
// ECHO MEASUREMENT & DISTANCE CALCULATION
// ---------------------------------------------------------
// Measurement Formula: Distance = (Time * Speed of Sound) / 2
// Speed of sound = 340 m/s or 0.34 mm/us
// ---------------------------------------------------------

always@(posedge clk_50M or negedge reset) begin
if(!reset) begin
echo_count<=0;
out_clone<=0;
end
else begin
if (echo_rx) begin
// Increment counter while echo signal is HIGH
echo_count<=echo_count+1;
end
else if (echo_count>0) begin
// Convert clock cycles to mm when echo falls to LOW
// (echo_count * clock_period * 340) / 2
out_clone<=(echo_count*339)/100000;
echo_count<=0;
end
end
end

endmodule

module task5noservo (
    input  wire        clk,           // 50MHz
    input  wire        rst_n,         // Manual Reset Input (Active Low)
    input  wire        ir_sensor,     // IR Sensor
    inout  wire        dht_dat,       // DHT11 Sensor Pin
    input  wire        adc_dout,      // From Moisture Sensor
    output wire        adc_cs_n,      // To ADC
    output wire        adc_din,       // To ADC
    output wire        adc_sck,       // To ADC
    input  wire        bt_rx_pin,     // From Bluetooth TXD
    output wire        bt_tx_pin,     // To Bluetooth RXD
    output wire [7:0]  led_moisture,  // LEDs for Moisture level
    output reg         servo_pwm1,    // Servo Signal 1
    output reg         servo_pwm2,    // Servo Signal 2
    output reg         servo_done,    // Pulse HIGH only when ALL tasks are finished
    output reg         start_received,// Signal that "start" command was received
    input  wire        uturn_edge     // U-turn completion trigger for MPI counter
);
    parameter CLK_FREQ = 50000000;
    parameter BAUD_RATE = 115200;
    localparam PWM_PERIOD_TICKS = CLK_FREQ / 50;
    localparam MIN_PULSE1 = CLK_FREQ / 2000;       
    localparam MAX_PULSE1 = (CLK_FREQ * 25) / 10000;
    
    localparam MIN_PULSE2 = CLK_FREQ / 1000;       
    localparam MAX_PULSE2 = 93000;  
    
	 // --- UART RX SIGNALS ---
    wire [7:0] rx_data;
    wire rx_ready;
    
    // Instantiate UART RX
    uart_rx_simple #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) uart_rx_inst (
        .clk(clk),
        .rst_n(global_rst_n),
        .rx(bt_rx_pin),
        .rx_data(rx_data),
        .rx_ready(rx_ready)
    );
	 
	 // --- DETECT "start" COMMAND ---
    reg [3:0] char_index = 0;
    reg start_trigger = 0;
    reg rx_ready_prev = 0;
    
    initial begin
        start_received = 0;
    end
    
    always @(posedge clk) begin
        rx_ready_prev <= rx_ready;
    end
    
    wire rx_edge = rx_ready && !rx_ready_prev;

    always @(posedge clk or negedge global_rst_n) begin
        if (!global_rst_n) begin
            char_index <= 0;
            start_trigger <= 0;
            start_received <= 0;
        end else begin
            start_trigger <= 0;
            
            if (rx_edge) begin
                case (char_index)
                    0: begin
                        if ((rx_data == "s") || (rx_data == "S"))
                            char_index <= 1;
                        else
                            char_index <= 0;
                    end
                    
                    1: begin
                        if ((rx_data == "t") || (rx_data == "T"))
                            char_index <= 2;
                        else if ((rx_data == "s") || (rx_data == "S"))
                            char_index <= 1;
                        else
                            char_index <= 0;
                    end
                    
                    2: begin
                        if ((rx_data == "a") || (rx_data == "A"))
                            char_index <= 3;
                        else if ((rx_data == "s") || (rx_data == "S"))
                            char_index <= 1;
                        else
                            char_index <= 0;
                    end
                    
                    3: begin
                        if ((rx_data == "r") || (rx_data == "R"))
                            char_index <= 4;
                        else if ((rx_data == "s") || (rx_data == "S"))
                            char_index <= 1;
                        else
                            char_index <= 0;
                    end
						  
						  4: begin
                        if ((rx_data == "t") || (rx_data == "T"))
                            char_index <= 5;
                        else if ((rx_data == "s") || (rx_data == "S"))
                            char_index <= 1;
                        else
                            char_index <= 0;
                    end
						  
						  5: begin
                        if ((rx_data == "-"))
                            char_index <= 6;
                        else if ((rx_data == "s") || (rx_data == "S"))
                            char_index <= 1;
                        else
                            char_index <= 0;
                    end
                    
						  6: begin
                        if ((rx_data == "3"))
                            char_index <= 7;
                        else if ((rx_data == "s") || (rx_data == "S"))
                            char_index <= 1;
                        else
                            char_index <= 0;
                    end
						  
						  7: begin
                        if ((rx_data == "-"))
                            char_index <= 8;
                        else if ((rx_data == "s") || (rx_data == "S"))
                            char_index <= 1;
                        else
                            char_index <= 0;
                    end
						  
                    8: begin
                        if ((rx_data == "#")) begin
                            start_trigger <= 1;
                            start_received <= 1;  // Latch the start signal
                            char_index <= 0;
                        end else if ((rx_data == "s") || (rx_data == "S")) begin
                            char_index <= 1;
                        end else begin
                            char_index <= 0;
                        end
                    end
                    
                    default: char_index <= 0;
                endcase
            end
        end
    end
	 
    // --- INTERNAL SIGNALS ---
    reg [31:0] pwm_counter = 0;
    reg [31:0] pulse_width1 = MIN_PULSE1;   
    reg [31:0] pulse_width2 = MIN_PULSE2;
    reg [2:0]  state_counter = 0;
    reg [31:0] move_timer = 0;
    reg        trigger_msg = 0;    
    reg        sys_rst_n_int = 0; 
    reg [15:0] p_delay = 0;
	 reg [7:0] mpi_count = 1;  // MPI counter starting from 1

    wire global_rst_n = sys_rst_n_int && rst_n;

    // Sensor Data Latches
    wire [11:0] moisture_live; 
    wire [7:0]  T_live, RH_live;
    reg  [11:0] moisture_safe; 
    reg  [7:0]  T_safe, RH_safe;
    wire [15:0] moisture_bcd, T_bcd_full, RH_bcd_full;

    // --- 1. RESET & PWM GENERATOR ---
    always @(posedge clk) begin
        if (p_delay < 16'hFFFF) begin p_delay <= p_delay + 1; sys_rst_n_int <= 0; end
        else sys_rst_n_int <= 1;

        if (!global_rst_n) begin
            pwm_counter <= 0;
            servo_pwm1 <= 0;
            servo_pwm2 <= 0;
        end else begin
            if (pwm_counter < PWM_PERIOD_TICKS - 1) pwm_counter <= pwm_counter + 1;
            else pwm_counter <= 0;
            servo_pwm1 <= (pwm_counter < pulse_width1);
            servo_pwm2 <= (pwm_counter < pulse_width2);
        end
    end

    // --- EDGE DETECTION LOGIC ---
    reg ir_prev;
    always @(posedge clk) begin
        ir_prev <= ir_sensor;
    end
    wire ir_rising_edge = (ir_sensor && !ir_prev);

    // --- 2. MAIN CONTROL (REVISED FOR COMPLETION SIGNAL) ---
    reg has_sent = 0; 

    always @(posedge clk or negedge global_rst_n) begin
        if (!global_rst_n) begin
            state_counter <= 0; 
            move_timer <= 0; 
            trigger_msg <= 0;
            has_sent <= 0;
            servo_done <= 0;
            pulse_width1 <= MIN_PULSE1;
            pulse_width2 <= MIN_PULSE2;
            mpi_count <= 1;
        end else begin
            // Increment MPI counter on U-turn completion
            if (uturn_edge) begin
                mpi_count <= mpi_count + 1;
            end
            case (state_counter)
                0: begin // IDLE
                    pulse_width1 <= MIN_PULSE1;
                    pulse_width2 <= MIN_PULSE2;
                    move_timer <= 0;
                    trigger_msg <= 0;
                    has_sent <= 0; 
                    servo_done <= 0; // Ensure done is low in IDLE
                    if (ir_rising_edge) state_counter <= 1;
                end

                1: begin // TASK START: Open and Measure
                    if (pwm_counter == 0) begin
                        if (pulse_width1 < MAX_PULSE1) pulse_width1 <= pulse_width1 + 500;
                        if ((pulse_width2 < MAX_PULSE2) && (pulse_width1 >= 75000)) pulse_width2 <= pulse_width2 + 500;
                    end

                    if ((pulse_width1 >= MAX_PULSE1) && (pulse_width2 >= MAX_PULSE2)) begin
                        if (!has_sent) begin
                            trigger_msg <= 1;
                            has_sent <= 1;
                        end else begin
                            trigger_msg <= 0;
                        end

                        if (move_timer < CLK_FREQ * 5) move_timer <= move_timer + 1;
                        else state_counter <= 2; 
                    end
                end

                2: begin // CLOSING: Returning to start
                    if (pwm_counter == 0) begin
                        if (pulse_width1 > MIN_PULSE1) pulse_width1 <= pulse_width1 - 500;
                        if ((pulse_width2 > MIN_PULSE2) && (pulse_width1 <= 115000)) pulse_width2 <= pulse_width2 - 500;
                    end
                    
                    // IF FULLY CLOSED AND RETURNED
                    if ((pulse_width1 <= MIN_PULSE1) && (pulse_width2 <= MIN_PULSE2)) begin
                        servo_done <= 1;    // Pulse HIGH to signal absolute completion
                        state_counter <= 0; // Go back to IDLE
                    end
                end
                
                default: state_counter <= 0;
            endcase
        end
    end

    // --- 3. SENSOR & BCD INSTANCES ---
    moisture_sensor m_inst(.dout(adc_dout), .clk50(clk), .adc_cs_n(adc_cs_n), .din(adc_din), .adc_sck(adc_sck), .d_out_ch0(moisture_live), .led_ind(led_moisture));
    t2a_dht d_inst(.clk_50M(clk), .reset(global_rst_n), .sensor(dht_dat), .T_integral(T_live), .RH_integral(RH_live), .data_valid(), .debug_leds());
    bin2bcd_12bit bcd_moist (.bin_in(moisture_safe), .bcd_out(moisture_bcd));
    bin2bcd_12bit bcd_temp  (.bin_in({4'b0000, T_safe}), .bcd_out(T_bcd_full));
    bin2bcd_12bit bcd_rh    (.bin_in({4'b0000, RH_safe}), .bcd_out(RH_bcd_full));

    // --- 4. UART DATA SENDING LOGIC ---
    reg [7:0] tx_data;
    reg tx_start;
    wire tx_ready;
    uart_tx_driver #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) bt_uart (
        .clk(clk), .rst_n(global_rst_n), .start(tx_start), .data(tx_data), .tx(bt_tx_pin), .ready(tx_ready)
    );

    reg [6:0] state = 0; 
    
    function [7:0] hex2ascii;
        input [3:0] nibble;
        begin hex2ascii = (nibble < 10) ? (nibble + 8'h30) : (nibble + 8'h37); end
    endfunction

    always @(posedge clk or negedge global_rst_n) begin
        if (!global_rst_n) state <= 0;
        else begin
            tx_start <= 0;
            case (state)
                0: if (trigger_msg) begin 
                        moisture_safe <= moisture_live;
                        T_safe        <= T_live;
                        RH_safe       <= RH_live;
                        state <= 1;
                   end
                1:  if (tx_ready) begin tx_data <= "M"; tx_start <= 1; state <= 2; end
                2:  if (!tx_ready) state <= 3;
                3:  if (tx_ready) begin tx_data <= "P"; tx_start <= 1; state <= 4; end
                4:  if (!tx_ready) state <= 5;
                5:  if (tx_ready) begin tx_data <= "I"; tx_start <= 1; state <= 6; end
                6:  if (!tx_ready) state <= 7;
                7:  if (tx_ready) begin tx_data <= "M"; tx_start <= 1; state <= 8; end
                8:  if (!tx_ready) state <= 9;
                9:  if (tx_ready) begin tx_data <= "-"; tx_start <= 1; state <= 10; end
                10: if (!tx_ready) state <= 11;
                11: if (tx_ready) begin tx_data <= hex2ascii(mpi_count[3:0]); tx_start <= 1; state <= 12; end 
                12: if (!tx_ready) state <= 13;
                13: if (tx_ready) begin tx_data <= "-"; tx_start <= 1; state <= 14; end
                14: if (!tx_ready) state <= 15;
                15: if (tx_ready) begin tx_data <= "#"; tx_start <= 1; state <= 16; end
                16: if (!tx_ready) state <= 17;
                17: if (tx_ready) begin tx_data <= 8'h0A; tx_start <= 1; state <= 18; end 
                18: if (!tx_ready) state <= 19;
                19: if (tx_ready) begin tx_data <= "M"; tx_start <= 1; state <= 20; end
                20: if (!tx_ready) state <= 21;
                21: if (tx_ready) begin tx_data <= "M"; tx_start <= 1; state <= 22; end
                22: if (!tx_ready) state <= 23;
                23: if (tx_ready) begin tx_data <= "-"; tx_start <= 1; state <= 24; end
                24: if (!tx_ready) state <= 25;
                25: if (tx_ready) begin tx_data <= hex2ascii(mpi_count[3:0]); tx_start <= 1; state <= 26; end
                26: if (!tx_ready) state <= 27;
                27: if (tx_ready) begin tx_data <= "-"; tx_start <= 1; state <= 28; end
                28: if (!tx_ready) state <= 29;
                29: if (tx_ready) begin 
                        tx_data <= (moisture_safe < 900) ? "M" : "D"; 
                        tx_start <= 1; state <= 30; 
                    end
                30: if (!tx_ready) state <= 31;
                31: if (tx_ready) begin tx_data <= "-"; tx_start <= 1; state <= 32; end
                32: if (!tx_ready) state <= 33;
                33: if (tx_ready) begin tx_data <= "#"; tx_start <= 1; state <= 34; end
                34: if (!tx_ready) state <= 35;
                35: if (tx_ready) begin tx_data <= 8'h0A; tx_start <= 1; state <= 36; end
                36: if (!tx_ready) state <= 37;
                37: if (tx_ready) begin tx_data <= "T"; tx_start <= 1; state <= 38; end
                38: if (!tx_ready) state <= 39;
                39: if (tx_ready) begin tx_data <= "H"; tx_start <= 1; state <= 40; end
                40: if (!tx_ready) state <= 41;
                41: if (tx_ready) begin tx_data <= "-"; tx_start <= 1; state <= 42; end
                42: if (!tx_ready) state <= 43;
                43: if (tx_ready) begin tx_data <= hex2ascii(mpi_count[3:0]); tx_start <= 1; state <= 44; end
                44: if (!tx_ready) state <= 45;
                45: if (tx_ready) begin tx_data <= "-"; tx_start <= 1; state <= 46; end
                46: if (!tx_ready) state <= 47;
                47: if (tx_ready) begin tx_data <= hex2ascii(T_bcd_full[7:4]); tx_start <= 1; state <= 48; end
                48: if (!tx_ready) state <= 49;
                49: if (tx_ready) begin tx_data <= hex2ascii(T_bcd_full[3:0]); tx_start <= 1; state <= 50; end
                50: if (!tx_ready) state <= 51;
                51: if (tx_ready) begin tx_data <= "-"; tx_start <= 1; state <= 52; end
                52: if (!tx_ready) state <= 53;
                53: if (tx_ready) begin tx_data <= hex2ascii(RH_bcd_full[7:4]); tx_start <= 1; state <= 54; end
                54: if (!tx_ready) state <= 55;
                55: if (tx_ready) begin tx_data <= hex2ascii(RH_bcd_full[3:0]); tx_start <= 1; state <= 56; end
                56: if (!tx_ready) state <= 57;
                57: if (tx_ready) begin tx_data <= "-"; tx_start <= 1; state <= 58; end
                58: if (!tx_ready) state <= 59;
                59: if (tx_ready) begin tx_data <= "#"; tx_start <= 1; state <= 60; end
                60: if (!tx_ready) state <= 61;
                61: if (tx_ready) begin tx_data <= 8'h0A; tx_start <= 1; state <= 62; end
                62: if (!tx_ready) state <= 0; 
                default: state <= 0;
            endcase
        end
    end
endmodule

// Submodules (bin2bcd_12bit, uart_tx_driver, moisture_sensor, adc_controller, t2a_dht)
// remain exactly the same as in your original logic.

// (Submodules bin2bcd_12bit, uart_tx_driver, moisture_sensor, adc_controller, and t2a_dht remain identical as provided)

// ==============================================================================
// SUBMODULE: Binary to BCD Converter
// ==============================================================================
module bin2bcd_12bit(
    input [11:0] bin_in,
    output reg [15:0] bcd_out 
);
    integer i;
    always @(bin_in) begin
        bcd_out = 0; 
        for (i = 11; i >= 0; i = i - 1) begin
            if (bcd_out[3:0] >= 5)   bcd_out[3:0]   = bcd_out[3:0] + 3;
            if (bcd_out[7:4] >= 5)   bcd_out[7:4]   = bcd_out[7:4] + 3;
            if (bcd_out[11:8] >= 5)  bcd_out[11:8]  = bcd_out[11:8] + 3;
            if (bcd_out[15:12] >= 5) bcd_out[15:12] = bcd_out[15:12] + 3;
            bcd_out = {bcd_out[14:0], bin_in[i]};
        end
    end
endmodule

// ==============================================================================
// SUBMODULE: UART TRANSMITTER
// ==============================================================================
module uart_tx_driver #(parameter CLK_FREQ = 50000000, parameter BAUD_RATE = 115200) (
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [7:0] data,
    output reg tx,
    output wire ready
);
    localparam WAIT_COUNT = CLK_FREQ / BAUD_RATE;
    reg [31:0] bit_timer = 0;
    reg [3:0]  bit_index = 0;
    reg [9:0]  shift_reg = 0;
    reg        active = 0;
    assign ready = !active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx <= 1; active <= 0; bit_timer <= 0;
        end else begin
            if (!active) begin
                tx <= 1'b1; 
                if (start) begin
                    shift_reg <= {1'b1, data, 1'b0}; 
                    bit_timer <= 0; bit_index <= 0; active <= 1;
                end
            end else begin
                if (bit_timer < WAIT_COUNT - 1) bit_timer <= bit_timer + 1;
                else begin
                    bit_timer <= 0; tx <= shift_reg[0]; shift_reg <= {1'b1, shift_reg[9:1]}; 
                    if (bit_index == 9) active <= 0;
                    else bit_index <= bit_index + 1;
                end
            end
        end
    end
endmodule

// ==============================================================================
// SUBMODULE: MOISTURE SENSOR & ADC (Logic Inverted for Intuition)
// ==============================================================================
module moisture_sensor(
    input dout, clk50,
    output adc_cs_n, din, adc_sck,
    output [11:0] d_out_ch0,
    output [7:0] led_ind
);
    reg [3:0] counter;
    always @(posedge clk50) counter <= counter + 1;
    assign adc_sck = counter[3]; 

    adc_controller adc_inst(
        .dout(dout), .adc_sck(adc_sck), .adc_cs_n(adc_cs_n), .din(din),
        .d_out_ch0(d_out_ch0), .led_ind(led_ind)
    );
endmodule

module adc_controller(
    input dout, adc_sck,
    output adc_cs_n, din, 
    output reg [11:0] d_out_ch0,
    output reg [7:0] led_ind
);
    parameter MIN = 500;  // Adjusted for typical wet soil
    parameter MAX = 3500; // Adjusted for dry air
    parameter STEP = (MAX - MIN) / 8;

    reg [3:0] din_counter = 0;
    reg [3:0] sp_counter = 0;
    reg adc_cs = 1;
    reg din_temp = 0;      
    reg [11:0] dout_chx = 0;
      
    always @(negedge adc_sck) begin
        din_counter <= din_counter + 1;
        if(din_counter == 0) adc_cs <= !adc_cs;
    end

    always @(posedge adc_sck) begin
        if((sp_counter >= 4) && (sp_counter <= 15)) dout_chx[15 - sp_counter] <= dout; 
        else dout_chx <= 0; 
        sp_counter <= sp_counter + 1'b1;
    end

    always @(posedge adc_sck ) begin
        if ((sp_counter == 15)&& (!adc_cs)) d_out_ch0 <= dout_chx;
    end

    // INVERTED LED LOGIC:
    // Low Value (Wet) = More LEDs
    // High Value (Dry) = Less LEDs
    always @(*) begin
        integer count;
        if (d_out_ch0 < MIN) count = 8;        // Very wet -> All LEDs
        else if (d_out_ch0 >= MAX) count = 0;  // Very dry -> No LEDs
        else count = 8 - ((d_out_ch0 - MIN) / STEP); // Invert scale
        
        if (count == 0) led_ind = 8'b00000000;
        else led_ind = (8'hFF >> (8 - count));  
    end
    assign adc_cs_n = adc_cs;
    assign din = din_temp;
endmodule

// ==============================================================================
// SUBMODULE: DHT11 SENSOR
// ==============================================================================
module t2a_dht(
    input clk_50M, input reset, inout sensor,
    output reg [7:0] T_integral, output reg [7:0] RH_integral,
    output reg [7:0] T_decimal, output reg [7:0] RH_decimal,
    output reg [7:0] Checksum, output reg data_valid, output reg [3:0] debug_leds
);
    initial begin
        T_integral = 0; RH_integral = 0; T_decimal = 0; RH_decimal = 0;
        Checksum = 0; data_valid = 0; debug_leds = 0;
    end
   
    localparam IDLE=0, START_LOW=1, START_HIGH=2, RESPONSE_LOW=3, RESPONSE_HIGH=4, 
               READ_BIT_LOW=5, READ_BIT_HIGH=6, PROCESS_DATA=7, WAIT_RETRY=8;
    localparam CNT_18MS=1000000, TIMEOUT_MAX=100000, WAIT_2S=100000000, THRESHOLD=2400; 

    reg [3:0] state; reg [27:0] counter; reg [5:0] bit_index;
    reg [39:0] data_buffer; reg s_sync1, s_sync2, sensor_prev, drive_low;             

    assign sensor = drive_low ? 1'b0 : 1'bz;
    always @(posedge clk_50M) begin
        s_sync1 <= sensor; s_sync2 <= s_sync1;    
        if (!reset) sensor_prev <= 1'b1; else sensor_prev <= s_sync2; 
    end
    wire sensor_clean = s_sync2;
    wire posedge_sensor = (~sensor_prev & sensor_clean);
    wire negedge_sensor = (sensor_prev & ~sensor_clean);

    always @(posedge clk_50M) begin
        if (!reset) begin state <= IDLE; counter <= 0; drive_low <= 0; end 
        else begin
            if (state != PROCESS_DATA) data_valid <= 0; 
            case (state)
                IDLE: begin drive_low <= 0; counter <= 0; state <= WAIT_RETRY; end
                START_LOW: begin
                    drive_low <= 1; 
                    if (counter >= CNT_18MS) begin drive_low <= 0; counter <= 0; state <= START_HIGH; end 
                    else counter <= counter + 1;
                end
                START_HIGH: begin
                    drive_low <= 0;
                    if (negedge_sensor) begin state <= RESPONSE_LOW; counter <= 0; end 
                    else if (counter > TIMEOUT_MAX) state <= WAIT_RETRY; 
                    else counter <= counter + 1;
                end
                RESPONSE_LOW: begin
                    if (posedge_sensor) begin state <= RESPONSE_HIGH; counter <= 0; end 
                    else if (counter > TIMEOUT_MAX) state <= WAIT_RETRY; 
                    else counter <= counter + 1;
                end
                RESPONSE_HIGH: begin
                    if (negedge_sensor) begin state <= READ_BIT_LOW; counter <= 0; bit_index <= 0; end 
                    else if (counter > TIMEOUT_MAX) state <= WAIT_RETRY; 
                    else counter <= counter + 1;
                end
                READ_BIT_LOW: begin
                    if (posedge_sensor) begin state <= READ_BIT_HIGH; counter <= 0; end 
                    else if (counter > TIMEOUT_MAX) state <= WAIT_RETRY; 
                    else counter <= counter + 1;
                end
                READ_BIT_HIGH: begin
                    if (sensor_clean == 1) begin
                        counter <= counter + 1;
                        if (counter > 5000) state <= WAIT_RETRY; 
                    end else begin 
                        if (counter > THRESHOLD) data_buffer[39 - bit_index] <= 1'b1;
                        else data_buffer[39 - bit_index] <= 1'b0;
                        bit_index <= bit_index + 1;
                        if (bit_index == 39) state <= PROCESS_DATA;
                        else begin state <= READ_BIT_LOW; counter <= 0; end
                    end
                end
                PROCESS_DATA: begin
                    if (data_buffer[7:0] == (data_buffer[39:32] + data_buffer[31:24] + data_buffer[23:16] + data_buffer[15:8])) begin
                        RH_integral <= data_buffer[39:32]; RH_decimal <= data_buffer[31:24];
                        T_integral <= data_buffer[23:16]; T_decimal <= data_buffer[15:8];
                        Checksum <= data_buffer[7:0]; data_valid <= 1'b1;
                    end
                    state <= WAIT_RETRY; counter <= 0;
                end
                WAIT_RETRY: begin
                    if (counter >= WAIT_2S) begin state <= START_LOW; counter <= 0; end 
                    else counter <= counter + 1;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule

// ============================================================
// U-TURN CONTROLLER MODULE
// ============================================================
module uturn_controller (
    input wire clk,
    input wire rst_n,
    input wire start,
    output reg [2:0] move_cmd,
    output reg active,
    output reg complete
);

    reg prev_start;
    wire start_edge;
    
    always @(posedge clk) prev_start <= start;
    assign start_edge = start && !prev_start;
    
    localparam UTURN_IDLE = 2'd0;
    localparam UTURN_EXECUTE = 2'd1;
    localparam UTURN_WAIT = 2'd2;
    
    reg [1:0] uturn_state;
    reg [31:0] wait_counter;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uturn_state <= UTURN_IDLE;
            move_cmd <= 3'b000;
            active <= 0;
            complete <= 0;
            wait_counter <= 0;
        end else begin
            case (uturn_state)
                UTURN_IDLE: begin
                    move_cmd <= 3'b000;
                    active <= 0;
                    complete <= 0;
                    wait_counter <= 0;
                    
                    if (start_edge) begin
                        uturn_state <= UTURN_EXECUTE;
                        move_cmd <= 3'b100;
                        active <= 1;
                    end
                end
                
                UTURN_EXECUTE: begin
                    move_cmd <= 3'b100;
                    
                    if (wait_counter < 50_000_000) begin
                        wait_counter <= wait_counter + 1;
                    end else begin
                        uturn_state <= UTURN_WAIT;
                        move_cmd <= 3'b000;
                        wait_counter <= 0;
                    end
                end
                
                UTURN_WAIT: begin
                    move_cmd <= 3'b000;
                    
                    if (wait_counter < 10_000_000) begin  // Hold complete signal for 0.2 seconds
                        wait_counter <= wait_counter + 1;
                        complete <= 1;
                    end else begin
                        complete <= 0;
                        if (!start) begin
                            uturn_state <= UTURN_IDLE;
                            active <= 0;
                            wait_counter <= 0;
                        end
                    end
                end
                    
                default: uturn_state <= UTURN_IDLE;
            endcase
        end
    end
endmodule

// ==============================================================================
// SIMPLE UART RECEIVER
// ==============================================================================
module uart_rx_simple #(
    parameter CLK_FREQ = 50000000,
    parameter BAUD_RATE = 115200
)(
    input wire clk,
    input wire rst_n,
    input wire rx,
    output reg [7:0] rx_data,
    output reg rx_ready
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;
    
    reg [1:0] rx_sync;
    reg [15:0] clk_count;
    reg [2:0] bit_index;
    reg [7:0] rx_byte;
    reg [2:0] state;
    
    localparam IDLE  = 3'd0;
    localparam START = 3'd1;
    localparam DATA  = 3'd2;
    localparam STOP  = 3'd3;
    
    // Synchronize RX input
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync <= 2'b11;
        end else begin
            rx_sync <= {rx_sync[0], rx};
        end
    end
    
    wire rx_line = rx_sync[1];
    
    // UART state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            clk_count <= 0;
            bit_index <= 0;
            rx_data <= 0;
            rx_ready <= 0;
            rx_byte <= 0;
        end else begin
            rx_ready <= 0;
            
            case (state)
                IDLE: begin
                    clk_count <= 0;
                    bit_index <= 0;
                    
                    if (rx_line == 0) begin // Start bit
                        state <= START;
                    end
                end
                
                START: begin
                    if (clk_count == (CLKS_PER_BIT / 2)) begin
                        if (rx_line == 0) begin
                            clk_count <= 0;
                            state <= DATA;
                        end else begin
                            state <= IDLE;
                        end
                    end else begin
                        clk_count <= clk_count + 1;
                    end
                end
                
                DATA: begin
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1;
                    end else begin
                        clk_count <= 0;
                        rx_byte[bit_index] <= rx_line;
                        
                        if (bit_index < 7) begin
                            bit_index <= bit_index + 1;
                        end else begin
                            bit_index <= 0;
                            state <= STOP;
                        end
                    end
                end
                
                STOP: begin
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1;
                    end else begin
                        clk_count <= 0;
                        rx_data <= rx_byte;
                        rx_ready <= 1;
                        state <= IDLE;
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end

endmodule