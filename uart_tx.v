// MazeSolver Bot: Task 2B - UART Transmitter
/*
Instructions
-------------------
Students are not allowed to make any changes in the Module declaration.

This file is used to generate UART Tx data packet to transmit the messages based on the input data.

Recommended Quartus Version : 20.1
The submitted project file must be 20.1 compatible as the evaluation will be done on Quartus Prime Lite 20.1.

Warning: The error due to compatibility will not be entertained.
-------------------
*/

/*
Module UART Transmitter

Input:  clk_3125 - 3125 KHz clock
        parity_type - even(0)/odd(1) parity type
        tx_start - signal to start the communication.
        data    - 8-bit data line to transmit

Output: tx      - UART Transmission Line
        tx_done - message transmitted flag


        Baudrate : 115200 bps
*/

// module declaration
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
//////////////////DO NOT MAKE ANY CHANGES ABOVE THIS LINE//////////////////
 
parameter 	IDLE = 0, START = 1, MESSAGE = 2, PARITY = 3, END = 4;
reg [2:0] state = IDLE;
reg [4:0] count = 0;
reg [2:0] index = 7;

wire calc_parity = (parity_type == 1) ? (~^data) : (^data);
wire [4:0] threshold = (state == START) ? 25 : 26;

always@(posedge clk_3125) begin
	case(state) 
       
		IDLE:begin
			tx_done <= 0;
			if (tx_start == 1) begin
				tx <= 0;
				state <= START;
			end
			else begin
				state <= IDLE;
			end
		end
        
		START:begin
			count <= count + 1;
			if (count == threshold) begin
				count <= 0;
				state <= MESSAGE;
			end
		end
        
		MESSAGE:begin
			tx <= data[index];
			count <= count + 1;
			if (count == threshold) begin
				count <= 0;
				if (index == 0) begin
					state <= PARITY;
				end
				else begin
					index <= index - 1;
					state <= MESSAGE;
				end
			end
		end
        
		PARITY:begin
			tx <= calc_parity;
			count <= count + 1;
			if (count == threshold) begin
				count <= 0;
				state <= END;
			end
		end
        
		END:begin
			tx <= 1;
			count <= count + 1;
			if (count == threshold) begin
				count <= 0;
				index <= 7;
				tx_done <= 1;
				state <= IDLE;
			end
		end

		default: begin
			state <= IDLE;
			tx <= 1;
			tx_done <= 0;
			count <= 0;
			index <= 7;
		end
        
	endcase
end
	
//////////////////DO NOT MAKE ANY CHANGES BELOW THIS LINE//////////////////

endmodule

