
module uart_rx(
    input clk_3125,
    input rx,
    output reg [7:0] rx_msg,
    output reg rx_parity,
    output reg rx_complete
    );

initial begin
    rx_msg = 8'b0;
    rx_parity = 1'b0;
    rx_complete = 1'b0;
end
//////////////////DO NOT MAKE ANY CHANGES ABOVE THIS LINE//////////////////

parameter 	IDLE = 0, START = 1, MESSAGE = 2, PARITY = 3, END = 4;
reg [2:0] state = IDLE;
reg [4:0] count = 0;
reg [2:0] index = 7;
reg [7:0] msg_clone = 0;
reg parity_clone=0;

wire calc_parity = (^msg_clone) ? 1 : 0;
wire [4:0] threshold = (state == START) ? 26 : 27;

always@(posedge clk_3125) begin
	case(state) 
       
		IDLE:begin
			rx_complete <= 0;
			index <= 7;
			if (rx == 0) begin
				count <= count+1;
				if(count == 1) begin
					state <= START;
					count <= 1;
				end
			end
			else begin
				state <= IDLE;
			end
		end
        
		START:begin
			count <= count + 1;
			if (count == threshold) begin
				count <= 1;
				state <= MESSAGE;
			end
		end
        
		MESSAGE:begin
			count <= count + 1;
			if (count == 13) begin
				msg_clone[index] <= rx;
			end
			if (count == threshold) begin
				count <= 1;
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
			count <= count + 1;
			if (count == 13) begin
				parity_clone <= rx;
			end
			if (count == threshold) begin
				count <= 1;
				state <= END;
			end
		end
        
		END:begin
			count <= count + 1;
			if (count == threshold) begin
				count <= 1;
				if (parity_clone == calc_parity) begin
					rx_msg <= msg_clone;
				end
				else begin
					parity_clone <= 0; 
					rx_msg <= "?";
				end
				rx_parity <= parity_clone;
				rx_complete <= 1;
				state <= IDLE;
			end
		end

		default: begin
			state <= IDLE;
			rx_complete <= 0;
			count <= 1;
			index <= 7;
		end
        
	endcase
end 

//////////////////DO NOT MAKE ANY CHANGES BELOW THIS LINE//////////////////

endmodule
