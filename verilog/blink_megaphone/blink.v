`default_nettype none
module top (
	input  clk48,
	output rgb_led0_r,
	output rgb_led0_g,
	output rgb_led0_b,
	output gpio_0,
	input  gpio_a0,
	inout  scl,
	inout  sda,
	input  usr_btn,
	output rst_n
);
	parameter signed [31:0] INPUT_CLK_RATE = 48000000;
	parameter signed [31:0] TARGET_SCL_RATE = 100000;
	parameter [7:0] ADDRESS = 8'h25;
	reg [31:0] counter = 0;
	always @(posedge clk48) counter <= counter + 1;
//	assign rgb_led0_g = ~gpio_a0;
	wire bus_clear;
	reg transfer_start = 1'b0;
	reg transfer_continues = 1'b0;
	reg [7:0] address;
	reg [7:0] data_tx = 8'd0;
	wire transfer_ready;
	wire interrupt;
	wire transaction_complete;
	wire nack;
	wire [7:0] data_rx;
	wire address_err;
	i2c_master #(
		.INPUT_CLK_RATE(INPUT_CLK_RATE),
		.TARGET_SCL_RATE(TARGET_SCL_RATE)
	) i2c_master(
		.scl(scl),
		.clk_in(clk48),
		.bus_clear(bus_clear),
		.sda(sda),
		.address(address),
		.transfer_start(transfer_start),
		.transfer_continues(transfer_continues),
		.data_tx(data_tx),
		.transfer_ready(transfer_ready),
		.interrupt(interrupt),
		.transaction_complete(transaction_complete),
		.nack(nack),
		.data_rx(data_rx),
		.address_err(address_err)
	);

   // Reset when btn0 is pressed
   assign rst_n = usr_btn;   
   
	reg [15:0] MODEL_ID = 16'h5647;
	reg [16:0] PRE_STANDBY [0:3];
	initial begin
		PRE_STANDBY[0] = {1'b0, 8'h06, 8'h00};
		PRE_STANDBY[1] = {1'b0, 8'h02, 8'hff};
		PRE_STANDBY[2] = {1'b0, 8'h07, 8'h00};
		PRE_STANDBY[3] = {1'b0, 8'h03, 8'hff};
	end
	reg [16:0] PRE_STREAM [0:3];
initial begin
    PRE_STREAM[0] = {1'b0, 8'h06, 8'h00};
    PRE_STREAM[1] = {1'b0, 8'h02, 8'hff};
    PRE_STREAM[2] = {1'b0, 8'h07, 8'h00};
    PRE_STREAM[3] = {1'b0, 8'h03, 8'hff};
	end
	reg [16:0] POST_STREAM [0:0];
	initial POST_STREAM[0] = {1'b0, 8'h03, 8'hff};
	reg [2:0] sensor_state = 3'd0;
	reg [7:0] rom_counter = 8'd0;
	reg [1:0] byte_counter = 2'd0;
	wire power_enable;
	wire [1:0] mode = 2;
	reg nack_err = 1'b0;
	reg model_err = 1'b0;
	wire ready;


   //
   // Test: Toggle 5V line
   //
   assign gpio_0 = counter[24];

	assign rgb_led0_r = byte_counter[0];
   assign rgb_led0_g = ~transfer_start;
   //assign rgb_led0_g = usr_btn;   
	assign rgb_led0_b = ~transfer_continues;

	assign ready = ((sensor_state == 3'd0) || (sensor_state == 3'd2)) || (sensor_state == 3'd4);
	assign power_enable = sensor_state != 3'd0;
	wire [7:0] rom_end;
	assign rom_end = (sensor_state == 3'd1 ? 8'd2 : (sensor_state == 3'd3 ? 8'd3 : (sensor_state == 3'd6 ? 8'd0 : 8'd0)));
	wire [16:0] current_rom;
	assign current_rom = (sensor_state == 3'd1 ? PRE_STANDBY[rom_counter] : (sensor_state == 3'd3 ? PRE_STREAM[rom_counter] : (sensor_state == 3'd6 ? POST_STREAM[rom_counter] : 25'd0)));
	always @(posedge clk48)
		case (sensor_state)
			3'd0:
				if (mode != 2'd0)
					sensor_state <= 3'd1;
			3'd1, 3'd3, 3'd6:
				if (interrupt || transfer_ready)
					if (interrupt && (address_err || (!address[0] && nack))) begin
						transfer_start <= 1'b0;
						transfer_continues <= 1'b0;
						byte_counter <= 2'd0;
						rom_counter <= 8'd0;
						nack_err <= 1'd1;
					end
					else if (transfer_ready && byte_counter == 2'd0) // Write address MSB
					begin
						transfer_start <= 1'd1;
						transfer_continues <= !current_rom[16];
						address <= {ADDRESS[7:1], 1'b0};
						data_tx <= current_rom[15:8];
						byte_counter <= 2'd1;
					end
					
					else if (interrupt && byte_counter == 2'd1) // Write address LSB
					begin
						//transfer_start <= 1'd0;
						//transfer_continues <= !current_rom[24];
						//data_tx <= current_rom[15:8];
						byte_counter <= 2'd2;
					end
					
					else if (interrupt && (byte_counter == 2'd2)) begin
						transfer_start <= current_rom[16];
						transfer_continues <= 1'd0;
						if (current_rom[16])
							address <= {ADDRESS[7:1], 1'b1};
						data_tx <= current_rom[7:0];
						byte_counter <= 2'd3;
					end
					else if (interrupt && (byte_counter == 2'd3)) begin
						transfer_start <= 1'd0;
						transfer_continues <= 1'd0;
						byte_counter <= 2'd0;
						if (current_rom[16] && (current_rom[7:0] != data_rx)) begin
							rom_counter <= 8'd0;
							if (sensor_state == 3'd1)
								model_err <= 1'd1;
							sensor_state <= 3'd7;
						end
						else if (rom_counter == rom_end) begin
							rom_counter <= 8'd0;
							if (sensor_state == 3'd5)
								sensor_state <= 3'd4;
							else if (sensor_state == 3'd6)
								sensor_state <= (mode == 2'd1 ? 3'd2 : 3'd0);
							else
								sensor_state <= sensor_state + 1'd1;
						end
						else
							rom_counter <= rom_counter + 1'd1;
					end
			3'd2:
				if (mode == 2'd0)
					sensor_state <= 3'd0;
				else if (mode == 2'd2)
					sensor_state <= 3'd3;
				else
					sensor_state <= 3'd2;
			3'd4:
				if (mode != 2'd2)
					sensor_state <= 3'd6;
				else
					sensor_state <= 3'd4;
			3'd5:
				;
			3'd7:
				if (mode == 2'd0) begin
					model_err <= 1'd0;
					nack_err <= 1'd0;
					sensor_state <= 3'd0;
				end
		endcase
endmodule
