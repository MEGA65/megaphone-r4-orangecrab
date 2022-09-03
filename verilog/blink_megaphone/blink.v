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

   // Reset when btn0 is pressed for easy access to DFU mode
   assign rst_n = usr_btn;   
      
   // Counter for timed events
   reg [31:0] 	   counter = 0;
   always @(posedge clk48) counter <= counter + 1;
   
   // I2C bus speed
   parameter signed [31:0] INPUT_CLK_RATE = 48000000;
   parameter signed [31:0] TARGET_SCL_RATE = 100000;
   // I2C device address (in 7-bit format)
   parameter [6:0] ADDRESS = 7'h25;
   // I2C interface signals
   reg [6:0] 	   i2c_addr;   
   reg 		   i2c_command_en = 0;
   reg 		   i2c_rw;
   reg [7:0] 	   i2c_wdata;
   reg [7:0] 	   i2c_rdata;
   reg 		   i2c_error;
   reg 		   i2c_busy;
   reg 		   i2c_busy_last = 0;
   reg 		   i2c_reset_n = 1;
   
   
   // I2C bus   
   i2c_master #(
		.input_clk(INPUT_CLK_RATE),
		.bus_clk(TARGET_SCL_RATE)
		) 
   i2c_master(
	      .clk(clk48),
	      .reset_n(i2c_reset_n),
	      .ena(i2c_command_en),
	      .addr(i2c_addr),
	      .rw(i2c_rw),
	      .data_wr(i2c_wdata),
	      .busy(i2c_busy),
	      .data_rd(i2c_rdata),
	      .ack_error(i2c_error),
	      .sda(sda),
	      .scl(scl),
	      .swap(0),
	      .debug_scl(0),
	      .debug_sda(0)
	      );


   // State machine for performing I2C operations
   reg [7:0] 	   busy_count = 0;
   
   
   //
   // Test: Toggle 5V line
   //
   assign gpio_0 = counter[24];

   //
   // Debug output on RGB LED
   //    
   assign rgb_led0_r = 0;   
   assign rgb_led0_g = 1;   
   assign rgb_led0_b = 0;   
   
   always @(posedge clk48) begin
      // Detect rising edge of busy signals
      i2c_busy_last <= i2c_busy;
      
      if ((busy_count == 8'd0) && (i2c_busy == 0)) begin
	 // On initial start up
	 busy_count <= 8'd1;
      end
      
      // Now each time i2c_busy goes high we schedule
      // the next read or write action to the I2C state machine
      if ((i2c_busy == 1) && (i2c_busy_last == 0)) begin
	 case (busy_count)
	   8'd1: begin
	      // Send address and start transaction
	      i2c_command_en <= 1;
	      i2c_rw <= 1;
	      i2c_addr <= ADDRESS;
	   end	   
	   
	 endcase // case (sensor_state)
	 
	 // Advance to next state in the FSM
	 busy_count <= busy_count + 1;
	 
      end // if i2c_busy
   end // always @ (posedge clk48)
   
   
endmodule
