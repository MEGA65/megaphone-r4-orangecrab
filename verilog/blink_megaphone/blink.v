`default_nettype none
module top (
	    input  clk48,
	    output reg rgb_led0_r,
	    output reg rgb_led0_g,
	    output reg rgb_led0_b,
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
   wire [7:0] 	   i2c_rdata;
   wire		   i2c_error;
   wire		   i2c_busy;
   reg 		   i2c_busy_last = 0;
   reg 		   i2c_reset_n = 0;

   reg [1:0] 	   i2c_busy_prev_state = 0;
   
   
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
	      .swap(1'd0),
	      .debug_scl(1'd0),
	      .debug_sda(1'd0)
	      );


   // State machine for performing I2C operations
   reg [7:0] 	   busy_count;
   
   
   //
   // Test: Toggle 5V line
   //
   assign gpio_0 = counter[24];

   //
   // Debug output on RGB LED
   //    
//   assign rgb_led0_r = i2c_busy;   
//   assign rgb_led0_g = busy_count[0];   
//   assign rgb_led0_b = counter[24];   

   initial begin
     rgb_led0_r <= ~0;
     rgb_led0_g <= ~0;
     rgb_led0_b <= ~0;
     busy_count <= 99;
   end
   
   always @(posedge clk48) begin

      // Detect rising edge of busy signals
      i2c_busy_last <= i2c_busy;

      rgb_led0_r = ~busy_count[0];      
      rgb_led0_g = ~busy_count[1];      
      rgb_led0_b = ~busy_count[2];      
//      rgb_led0_g = ~counter[24];      
      
      // Clear initial reset of I2C master
      i2c_reset_n <= 1;
      
      // Retrigger I2C every ~0.125 sec
      counter <= counter + 1;
      if (counter[21:0] == 22'd0) busy_count <= 99;
      
      // Now each time i2c_busy goes high we schedule
      // the next read or write action to the I2C state machine
      if ( { i2c_busy,i2c_busy_last } != i2c_busy_prev_state) begin
	 $display($time,": i2c_busy=",i2c_busy,", i2c_busy_last=",i2c_busy_last);
	 i2c_busy_prev_state <= { i2c_busy,i2c_busy_last};	 
      end
      
      if ( ((i2c_busy == 1) && (i2c_busy_last == 0))
	   || ( busy_count == 99 ) )
	  begin

	 // Advance to next state in the FSM
	 if ( busy_count != 8'd20 ) begin
	    busy_count <= busy_count + 1;
	 end
	 	 
	 // Schedule I2C byte transfers
	 if ( busy_count != 99 ) begin
	    $display("dispatching I2C command, busy_count=", busy_count);
	 end
	 else
	   $display("Kick-starting I2C communications");	    
	 
	 case (busy_count)
	   8'd99: begin
	      // Send address and start read transaction by first writing the selected register number 
	      i2c_command_en <= 1;
	      i2c_rw <= 0;
	      i2c_addr <= ADDRESS;
	      i2c_wdata <= 8'h06;  // Starting register number
	      busy_count <= 0;	      
	   end
	   8'd0: begin
	      // Read first byte (register 6)
	      i2c_command_en <= 1;	      
	      i2c_rw <= 1;
	     end	   

	   8'd1: begin
	      // Send address and start write transaction to select register 6
	      i2c_command_en <= 1;
	      i2c_rw <= 0;
	      i2c_addr <= ADDRESS;
	      i2c_wdata <= 8'd6;
	     end
 	   8'd2: begin
	      // Write to port 6
	      i2c_command_en <= 1;	      
	      i2c_rw <= 0;
	      i2c_wdata <= counter[31:24];
	     
	     end	   

	   8'd3: begin
	      // Complete write transaction
	      i2c_command_en <= 0;
	   end
	   
	 endcase // case (sensor_state)

      end // if i2c_busy
   end // always @ (posedge clk48)
   
   
endmodule
