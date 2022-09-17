`default_nettype none
module top (
	    input      clk48,
	    output reg rgb_led0_r,
	    output reg rgb_led0_g,
	    output reg rgb_led0_b,
	    output reg gpio_0, // VCC_MIC/JOYSTICK enable
	    output     gpio_a0, // FPGA Power Off (active low)
	    output     gpio_9, // ESP32 UART TX
	    input      gpio_6, // ESP32 UART RX
	    
	    inout      scl,
	    inout      sda,
	    input      usr_btn,
	    output     rst_n
	    );

   // Reset when btn0 is pressed for easy access to DFU mode
   assign rst_n = usr_btn;   
 
   // Counter for timed events
   reg [31:0] 	   counter = 0;
   
   // I2C bus speed
   parameter signed [31:0] INPUT_CLK_RATE = 48000000;
   parameter signed [31:0] TARGET_SCL_RATE = 100000;
   // I2C device address (in 7-bit format)
   parameter [6:0] ADDRESS = 7'h24;
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
   // Remember which register pair we are writing to
   reg [1:0] 	   reg_pair = 2'b10;
   reg [1:0] 	   io_expander;   

   reg [7:0] 	   uart_xilinx0_txdata;
   reg 		   uart_xilinx0_txtrigger;
   reg 		   uart_xilinx0_dispatched;
   wire		   uart_xilinx0_txready;
   reg [7:0] 	   uart_xilinx0_txstate; 		   

   wire [7:0] 	   uart_xilinx0_rxdata;
   wire 	   uart_xilinx0_rxready;
   reg 		   uart_xilinx0_rxack;

   reg [7:0] 	   power_rail_states;   
   
   // UART RX from Xilinx FPGA
   uart_rx xilinx_uart0_rx (
			    .clk(clk48),
			    .bit_rate_divisor(24'd23), // 2Mbs
			    .UART_RX(gpio_6),
			    .data(uart_xilinx0_rxdata),
			    .data_ready(uart_xilinx0_rxready),
			    .data_acknowledge(uart_xilinx0_rxack)
			    );   
   
   // UART TX to Xilinx FPGA
   uart_tx xilinx_uart0_tx (
	    .CLK(clk48),
	    .BIT_TMR_MAX(24'd23), // 48MHz/2Mbps = 24. 24 - 1 = 23
	    .DATA(uart_xilinx0_txdata),
	    .SEND(uart_xilinx0_txtrigger),
	    .READY(uart_xilinx0_txready),
	    // XXX for now feed to ESP32 board UART pins during bring-up testing
	    .UART_TX(gpio_9)	    
	    );
   

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
   

   /*
    Signals to keep track of power rails and other outputs
    */
   reg 		   power_rail_modem1;
   reg 		   power_rail_modem2;
   reg 		   power_rail_rfd900;
   reg 		   power_rail_esp32;
   reg 		   power_rail_screen;
   reg 		   power_rail_speaker_amplifier;
   reg 		   lcd_standby;
   reg 		   modem1_wake_n;
   reg 		   power_rail_mic;
   reg 		   cm4_en;
   reg 		   cm4_wifi_en;
   reg 		   cm4_bt_en;
   reg 		   lcd_display_en;
   reg 		   lcd_backlight_en;
   reg 		   esp32_reset_n;
   reg 		   hdmi_hotplug_detect_enable;
   reg 		   hdmi_en;
   reg 		   hdmi_rx_enable;
   reg 		   modem1_reset_n;
   reg 		   modem2_wake_n;
   reg 		   modem2_reset_n;
   reg 		   modem2_wireless_disable;
   reg 		   modem1_wireless_disable;
   reg 		   power_rail_headphone_amplifier;
   reg 		   hdmi_hotplug_detect;
   reg 		   hdmi_cec_a;
   reg 		   otp_hold_n;
   reg 		   otp_reset_n;
   reg 		   otp_cs2;
   reg 		   otp_cs1;
   reg 		   otp_wp_n;
   reg 		   otp_si;
   reg 		   main_fpga_poweroff_n;

   reg [7:0]	   io_expander0_port0;
   reg [7:0]	   io_expander0_port1;
   reg [7:0]	   io_expander1_port0;
   reg [7:0]	   io_expander1_port1;
   reg [7:0]	   io_expander2_port0;
   reg [7:0]	   io_expander2_port1;

   reg [3:0] 	   loop_count;

   reg 		   debugstrobe;
   
   
   //
   // Debug output on RGB LED
   //    
//   assign rgb_led0_r = i2c_busy;   
//   assign rgb_led0_g = busy_count[0];   
//   assign rgb_led0_b = counter[24];   

   initial begin

      gpio_a0 <= 1'b1;      
            
      debugstrobe <= 1'b0;      
      
      loop_count <= 4'd0;
      
      rgb_led0_r <= ~0;
      rgb_led0_g <= ~0;
      rgb_led0_b <= ~0;
      busy_count <= 99;
      // Start by setting up inversion and DDR bits for ports
      reg_pair <= 2'b10;
      io_expander <= 2'b00;      

      power_rail_states <= 8'd0;      
      
      // Put UARTs idle on reset
      uart_xilinx0_txtrigger <= 1'b0;      
      uart_xilinx0_dispatched <= 1'b0;
      uart_xilinx0_txstate <= 8'd0;
      uart_xilinx0_rxack <= 1'b0;

      // Default output settings
      power_rail_modem1 <= 1'b0;
      power_rail_modem2 <= 1'b0;
      power_rail_rfd900 <= 1'b0;
      power_rail_esp32 <= 1'b0;
      power_rail_screen <= 1'b0;
      power_rail_speaker_amplifier <= 1'b0;
      lcd_standby <= 1'b0;
      modem1_wake_n <= 1'b0;
      power_rail_mic <= 1'b0;
      cm4_en <= 1'b0;
      cm4_wifi_en <= 1'b0;
      cm4_bt_en <= 1'b0;
      lcd_display_en <= 1'b0;
      lcd_backlight_en <= 1'b0;
      esp32_reset_n <= 1'b0;
      hdmi_hotplug_detect_enable <= 1'b0;
      hdmi_en <= 1'b0;
      hdmi_rx_enable <= 1'b0;
      modem1_reset_n <= 1'b0;
      modem2_wake_n <= 1'b0;
      modem2_reset_n <= 1'b0;
      modem2_wireless_disable <= 1'b0;
      modem1_wireless_disable <= 1'b0;
      power_rail_headphone_amplifier <= 1'b0;
      hdmi_hotplug_detect <= 1'b0;
      hdmi_cec_a <= 1'b0;
      otp_hold_n <= 1'b0;
      otp_reset_n <= 1'b0;
      otp_cs2 <= 1'b0;
      otp_cs1 <= 1'b0;
      otp_wp_n <= 1'b0;
      otp_si <= 1'b0;
      main_fpga_poweroff_n <= 1'b1;            
      
   end
   
   always @(posedge clk48) begin

      // Propagate main FPGA power rail cut signal
      gpio_a0 <= main_fpga_poweroff_n;      
      
//      $display("txready=",uart_xilinx0_txready,", dispatched=", uart_xilinx0_dispatched);

//      rgb_led0_r = ~uart_xilinx0_txready;      
//      rgb_led0_g = ~uart_xilinx0_dispatched;      
//      rgb_led0_b = ~1'b0;      

      if ( uart_xilinx0_txready == 1'b1 && uart_xilinx0_dispatched == 1'b0 ) begin
	 // Send next char via UART
	 uart_xilinx0_dispatched <= 1'b1;
	 uart_xilinx0_txtrigger <= 1'b1;

	 if (uart_xilinx0_txstate < 255) uart_xilinx0_txstate <= uart_xilinx0_txstate + 1;

	 $display("Sending char ",uart_xilinx0_txstate);
	 
	 case (uart_xilinx0_txstate)
	   // Display ready message
	   // MEGAphone CTL0<CRLF>
	   8'd0: uart_xilinx0_txdata <= 8'h4D;
	   8'd1: uart_xilinx0_txdata <= 8'h45;
	   8'd2: uart_xilinx0_txdata <= 8'h47;
	   8'd3: uart_xilinx0_txdata <= 8'h41;
	   8'd4: uart_xilinx0_txdata <= 8'h70;
	   8'd5: uart_xilinx0_txdata <= 8'h68;
	   8'd6: uart_xilinx0_txdata <= 8'h6f;
	   8'd7: uart_xilinx0_txdata <= 8'h6e;
	   8'd8: uart_xilinx0_txdata <= 8'h65;
	   8'd9: uart_xilinx0_txdata <= 8'h20;
	   8'd10: uart_xilinx0_txdata <= 8'h43;
	   8'd11: uart_xilinx0_txdata <= 8'h54;
	   8'd12: uart_xilinx0_txdata <= 8'h4c;
	   8'd13: uart_xilinx0_txdata <= 8'h30;
	   8'd14: uart_xilinx0_txdata <= 8'h20;
	   8'd15: begin
	      if (io_expander0_port0[7:4] < 4'd10) uart_xilinx0_txdata <= 8'h30 + io_expander0_port0[7:4];
	      else uart_xilinx0_txdata <= 8'h57 + io_expander0_port0[7:4];
	   end
	   8'd16: begin
	      if (io_expander0_port0[3:0] < 4'd10) uart_xilinx0_txdata <= 8'h30 + io_expander0_port0[3:0];
	      else uart_xilinx0_txdata <= 8'h57 + io_expander0_port0[3:0];
	   end
	   8'd17: begin
	      if (io_expander0_port1[7:4] < 4'd10) uart_xilinx0_txdata <= 8'h30 + io_expander0_port1[7:4];
	      else uart_xilinx0_txdata <= 8'h57 + io_expander0_port1[7:4];
	   end
	   8'd18: begin
	      if (io_expander0_port1[3:0] < 4'd10) uart_xilinx0_txdata <= 8'h30 + io_expander0_port1[3:0];
	      else uart_xilinx0_txdata <= 8'h57 + io_expander0_port1[3:0];
	   end

	   8'd19: begin
	      if (io_expander1_port0[7:4] < 4'd10) uart_xilinx0_txdata <= 8'h30 + io_expander1_port0[7:4];
	      else uart_xilinx0_txdata <= 8'h57 + io_expander1_port0[7:4];
	   end
	   8'd20: begin
	      if (io_expander1_port0[3:0] < 4'd10) uart_xilinx0_txdata <= 8'h30 + io_expander1_port0[3:0];
	      else uart_xilinx0_txdata <= 8'h57 + io_expander1_port0[3:0];
	   end
	   8'd21: begin
	      if (io_expander1_port1[7:4] < 4'd10) uart_xilinx0_txdata <= 8'h30 + io_expander1_port1[7:4];
	      else uart_xilinx0_txdata <= 8'h57 + io_expander1_port1[7:4];
	   end
	   8'd22: begin
	      if (io_expander1_port1[3:0] < 4'd10) uart_xilinx0_txdata <= 8'h30 + io_expander1_port1[3:0];
	      else uart_xilinx0_txdata <= 8'h57 + io_expander1_port1[3:0];
	   end

	   8'd23: begin
	      if (io_expander2_port0[7:4] < 4'd10) uart_xilinx0_txdata <= 8'h30 + io_expander2_port0[7:4];
	      else uart_xilinx0_txdata <= 8'h57 + io_expander2_port0[7:4];
	   end
	   8'd24: begin
	      if (io_expander2_port0[3:0] < 4'd10) uart_xilinx0_txdata <= 8'h30 + io_expander2_port0[3:0];
	      else uart_xilinx0_txdata <= 8'h57 + io_expander2_port0[3:0];
	   end
	   8'd25: begin
	      if (io_expander2_port1[7:4] < 4'd10) uart_xilinx0_txdata <= 8'h30 + io_expander2_port1[7:4];
	      else uart_xilinx0_txdata <= 8'h57 + io_expander2_port1[7:4];
	   end
	   8'd26: begin
	      if (io_expander2_port1[3:0] < 4'd10) uart_xilinx0_txdata <= 8'h30 + io_expander2_port1[3:0];
	      else uart_xilinx0_txdata <= 8'h57 + io_expander2_port1[3:0];
	   end
	   8'd27: begin
		  if (loop_count[3:0] < 4'd10) uart_xilinx0_txdata <= 8'h30 + loop_count[3:0];
		  else uart_xilinx0_txdata <= 8'h57 + loop_count[3:0];
		  loop_count <= loop_count + 1;
	   end
	   	   
	   8'd28: uart_xilinx0_txdata <= 8'h0d;
	   8'd29: uart_xilinx0_txdata <= 8'h0a;
	   default: begin
	      // Do nothing while idle
	      uart_xilinx0_txtrigger <= 1'b0;	   
	      uart_xilinx0_dispatched <= 1'b0;
	   end
	 endcase
	   
      end
      if ( uart_xilinx0_txready == 1'b0 ) begin
	 uart_xilinx0_dispatched <= 1'b0;	 
	 uart_xilinx0_txtrigger <= 1'b0;	 
      end

      uart_xilinx0_rxack <= 1'b0;      
      if ( uart_xilinx0_rxready == 1 ) begin
	 uart_xilinx0_rxack <= 1'b1;
	 
	 case (uart_xilinx0_rxdata)
	   8'd13,8'd10: // ENTER = display ID message
	     uart_xilinx0_txstate <= 8'd0;
	   8'h40: power_rail_modem1 <= 1'b1;
	   8'h41: power_rail_modem2 <= 1'b1;
	   8'h42: power_rail_rfd900 <= 1'b1;
	   8'h43: power_rail_esp32 <= 1'b1;
	   8'h44: power_rail_screen <= 1'b1;
	   8'h45: power_rail_speaker_amplifier <= 1'b1;
	   8'h46: lcd_standby <= 1'b1;
	   8'h47: modem1_wake_n <= 1'b1;
	   8'h48: power_rail_mic <= 1'b1;
	   8'h49: cm4_en <= 1'b1;
	   8'h4a: cm4_wifi_en <= 1'b1;
	   8'h4b: cm4_bt_en <= 1'b1;
	   8'h4c: lcd_display_en <= 1'b1;
	   8'h4d: lcd_backlight_en <= 1'b1;
	   8'h4e: esp32_reset_n <= 1'b1;
	   8'h4f: hdmi_hotplug_detect_enable <= 1'b1;
	   8'h50: hdmi_en <= 1'b1;
	   8'h51: hdmi_rx_enable <= 1'b1;
	   8'h52: modem1_reset_n <= 1'b1;
	   8'h53: modem2_wake_n <= 1'b1;
	   8'h54: modem2_reset_n <= 1'b1;
	   8'h55: modem2_wireless_disable <= 1'b1;
	   8'h56: modem1_wireless_disable <= 1'b1;
	   8'h57: power_rail_headphone_amplifier <= 1'b1;
	   8'h58: hdmi_hotplug_detect <= 1'b1;
	   8'h59: hdmi_cec_a <= 1'b1;
	   8'h5a: otp_hold_n <= 1'b1;
	   8'h5b: otp_reset_n <= 1'b1;
	   8'h5c: otp_cs2 <= 1'b1;
	   8'h5d: otp_cs1 <= 1'b1;
	   8'h5e: otp_wp_n <= 1'b1;
	   8'h5f: otp_si <= 1'b1;

	   8'h60: power_rail_modem1 <= 1'b0;
	   8'h61: power_rail_modem2 <= 1'b0;
	   8'h62: power_rail_rfd900 <= 1'b0;
	   8'h63: power_rail_esp32 <= 1'b0;
	   8'h64: power_rail_screen <= 1'b0;
	   8'h65: power_rail_speaker_amplifier <= 1'b0;
	   8'h66: lcd_standby <= 1'b0;
	   8'h67: modem1_wake_n <= 1'b0;
	   8'h68: power_rail_mic <= 1'b0;
	   8'h69: cm4_en <= 1'b0;
	   8'h6a: cm4_wifi_en <= 1'b0;
	   8'h6b: cm4_bt_en <= 1'b0;
	   8'h6c: lcd_display_en <= 1'b0;
	   8'h6d: lcd_backlight_en <= 1'b0;
	   8'h6e: esp32_reset_n <= 1'b0;
	   8'h6f: hdmi_hotplug_detect_enable <= 1'b0;
	   8'h70: hdmi_en <= 1'b0;
	   8'h71: hdmi_rx_enable <= 1'b0;
	   8'h72: modem1_reset_n <= 1'b0;
	   8'h73: modem2_wake_n <= 1'b0;
	   8'h74: modem2_reset_n <= 1'b0;
	   8'h75: modem2_wireless_disable <= 1'b0;
	   8'h76: modem1_wireless_disable <= 1'b0;
	   8'h77: power_rail_headphone_amplifier <= 1'b0;
	   8'h78: hdmi_hotplug_detect <= 1'b0;
	   8'h79: hdmi_cec_a <= 1'b0;
	   8'h7a: otp_hold_n <= 1'b0;
	   8'h7b: otp_reset_n <= 1'b0;
	   8'h7c: otp_cs2 <= 1'b0;
	   8'h7d: otp_cs1 <= 1'b0;
	   8'h7e: otp_wp_n <= 1'b0;
	   8'h7f: otp_si <= 1'b0;

	   8'h2e: // . = power off main FPGA
	     main_fpga_poweroff_n <= 1'b0;
	   8'h3a: // : = stop requesting power off to main FPGA
	     main_fpga_poweroff_n <= 1'b1;
	   
	 endcase; 
	       
	 rgb_led0_r <= ~rgb_led0_r;
      end
      
      
      // Detect rising edge of busy signals
      i2c_busy_last <= i2c_busy;

//      rgb_led0_g = ~counter[24];      
      
      // Clear initial reset of I2C master
      i2c_reset_n <= 1;
      
      /*
       We have 3 IO expanders to scan. One of them has the joystick and other buttons, 
       so we want to minimise the latency on those.  1ms would be nice, but 10ms should 
       be fine, too.  
       
       The two IO expanders that need to have signals set require 6 I2C transfers each x 10 bits = 60 bits.
       The third IO expander that needs to be read and written required 11 transfers = 110 bits.
       Thus the total time required is < 200 bits.
       At 100KHz clock rate, we should be able to scan every 2ms or so.
       If we can increase the I2C clock to 400KHz, then 0.5ms should be possible.
       But we will stick to 100KHz for now, and schedule one IO expander every 2ms, so that even the 110 bit 
       one won't overflow the time. This will give us an actual update interval of 6ms, which should be fine.
       
       Our clock is 48MHz, so to get 2ms (500 Hz), we need 48x10^6 / 500 Hz = 96,000 cycles.
       That's annoying, as a power of 2 would be easier. Fortunately there is enough slop in our calculations
       to use 65,536 cycles, which gives us one IO expander probe every 1.3ms, i.e., a complete update cycle
       every 4.096ms.  That seems pretty reasonable to me.
       
       */
      counter <= counter + 1;
      if (counter[16:0] == 16'd0) begin
	 busy_count <= 99;
      end
      
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
	      i2c_addr[6:2] <= ADDRESS[6:2];
	      i2c_addr[1:0] <= io_expander;	      
	      i2c_wdata <= 8'd0; // Port 0 read address
	      busy_count <= 0;	      
	   end
	   8'd0: begin
	      // Read first byte (register 0)
	      i2c_command_en <= 1;	      
	      i2c_rw <= 1;
	     end	   
	   8'd1: begin
	      // Read second byte (register 1)
	      i2c_command_en <= 1;	      
	      i2c_rw <= 1;
	     end	   

	   8'd2: begin

	      // First byte is now available
	      case (io_expander)
		2'b00: begin 
		   io_expander0_port0 <= i2c_rdata;
		   if (io_expander0_port0 != i2c_rdata) uart_xilinx0_txstate <= 8'd0;
		end
		2'b01: begin
		   io_expander1_port0 <= i2c_rdata;
		   if (io_expander1_port0 != i2c_rdata) uart_xilinx0_txstate <= 8'd0;
		end
		2'b10: begin
		   io_expander2_port0 <= i2c_rdata;
		   if (io_expander2_port0 != i2c_rdata) uart_xilinx0_txstate <= 8'd0;
		end
	      endcase; // case (io_expander)	      

	      // Send address and start write transaction to select register 2
	      // except during start-up, when we initialise the other registers
	      i2c_command_en <= 1;
	      i2c_rw <= 0;
	      i2c_addr[6:2] <= ADDRESS[6:2];
	      i2c_addr[1:0] <= io_expander;	      
	      // Select the register pair to write to	      
	      i2c_wdata[7:3] <= 5'd0;     
	      i2c_wdata[2:1] <= reg_pair;
	      i2c_wdata[0] <= 1'b0;     
	     end
 	   8'd3: begin

	      // And second read byte is now available
	      case (io_expander)
		2'b00: begin
		   io_expander0_port1 <= i2c_rdata;
		   if (io_expander0_port1 != i2c_rdata) uart_xilinx0_txstate <= 8'd0;
		end
		2'b01: begin
		   io_expander1_port1 <= i2c_rdata;
		   if (io_expander1_port1 != i2c_rdata) uart_xilinx0_txstate <= 8'd0;
		end
		2'b10: begin
		   io_expander2_port1 <= i2c_rdata;
		   if (io_expander2_port1 != i2c_rdata) uart_xilinx0_txstate <= 8'd0;
		end
	      endcase; // case (io_expander)	      	      
	      
	      // Write to register 2 : Power rail enables
	      // (or reg 4 (inversions of port 0) or reg 6 (ddr of port 0)
	      i2c_command_en <= 1;	      
	      i2c_rw <= 0;
	      case (io_expander)
		2'b00: begin // U14
		   case (reg_pair)
		     2'b01: begin	       
			i2c_wdata[0] <= power_rail_modem1;
			i2c_wdata[1] <= power_rail_modem2;
			i2c_wdata[2] <= power_rail_rfd900;
			i2c_wdata[3] <= power_rail_esp32;
			i2c_wdata[4] <= power_rail_screen;
			i2c_wdata[5] <= power_rail_speaker_amplifier;
			i2c_wdata[6] <= lcd_standby;
			i2c_wdata[7] <= modem1_wake_n; // DTR line, pull low to wake module from sleep		   
			
			gpio_0 <= power_rail_mic;			
		     end
	             2'b10: i2c_wdata <= 8'h00; // port 0 inversions
		     2'b11: i2c_wdata <= 8'b11000000; // port 0 DDR
		   endcase; // case (reg_pair)		  
		end // case: 2'b00
		2'b01: begin // U13
		   case (reg_pair)
		     2'b01: begin	       
			i2c_wdata[0] <= cm4_en;			
			i2c_wdata[1] <= cm4_wifi_en;			
			i2c_wdata[2] <= cm4_bt_en;			
			i2c_wdata[3] <= lcd_display_en;			
			i2c_wdata[4] <= lcd_backlight_en;			
			i2c_wdata[5] <= esp32_reset_n;			
			i2c_wdata[6] <= hdmi_hotplug_detect_enable;		
			i2c_wdata[7] <= hdmi_en;			
		     end
	             2'b10: i2c_wdata <= 8'h00; // port 0 inversions
		     2'b11: i2c_wdata <= 8'b11000000; // port 0 DDR
		   endcase; // case (reg_pair)		   
		end // case: 2'b01		
		2'b10: begin // U12
		   case (reg_pair)
		     2'b01: begin	       
			i2c_wdata[0] <= 1'b1;  // D-PAD inputs
			i2c_wdata[1] <= 1'b1;  	
			i2c_wdata[2] <= 1'b1;			
			i2c_wdata[3] <= 1'b1;			
			i2c_wdata[4] <= 1'b1;  // S2 buttons	
			i2c_wdata[5] <= 1'b1;
			i2c_wdata[6] <= hdmi_rx_enable;			
			i2c_wdata[7] <= 1'b1;			
		     end
	             2'b10: i2c_wdata <= 8'h00; // port 0 inversions
		     2'b11: i2c_wdata <= 8'b10111111; // port 0 DDR
		   endcase; // case (reg_pair)		   
		end // case: 2'b10
	      endcase; // case (io_expander)
	   end // case: 8'd3	   
 	   8'd4: begin

	      // Write to register 3 : Power rail enable in bit 5 for headphones amplifier
	      i2c_command_en <= 1;	      
	      i2c_rw <= 0;
	      case (io_expander)
		2'b00: begin // U14
		   case (reg_pair)
		     2'b01: begin
			i2c_wdata[0] <= modem1_reset_n;		   
			i2c_wdata[1] <= modem2_wake_n;		   
			i2c_wdata[2] <= modem2_reset_n;		   
			i2c_wdata[3] <= modem2_wireless_disable;		   
			i2c_wdata[4] <= modem1_wireless_disable;		   
			i2c_wdata[5] <= power_rail_headphone_amplifier;		   
			i2c_wdata[6] <= hdmi_hotplug_detect;		   
			i2c_wdata[7] <= 1'b0; // NOT CONNECTED		   
		     end
		     2'b10: i2c_wdata <= 8'h00; // port 1 inversions
		     2'b11: i2c_wdata <= 8'b11011111; // port 1 DDR
		   endcase
		end	  
		2'b01: begin // U13
		   case (reg_pair)
		     2'b01: begin
			i2c_wdata[0] <= hdmi_cec_a;		   
			i2c_wdata[1] <= otp_hold_n;		   
			i2c_wdata[2] <= otp_reset_n;		   
			i2c_wdata[3] <= otp_cs2;		   
			i2c_wdata[4] <= otp_cs1;		  
			i2c_wdata[5] <= otp_wp_n;		   
			i2c_wdata[6] <= 1'b1;      // otp_so;		   
			i2c_wdata[7] <= otp_si;		   
		     end
		     2'b10: i2c_wdata <= 8'h00; // port 1 inversions
		     2'b11: i2c_wdata <= 8'b01000000; // port 1 DDR
		   endcase
		end // case: 2'b01
		2'b10: begin // U12
		   case (reg_pair)
		     2'b01: begin	       
			i2c_wdata[0] <= 1'b1;  // S3 buttons
			i2c_wdata[1] <= 1'b1;  	
			i2c_wdata[2] <= 1'b1;  // Interrupt sense	
			i2c_wdata[3] <= 1'b1;  // J3 joystick inputs			
			i2c_wdata[4] <= 1'b1; 
			i2c_wdata[5] <= 1'b1;
			i2c_wdata[6] <= 1'b1;			
			i2c_wdata[7] <= 1'b1;			
		     end
	             2'b10: i2c_wdata <= 8'h00; // port 0 inversions
		     2'b11: i2c_wdata <= 8'b11111111; // port 0 DDR
		   endcase		   
		end // case: 2'b10
	      endcase; // case (io_expander)
	   end // case: 8'd4	   
	   8'd5: begin
	      // Complete write transaction
	      i2c_command_en <= 0;
	      // Select the next register pair to write
	      case (io_expander)
		2'b00: io_expander <= 2'b01;
		2'b01: io_expander <= 2'b10;		
		2'b10: begin
		   io_expander <= 2'b00;		   
		   case (reg_pair)
		     2'b10: reg_pair <= 2'b11; // setup DDRs for ports after clearing inversions
		     2'b11: reg_pair <= 2'b01; // then write port 0/1 outputs forever after
		   endcase // case reg_pair
		end
	      endcase; // case (io_expander)	      
	   end
	 endcase // case (sensor_state)
      end // if i2c_busy
   end // always @ (posedge clk48)
   
   
endmodule
