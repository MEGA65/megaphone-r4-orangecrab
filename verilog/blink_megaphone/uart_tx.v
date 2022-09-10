
module uart_tx(
input wire SEND,
input wire [23:0] BIT_TMR_MAX,
input wire [7:0] DATA,
input wire CLK,
output reg READY,
output reg UART_TX
);

   reg [7:0] state;
   reg [9:0] txData;
   reg [23:0] bitTimer;
   reg [3:0]  bits_remaining;
   reg [15:0]	      foo;
   reg [7:0] 	      chars;
   

   initial begin
      $display("Setting up UART_TX");
      
      // Idle UART on power up
      state <= 8'd0;
      READY <= 1'b1;
      bitTimer <= 24'd0;
      foo <= 16'd0;
      chars <= 8'd0;
      
   end
   
  //Next state logic
  always @(posedge CLK) begin
     case (state)
       8'd0: // IDLE
	 begin
	    
	    if (SEND==1'b1) begin
	       // Begin sending
	       $display("Sending character %%%08b",DATA);
               txData <= {1'b1,DATA,1'b0};
	       bits_remaining <= 10;
	       state <= 8'd1;
	       bitTimer <= 24'd0;
	       READY <= 1'b0;
	       chars <= chars + 1;
	       
	    end
	 end // case: 8'd0       
       8'd1: // Send bit
	 begin
//	    $display("bitTimer=",bitTimer,", BIT_TMR_MAX=",BIT_TMR_MAX,", bits_remaining=",bits_remaining);
	    
	    UART_TX <= txData[0];       
//	    UART_TX <= bitTimer[0];
	    
	    if (bitTimer < BIT_TMR_MAX) begin
	       bitTimer <= bitTimer + 1;
	    end else begin
	       bitTimer <= 0;
	       txData[8:0] <= txData[9:1];
	       $display("  sending bit",txData[0]);
	       
	       bits_remaining <= bits_remaining - 1;	       
	       if (bits_remaining == 1) begin
		  // We just sent the last bit
		  state <= 8'd0;
		  $display("Asserting READY at end of char TX");
		  
		  READY <= 1'b1;
	       end
	    end // else: !if(bitTimer < BIT_TMR_MAX)
	 end // case: 8'd1
       default: begin
	  READY <= 1'b0;
       end
     endcase // case (state)
  end

endmodule
