`default_nettype none
module testbed (
	    );

   reg clk48;
   input rgb_led0_r;
   input rgb_led0_g;
   input rgb_led0_b;
   wire sda;
   wire scl;

   reg 	sda_last = 1;
   reg 	scl_last = 1;   
   
   reg [31:0]	cycles;   
   
   top blink (
	  .clk48(clk48),
	  .rgb_led0_r(rgb_led0_r),
	  .rgb_led0_g(rgb_led0_g),
	  .rgb_led0_b(rgb_led0_b),
	  .scl(scl),
	  .sda(sda)
	  );

   pullup(sda);
   pullup(scl);
   
   
initial
  begin
     $display("starting");
     for (cycles=0;cycles<10000;cycles++)
       begin
	  clk48 = 1'b1; 
	  #20; // high for 20 * timescale = 20 ns
	  
	  clk48 = 1'b0;
	  #20; // low for 20 * timescale = 20 ns

	  if ((sda != sda_last) || (scl != scl_last)) begin
	     $display("testbed: SDA=", sda, ", SCL=", scl);
	     sda_last <= sda;
	     scl_last <= scl;
	     
	  end
	  
       end
     $display("done");
     $finish;
     
  end
   
   
endmodule
