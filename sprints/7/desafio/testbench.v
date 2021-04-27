module testbench();
  
  reg clk;
  reg rst;
  reg [7:0] leds;
  
  controlador_led LED_CTRL
  (
    .clk(clk),
    .rst(rst),
    .leds(leds)
  );
  
  initial begin
    clk = 1'b0;
    rst = 1'b0;
    #10 clk = ~clk;
    #10 rst = ~rst;
    #10 clk = ~clk;
    #10 rst = ~rst;
    for (integer i=0; i<20; i++) begin
      $display("%02d: %b", i, leds);
      #10 clk = ~clk;
      #10 clk = ~clk;
    end
  end
  
endmodule