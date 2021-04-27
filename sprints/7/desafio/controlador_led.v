module controlador_led(clk, rst, leds);
  
  input wire clk;
  input wire rst;
  output reg [7:0] leds;
  
  reg estado;
  wire [7:0] leds_subida;
  wire [7:0] leds_descida;
  
  assign leds_subida = {leds, 1'b1};
  assign leds_descida = leds>>1;
  
  // Lógica de estado
  always @(posedge clk, posedge rst) begin
    if (rst)
      estado <= 1'b0;
    else begin
      if (leds_subida == 8'hFF)
        estado <= 1'b1;
      else if(leds_descida == 8'h00)
        estado <= 1'b0;
      else
        estado <= estado;
    end
  end
  
  // Lógica dos leds de saída
  always @(posedge clk, posedge rst) begin
    if (rst)
      leds <= 8'h00;
    else begin
      if (estado)
        leds <= leds_descida;
      else
        leds <= leds_subida;
    end
  end
  
endmodule