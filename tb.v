module tb();

  reg clk, resetn;
  wire we;
  wire [31:0] address, data_out, data_in;

  core dut(
    .clk(clk),
    .resetn(resetn),
    .address(address),
    .data_out(data_out),
    .data_in(data_in),
    .we(we)
  );

  memory m(
    .address(address),
    .data_in(data_out),
    .data_out(data_in),
    .we(we) 
  );

  // Clock generator
  always #1 clk = (clk === 1'b0);

  // Inicia a simulação e executa até 4000 unidades de tempo
  initial begin
    $dumpfile("saida.vcd");
    $dumpvars(0, tb);
    clk = 0;
    resetn = 1'b0;
    #11 resetn = 1'b1;
    $display("*** Starting simulation. ***");
    #4000 $display("*** Simulation timeout reached. ***");
    $finish;
  end

  // Monitoramento de acessos de memória e parada no endereço 0xFFC
  always @(posedge clk) begin
    if (address == 32'h00000FFC) begin
      $display("Address reached 4092 (0xFFC). Stopping simulation.");
      $finish;
    end else if (address[11] == 1) begin
      if (we == 1) begin
        $display("M[0x%h] <- 0x%h", address, data_out);
      end else begin
        $display("M[0x%h] -> 0x%h", address, data_in);
      end
    end
  end

endmodule
