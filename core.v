module core(
  input clk,
  input resetn,
  output reg [31:0] address,
  output reg [31:0] data_out,
  input [31:0] data_in,
  output reg we
);

reg [2:0] state;
localparam FETCH = 0, DECODE = 1, EXECUTE = 2, WRITE_BACK = 3;

reg [31:0] pc;
reg [31:0] instruction;
reg [31:0] registers [0:31];
reg [31:0] alu_result;
reg [4:0] rs1, rs2, rd;
reg [6:0] opcode;
reg [2:0] funct3;
reg [6:0] funct7;

integer i;
initial begin
  for (i = 0; i < 32; i = i + 1)
    registers[i] = 0;
end

always @(posedge clk) begin
  if (!resetn) begin
    pc <= 0;
    state <= FETCH;
    we <= 0;
  end else begin
    case (state)
      FETCH: begin
        address <= pc;
        we <= 0;
        state <= DECODE;
      end

      DECODE: begin
        instruction <= data_in;
        opcode <= data_in[6:0];
        rd <= data_in[11:7];
        funct3 <= data_in[14:12];
        rs1 <= data_in[19:15];
        rs2 <= data_in[24:20];
        funct7 <= data_in[31:25];
        state <= EXECUTE;
      end

      EXECUTE: begin
        if (opcode == 7'b0110011) begin // Tipo R
          case ({funct7, funct3})
            10'b0000000111: begin // AND
              alu_result <= registers[rs1] & registers[rs2];
              state <= WRITE_BACK;
            end
            default: begin
              pc <= pc + 4;
              state <= FETCH;
            end
          endcase
        end else if (opcode == 7'b1110011 && funct3 == 3'b000) begin // EBREAK
          $finish;
        end else begin
          pc <= pc + 4;
          state <= FETCH;
        end
      end

      WRITE_BACK: begin
        if (rd != 0)
          registers[rd] <= alu_result;
        pc <= pc + 4;
        state <= FETCH;
      end
    endcase
  end
end

endmodule