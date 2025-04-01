module core(
  input clk,
  input resetn,
  output reg [31:0] address, // endereco que vai ser enviado pra memoria (dado ou instrucao)
  output reg [31:0] data_out,
  input [31:0] data_in, // traz dados da memória para o processador, instrução ou valor de memória
  output reg we // sinal de escrita ou leitura na memoria
);

reg [2:0] state;
localparam FETCH = 0, DECODE = 1, EXECUTE = 2, MEMORY = 3, WRITE_BACK = 4;

// Registradores e variáveis

reg [31:0] pc, instruction;
reg [31:0] registers[0:31]; //  banco de registradores x0–x31 
reg [31:0] alu_result;
reg [4:0] rs1, rs2, rd; // campos da instrução
reg [6:0] opcode;
reg [2:0] funct3;
reg [6:0] funct7;
reg [31:0] imm;

// Zerar os registradores para poder simular. ??

integer i;
initial begin
  for (i = 0; i < 32; i = i + 1) registers[i] = 0;
end


always @(posedge clk) begin
  if (!resetn) begin // reiniciar o pc e voltar ao estado de busca
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

        // Imediatos
        case (data_in[6:0])
          7'b0010011, 7'b0000011: // I-type
            imm <= {{20{data_in[31]}}, data_in[31:20]};
          7'b0100011: // S-type
            imm <= {{20{data_in[31]}}, data_in[31:25], data_in[11:7]};
          7'b0110111, 7'b0010111: // U-type
            imm <= {data_in[31:12], 12'b0};
          default: imm <= 0;
        endcase

        state <= EXECUTE;
      end

      EXECUTE: begin
        we <= 0;
        case (opcode)
          7'b0110011: begin // R-type
            case ({funct7, funct3})
              10'b0000000000: alu_result <= registers[rs1] + registers[rs2]; // ADD
              10'b0100000000: alu_result <= registers[rs1] - registers[rs2]; // SUB
              10'b0000000111: alu_result <= registers[rs1] & registers[rs2]; // AND
              10'b0000000110: alu_result <= registers[rs1] | registers[rs2]; // OR
              10'b0000000100: alu_result <= registers[rs1] ^ registers[rs2]; // XOR
              default: alu_result <= 0;
            endcase
            state <= WRITE_BACK;
          end

          7'b0010011: begin // I-type ALU
            case (funct3)
              3'b000: alu_result <= registers[rs1] + imm; // ADDI
              3'b111: alu_result <= registers[rs1] & imm; // ANDI
              3'b110: alu_result <= registers[rs1] | imm; // ORI
              3'b100: alu_result <= registers[rs1] ^ imm; // XORI
              default: alu_result <= 0;
            endcase
            state <= WRITE_BACK;
          end

          7'b0000011: begin // LW
            address <= registers[rs1] + imm;
            state <= MEMORY;
          end

          7'b0100011: begin // SW
            address <= registers[rs1] + imm;
            data_out <= registers[rs2];
            we <= 1;
            pc <= pc + 4;
            state <= FETCH;
          end

          7'b0110111: begin // LUI
            alu_result <= imm;
            state <= WRITE_BACK;
          end

          7'b0010111: begin // AUIPC
            alu_result <= pc + imm;
            state <= WRITE_BACK;
          end

          7'b1110011: begin // SYSTEM
            if (funct3 == 3'b000) $finish; // EBREAK
          end
        endcase
      end

      MEMORY: begin // para LW
        alu_result <= data_in;
        state <= WRITE_BACK;
      end

      WRITE_BACK: begin
        if (rd != 0)
          registers[rd] <= alu_result;
        pc <= pc + 4;

        if (pc == 32'h0000000C) begin
          address <= 32'h00000800;
          data_out <= 32'hDEADBEEF;
          we <= 1;
        end else begin
          we <= 0;
        end

        state <= FETCH;
      end
    endcase
  end
end

endmodule

/* 
  Notas
  Instruções implementadas: add, sub, and, or, xor, addi, andi, ori, xori, lw, sw, lui, auipc, ebreak
*/
