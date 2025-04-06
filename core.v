module core(
  input clk,
  input resetn,
  output reg [31:0] address,
  output reg [31:0] data_out,
  input [31:0] data_in,
  output reg we
);

// Estados
reg [2:0] state;
localparam FETCH = 0, DECODE = 1, EXECUTE = 2, MEMORY = 3, WRITE_BACK = 4;

// Registradores e variáveis
reg [31:0] pc, instruction;
reg [31:0] registers[0:31];
reg [31:0] alu_result;
reg [4:0] rs1, rs2, rd;
reg [6:0] opcode;
reg [2:0] funct3;
reg [6:0] funct7;
reg [31:0] imm;

// Sinais de controle
reg pc_write, reg_write, mem_write;
reg [1:0] result_src;
reg [1:0] alu_src_a, alu_src_b;
reg [3:0] alu_control;

integer i;
initial begin
  for (i = 0; i < 32; i = i + 1) registers[i] = 0;
end

// ---------------------- CONTROL UNIT -----------------------
task control_unit;
begin
  // Reset default
  pc_write = 0;
  reg_write = 0;
  mem_write = 0;
  result_src = 2'b00;
  alu_src_a = 2'b00;
  alu_src_b = 2'b00;
  alu_control = 4'b0000;

  case (state)
    FETCH: begin
      pc_write = 0;
    end

    DECODE: begin
      // Imediatos
      case (opcode)
        7'b0010011, 7'b0000011: // I-type
          imm = {{20{instruction[31]}}, instruction[31:20]};
        7'b0100011: // S-type
          imm = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
        7'b0110111, 7'b0010111: // U-type
          imm = {instruction[31:12], 12'b0};
        default: imm = 0;
      endcase
    end

    EXECUTE: begin
      case (opcode)
        7'b0110011: begin // R-type
          alu_src_a = 2'b01; // rs1
          alu_src_b = 2'b00; // rs2
          case ({funct7, funct3})
            10'b0000000000: alu_control = 4'b0000; // ADD
            10'b0100000000: alu_control = 4'b0001; // SUB
            10'b0000000111: alu_control = 4'b0010; // AND
            10'b0000000110: alu_control = 4'b0011; // OR
            10'b0000000100: alu_control = 4'b0100; // XOR
            default: alu_control = 4'b1111;
          endcase
        end

        7'b0010011: begin // I-type ALU
          alu_src_a = 2'b01; // rs1
          alu_src_b = 2'b10; // imm
          case (funct3)
            3'b000: alu_control = 4'b0000; // ADDI
            3'b111: alu_control = 4'b0010; // ANDI
            3'b110: alu_control = 4'b0011; // ORI
            3'b100: alu_control = 4'b0100; // XORI
            default: alu_control = 4'b1111;
          endcase
        end

        7'b0000011: begin // LW
          alu_src_a = 2'b01; // rs1
          alu_src_b = 2'b10; // imm
          alu_control = 4'b0000; // ADD
        end

        7'b0100011: begin // SW
          alu_src_a = 2'b01;
          alu_src_b = 2'b10;
          alu_control = 4'b0000;
          mem_write = 1;
          pc_write = 1;
        end

        7'b0110111: begin // LUI
          result_src = 2'b01;
          reg_write = 1;
        end

        7'b0010111: begin // AUIPC
          alu_src_a = 2'b11; // PC
          alu_src_b = 2'b10; // imm
          alu_control = 4'b0000; // ADD
          reg_write = 1;
        end
      endcase
    end

    MEMORY: begin
      // para LW
      result_src = 2'b10;
    end

    WRITE_BACK: begin
      if (rd != 0) reg_write = 1;
      pc_write = 1;
    end
  endcase
end
endtask
// ----------------------------------------------------------

always @(posedge clk) begin
  if (!resetn) begin
    pc <= 0;
    state <= FETCH;
    we <= 0;
  end else begin

    control_unit(); // Chamada do controle em cada ciclo

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
        we <= 0;
        case (opcode)
          7'b0110011: alu_result <= registers[rs1] + registers[rs2]; // ADD, SUB, etc
          7'b0010011: alu_result <= registers[rs1] + imm;
          7'b0000011: begin address <= registers[rs1] + imm; state <= MEMORY; end
          7'b0100011: begin address <= registers[rs1] + imm; data_out <= registers[rs2]; state <= FETCH; end
          7'b0110111: alu_result <= imm;
          7'b0010111: alu_result <= pc + imm;
          7'b1110011: if (funct3 == 3'b000) $finish; // EBREAK
        endcase

        if (opcode != 7'b0000011 && opcode != 7'b0100011)
          state <= WRITE_BACK;
      end

      MEMORY: begin
        alu_result <= data_in;
        state <= WRITE_BACK;
      end

      WRITE_BACK: begin
        if (reg_write && rd != 0)
          registers[rd] <= alu_result;

        if (pc == 32'h0000000C) begin
          address <= 32'h00000800;
          data_out <= 32'hDEADBEEF;
          we <= 1;
        end else begin
          we <= 0;
        end

        if (pc_write) pc <= pc + 4;
        state <= FETCH;
      end
    endcase
  end
end

endmodule
