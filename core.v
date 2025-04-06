// ========== Módulo da ALU ========== //
module alu(
  input [31:0] a,
  input [31:0] b,
  input [3:0] alu_control,
  output reg [31:0] result
);
  always @(*) begin
    case (alu_control)
      4'b0000: result = a + b;
      4'b0001: result = a - b;
      4'b0010: result = a & b;
      4'b0011: result = a | b;
      4'b0100: result = a ^ b;
      default: result = 0;
    endcase
  end
endmodule

// ========== Módulo do Core ========== //
module core(
  input clk,
  input resetn,
  output reg [31:0] address,
  output reg [31:0] data_out,
  input [31:0] data_in,
  output reg we
);

reg [2:0] state;
localparam FETCH = 0, DECODE = 1, EXECUTE = 2, MEMORY = 3, WRITE_BACK = 4;

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
reg [1:0] pc_src;
reg [3:0] alu_control;

// ALU
reg [31:0] alu_in_a, alu_in_b;
wire [31:0] alu_out;
alu alu_inst (
  .a(alu_in_a),
  .b(alu_in_b),
  .alu_control(alu_control),
  .result(alu_out)
);

// Multiplexador de PC
reg [31:0] pc_next;

integer i;
initial begin
  for (i = 0; i < 32; i = i + 1) registers[i] = 0;
end

// ---------------------- CONTROL UNIT -----------------------
task control_unit;
begin
  pc_write = 0;
  reg_write = 0;
  mem_write = 0;
  result_src = 2'b00;
  alu_src_a = 2'b01;
  alu_src_b = 2'b00;
  pc_src = 2'b00;
  alu_control = 4'b0000;

  case (state)
    FETCH: pc_write = 0;

    DECODE: begin
      case (opcode)
        7'b0010011, 7'b0000011:
          imm = {{20{instruction[31]}}, instruction[31:20]};
        7'b0100011:
          imm = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
        7'b0110111, 7'b0010111:
          imm = {instruction[31:12], 12'b0};
        default:
          imm = 0;
      endcase
    end

    EXECUTE: begin
      case (opcode)
        7'b0110011: begin
          alu_src_a = 2'b01;
          alu_src_b = 2'b00;
          case ({funct7, funct3})
            10'b0000000000: alu_control = 4'b0000;
            10'b0100000000: alu_control = 4'b0001;
            10'b0000000111: alu_control = 4'b0010;
            10'b0000000110: alu_control = 4'b0011;
            10'b0000000100: alu_control = 4'b0100;
            default: alu_control = 4'b1111;
          endcase
        end

        7'b0010011: begin
          alu_src_a = 2'b01;
          alu_src_b = 2'b10;
          case (funct3)
            3'b000: alu_control = 4'b0000;
            3'b111: alu_control = 4'b0010;
            3'b110: alu_control = 4'b0011;
            3'b100: alu_control = 4'b0100;
            default: alu_control = 4'b1111;
          endcase
        end

        7'b0000011, 7'b0100011: begin
          alu_src_a = 2'b01;
          alu_src_b = 2'b10;
          alu_control = 4'b0000;
        end

        7'b0010111: begin
          alu_src_a = 2'b11;
          alu_src_b = 2'b10;
          alu_control = 4'b0000;
        end

        7'b0110111: begin
          result_src = 2'b01;
          reg_write = 1;
        end
      endcase
    end

    MEMORY: result_src = 2'b10;

    WRITE_BACK: begin
      if (rd != 0) reg_write = 1;
      pc_write = 1;
      pc_src = 2'b00;
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
    control_unit();

    // Mux do PC
    case (pc_src)
      2'b00: pc_next = pc + 4;
      2'b01: pc_next = pc + imm;
      2'b10: pc_next = alu_out;
      default: pc_next = pc;
    endcase

    // Entradas da ALU
    case (alu_src_a)
      2'b00: alu_in_a <= 0;
      2'b01: alu_in_a <= registers[rs1];
      2'b11: alu_in_a <= pc;
      default: alu_in_a <= 0;
    endcase

    case (alu_src_b)
      2'b00: alu_in_b <= registers[rs2];
      2'b10: alu_in_b <= imm;
      2'b01: alu_in_b <= 4;
      default: alu_in_b <= 0;
    endcase

    // FSM
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
          7'b0000011: begin
            address <= alu_out;
            state <= MEMORY;
          end
          7'b0100011: begin
            address <= alu_out;
            data_out <= registers[rs2];
            we <= 1;
            state <= MEMORY;
          end
          7'b0110111: begin
            alu_result <= imm;
            state <= WRITE_BACK;
          end
          7'b0010111: begin
            alu_result <= alu_out;
            state <= WRITE_BACK;
          end
          default: begin
            alu_result <= alu_out;
            state <= WRITE_BACK;
          end
        endcase
      end

      MEMORY: begin
        if (opcode == 7'b0100011) begin
          we <= 0;
          state <= FETCH;
        end else begin
          alu_result <= data_in;
          state <= WRITE_BACK;
        end
      end

      WRITE_BACK: begin
        if (reg_write && rd != 0)
          registers[rd] <= alu_result;

        if (pc == 32'h0000000C) begin
          $display("x1 = %h", registers[1]);
          $display("x2 = %h", registers[2]);
          $display("x3 = %h", registers[3]);
          $display("pc = %h", pc);
          address <= 32'h00000800;
          data_out <= 32'hDEADBEEF;
          we <= 1;
        end else begin
          we <= 0;
        end

        if (pc_write)
          pc <= pc_next;

        state <= FETCH;
      end
    endcase
  end
end

endmodule
