// == Módulo da ALU
module alu(
  input [31:0] a,            // Operando A
  input [31:0] b,            // Operando B (neste caso, b[4:0] é usado para shift)
  input [3:0] alu_control,   // Código de controle da operação ALU
  output reg [31:0] result   // Resultado da operação
);
  always @(*) begin
    case (alu_control)
      4'b0000: result = a + b;            // Adição (ADDI ou ADD)
      4'b0001: result = a - b;            // Subtração (SUB)
      4'b0010: result = a & b;            // AND
      4'b0011: result = a | b;            // OR
      4'b0100: result = a ^ b;            // XOR
      4'b0101: result = a << b[4:0];       // SLLI (deslocamento lógico à esquerda)
      4'b0110: result = a >> b[4:0];       // SRLI (deslocamento lógico à direita)
      4'b0111: result = $signed(a) >>> b[4:0]; // SRAI (deslocamento aritmético à direita)
      4'b1000: result = ($signed(a) < $signed(b)) ? 32'd1 : 32'd0; // SLT (set on less than - com sinal)
      4'b1001: result = (a < b) ? 32'd1 : 32'd0;                    // SLTU (set on less than - sem sinal)
      default: result = 32'b0;            // Operação inválida
    endcase
  end
endmodule

// == Módulo do processador principal (core)
module core(
  input        clk,         // Clock
  input        resetn,      // Reset ativo em 0
  output       we,          // Sinal de escrita na memória
  output [31:0] address,    // Endereço de acesso à memória
  output [31:0] data_out,   // Dados a serem escritos na memória
  input  [31:0] data_in     // Dados lidos da memória
);

  // Máquina de estados
  reg [3:0] state, next_state;

  // Registrador de programa (PC) e banco de registradores
  reg [31:0] pc;
  reg [31:0] registers[0:31];

  // Registradores auxiliares
  reg [31:0] instruction;
  reg [31:0] read_data1;
  reg [31:0] read_data2;
  reg [31:0] imm_reg;
  reg [31:0] alu_result_reg;
  reg [31:0] mem_data_reg;
  reg [4:0]  rd_reg; // Registrador destino

  // Campos da instrução
  wire [6:0] opcode = instruction[6:0];
  wire [4:0] rd     = instruction[11:7];
  wire [2:0] funct3 = instruction[14:12];
  wire [4:0] rs1    = instruction[19:15];
  wire [4:0] rs2    = instruction[24:20];
  wire [6:0] funct7 = instruction[31:25];

  // Sinais de controle
  reg        PCWrite;
  reg        IRWrite;
  reg        RegWrite;
  reg        MemWrite;
  reg [1:0]  ALUSrcA;
  reg [1:0]  ALUSrcB;
  reg [3:0]  ALUControl;
  reg        AdrSrc;
  reg [0:0]  MemToReg;

  // Imediatos
  wire [31:0] imm_i = {{20{instruction[31]}}, instruction[31:20]};
  wire [31:0] imm_s = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
  wire [31:0] imm_j = {{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0};
  wire [31:0] imm_u = {instruction[31:12], 12'b0};

  // PC + 4
  wire [31:0] pc_plus_4 = pc + 4;
  wire [31:0] alu_out;

  // Registradores auxiliares combinacionais
  reg [31:0] imm_comb;
  reg [31:0] pc_next;
  reg [31:0] alu_in_a;
  reg [31:0] alu_in_b;
  reg [31:0] write_back_data;

  // Endereço e dados de escrita na memória (sinais de saída)
  reg        we_reg;
  reg [31:0] address_reg;
  reg [31:0] data_out_reg;

  integer i;
  integer k;

  // Instância da ALU
  alu alu_inst (
    .a(alu_in_a),
    .b(alu_in_b),
    .alu_control(ALUControl),
    .result(alu_out)
  );

  // Lógica combinacional de controle e datapath
  always @(*) begin
    // Sinais padrão
    PCWrite      = 1'b0;
    IRWrite      = 1'b0;
    RegWrite     = 1'b0;
    MemWrite     = 1'b0;
    ALUSrcA      = 2'b01;
    ALUSrcB      = 2'b00;
    ALUControl   = 4'b0000;
    AdrSrc       = 1'b0;
    MemToReg     = 1'b0;
    next_state   = state;

    // Máquina de estados
    case (state)
      4'b0000: begin // FETCH
        IRWrite    = 1'b1;
        AdrSrc     = 1'b0;
        next_state = 4'b0001;
      end

      4'b0001: begin // DECODE
        // Carrega os operandos e imediato
        // Observação: Para instruções de branch, o imediato é calculado conforme o B-type
        // e já será armazenado em imm_reg.
        case (opcode)
          7'b0110011: next_state = 4'b0010; // R-type
          7'b0010011: next_state = 4'b0011; // I-type (ADDI, ANDI, ORI, XORI, SLLI, SRLI, SRAI)
          7'b0000011: next_state = 4'b0100; // LOAD
          7'b0100011: next_state = 4'b0100; // STORE
          7'b1101111: next_state = 4'b0011; // JAL
          7'b0110111: next_state = 4'b0011; // LUI
          7'b1110011: begin // EBREAK
                        if (funct3 == 3'b000 && instruction[31:20] == 12'b0) begin
                          PCWrite   = 1'b1;
                          pc_next   = 32'h00000FFC; // Termina simulação
                        end
                        next_state = 4'b0000;
                      end
          7'b1100011: next_state = 4'b1010; // Branch: BEQ e BNE
          default: next_state = 4'b0000; // Instrução inválida
        endcase
      end

      4'b0010: begin // EXECUTE_R (R-type)
        ALUSrcA = 2'b01;
        ALUSrcB = 2'b00;
        // Verifica instruções de comparação:
        // SLT: funct3 = 010, SLTU: funct3 = 011, com funct7 = 0000000.
        if (funct3 == 3'b010 && funct7 == 7'b0000000)
          ALUControl = 4'b1000; // SLT
        else if (funct3 == 3'b011 && funct7 == 7'b0000000)
          ALUControl = 4'b1001; // SLTU
        else begin
          // Instruções aritméticas comuns
          case ({funct7[5], funct3})
            4'b0_000: ALUControl = 4'b0000; // ADD
            4'b1_000: ALUControl = 4'b0001; // SUB
            4'b0_111: ALUControl = 4'b0010; // AND
            4'b0_110: ALUControl = 4'b0011; // OR
            4'b0_100: ALUControl = 4'b0100; // XOR
            default:  ALUControl = 4'b1111; // Não implementado
          endcase
        end
        next_state = 4'b0111; // Vai para write-back
      end

      4'b0011: begin // EXECUTE_I (I-type, JAL ou LUI)
        if (opcode == 7'b1101111) begin // JAL
          PCWrite   = 1'b1;
          RegWrite  = 1'b1;
          ALUSrcA   = 2'b00;
          ALUSrcB   = 2'b10;
        end
        else if (opcode == 7'b0110111) begin // LUI
          RegWrite  = 1'b1;
        end
        else begin // Outras instruções I-type
          ALUSrcA = 2'b01;
          ALUSrcB = 2'b01;
          case (funct3)
            3'b000: ALUControl = 4'b0000; // ADDI
            3'b001: ALUControl = 4'b0101; // SLLI
            3'b101: begin
                      if (funct7 == 7'b0000000)
                        ALUControl = 4'b0110; // SRLI
                      else if (funct7 == 7'b0100000)
                        ALUControl = 4'b0111; // SRAI
                      else
                        ALUControl = 4'b1111;
                    end
            3'b111: ALUControl = 4'b0010; // ANDI
            3'b110: ALUControl = 4'b0011; // ORI
            3'b100: ALUControl = 4'b0100; // XORI
            default: ALUControl = 4'b1111;
          endcase
        end
        next_state = 4'b0111;
      end

      4'b0100: begin // MEM_ADDR_CALC para LOAD e STORE
        ALUSrcA = 2'b01;
        ALUSrcB = 2'b01;
        ALUControl = 4'b0000; // ADD
        next_state = (opcode == 7'b0000011) ? 4'b0101 : 4'b0110; // Load ou Store
      end

      4'b0101: begin // MEM_READ
        AdrSrc = 1'b1;
        next_state = 4'b1000;
      end

      4'b0110: begin // MEM_WRITE
        AdrSrc = 1'b1;
        MemWrite = 1'b1;
        PCWrite = 1'b1;
        next_state = 4'b0000;
      end

      4'b0111: begin // WB_REG (write-back para operações aritméticas)
        RegWrite = 1'b1;
        MemToReg = 1'b0;
        PCWrite = 1'b1;
        next_state = 4'b0000;
      end

      4'b1000: begin // WB_MEM (write-back para load)
        RegWrite = 1'b1;
        MemToReg = 1'b1;
        PCWrite = 1'b1;
        next_state = 4'b0000;
      end

      4'b1010: begin // BRANCH (BEQ e BNE)
        // Avalia a condição de branch:
        // BEQ (funct3 == 000): branch se x[rs1] == x[rs2]
        // BNE (funct3 == 001): branch se x[rs1] != x[rs2]
        if ((funct3 == 3'b000 && read_data1 == read_data2) ||
            (funct3 == 3'b001 && read_data1 != read_data2)) begin
          PCWrite = 1'b1;
          pc_next = pc + imm_reg;  // Utiliza imediato B-type já em imm_reg
        end else begin
          PCWrite = 1'b1;
          pc_next = pc_plus_4;
        end
        next_state = 4'b0000;
      end

      default: next_state = 4'b0000;
    endcase

    // Seleção do imediato conforme formatos (I, S, J, U e B)
    case (opcode)
      7'b0010011: imm_comb = imm_i;
      7'b0000011: imm_comb = imm_i;  // LW utiliza o imediato I
      7'b0100011: imm_comb = imm_s;
      7'b1101111: imm_comb = imm_j;
      7'b0110111: imm_comb = imm_u;
      7'b1100011: imm_comb = {{20{instruction[31]}}, instruction[7],
                              instruction[30:25], instruction[11:8], 1'b0}; // Branch (B-type)
      default:    imm_comb = 32'b0;
    endcase

    // Entradas da ALU
    alu_in_a = (ALUSrcA == 2'b00) ? pc : read_data1;
    case (ALUSrcB)
      2'b00: alu_in_b = read_data2;
      2'b01: alu_in_b = imm_reg;
      2'b10: alu_in_b = 32'd4;
      default: alu_in_b = 32'b0;
    endcase

    // Próximo PC (padrão: PC+4, exceto se branch ou JAL)
    pc_next = (opcode == 7'b1101111) ? (pc + imm_j) : pc_plus_4;

    // Seleção do dado para escrita no registrador
    write_back_data = (opcode == 7'b0110111) ? imm_reg :
                      (MemToReg == 1'b1)     ? mem_data_reg : alu_result_reg;

    // Geração dos sinais de endereço e escrita para memória
    address_reg = (AdrSrc == 1'b1) ? alu_result_reg : pc;
    we_reg = MemWrite;
    data_out_reg = read_data2;
  end

  // Atribuições para as saídas do módulo
  assign address = address_reg;
  assign we = we_reg;
  assign data_out = data_out_reg;

  // Bloco sequencial (executa em fronteira de clock)
  always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      pc <= 32'b0;
      state <= 4'b0000;
      for (i = 0; i < 32; i = i + 1)
        registers[i] <= 32'b0;
      instruction     <= 32'b0;
      read_data1      <= 32'b0;
      read_data2      <= 32'b0;
      imm_reg         <= 32'b0;
      alu_result_reg  <= 32'b0;
      mem_data_reg    <= 32'b0;
      rd_reg          <= 5'b0;
    end else begin
      state <= next_state;
      if (PCWrite)
        pc <= pc_next;
      if (IRWrite)
        instruction <= data_in;
      if (state == 4'b0001) begin // DECODE: carrega operandos e imediato
        read_data1 <= registers[rs1];
        read_data2 <= registers[rs2];
        imm_reg    <= imm_comb;
        rd_reg     <= rd;
      end
      if (state == 4'b0010 || state == 4'b0011 || state == 4'b0100)
        alu_result_reg <= alu_out;
      if (state == 4'b0101)
        mem_data_reg <= data_in;
      if (RegWrite && rd_reg != 5'b0)
        registers[rd_reg] <= write_back_data;
    end
  end

  // Inicialização dos registradores
  initial begin
    for (k = 0; k < 32; k = k + 1)
      registers[k] = 32'b0;
  end
endmodule