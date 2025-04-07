// ========== Módulo da ALU ========== //
module alu(
  input [31:0] a,
  input [31:0] b,
  input [3:0] alu_control, // 0000:add, 0001:sub, 0010:and, 0011:or, 0100:xor
  output reg [31:0] result
);
  always @(*) begin
    case (alu_control)
      4'b0000: result = a + b;
      4'b0001: result = a - b;
      4'b0010: result = a & b;
      4'b0011: result = a | b;
      4'b0100: result = a ^ b;
      default: result = 32'b0; // Operação inválida -> resultado 0
    endcase
  end
endmodule

// ========== Módulo do Core ========== //
module core(
  input        clk,
  input        resetn,
  output       we,         // Write Enable para memória
  output [31:0] address,    // Endereço para memória
  output [31:0] data_out,   // Dado a ser escrito na memória
  input  [31:0] data_in     // Dado lido da memória
);

  // --- Estados da Máquina de Estados ---
  localparam FETCH         = 4'b0000;
  localparam DECODE        = 4'b0001;
  localparam EXECUTE_R     = 4'b0010; // Execução R-type
  localparam EXECUTE_I     = 4'b0011; // Execução I-type (ALU)
  localparam MEM_ADDR_CALC = 4'b0100; // Calcula endereço LW/SW
  localparam MEM_READ      = 4'b0101; // Lê memória (LW)
  localparam MEM_WRITE     = 4'b0110; // Escreve memória (SW)
  localparam WB_REG        = 4'b0111; // Write-Back (ALU result)
  localparam WB_MEM        = 4'b1000; // Write-Back (Memory result)

  reg [3:0] state, next_state; // Registradores de Estado

  // --- Registradores Principais ---
  reg [31:0] pc;             // Program Counter
  reg [31:0] registers[0:31]; // Banco de Registradores

  // --- Registradores de Pipeline / Intermediários ---
  reg [31:0] instruction;    // Instrução atual (saída do Fetch)
  reg [31:0] read_data1;     // Valor lido de rs1 (saída do Decode)
  reg [31:0] read_data2;     // Valor lido de rs2 (saída do Decode)
  reg [31:0] imm_reg;        // Imediato estendido (saída do Decode)
  reg [31:0] alu_result_reg; // Resultado da ALU (saída da Execução/AddrCalc)
  reg [31:0] mem_data_reg;   // Dado lido da memória (saída do MemRead)
  reg [4:0] rd_reg;          // Registrador destino (saída do Decode)

  // --- Decodificação (fios intermediários) ---
  wire [6:0] opcode = instruction[6:0];
  wire [4:0] rd     = instruction[11:7];
  wire [2:0] funct3 = instruction[14:12];
  wire [4:0] rs1    = instruction[19:15];
  wire [4:0] rs2    = instruction[24:20];
  wire [6:0] funct7 = instruction[31:25];

  // --- Sinais de Controle (gerados combinacionalmente - devem ser REG) ---
  reg        PCWrite;     // Habilita escrita no PC
  reg        IRWrite;     // Habilita escrita no reg. de instrução
  reg        RegWrite;    // Habilita escrita no banco de registradores
  reg        MemWrite;    // Habilita escrita na memória de dados (= 'we' output)
  reg [1:0]  ALUSrcA;     // Seleciona entrada A da ALU (00=PC, 01=Reg[rs1])
  reg [1:0]  ALUSrcB;     // Seleciona entrada B da ALU (00=Reg[rs2], 01=Imm, 10=Const 4)
  reg [3:0]  ALUControl;  // Código da operação para a ALU
  reg        AdrSrc;      // Seleciona fonte do endereço (0=PC, 1=ALUOut)
  reg [0:0]  MemToReg;    // Seleciona dado do Write Back (0=ALU, 1=Mem)

  // --- Datapath ---
  // Fios para lógica combinacional intermediária
  wire [31:0] imm_i = {{20{instruction[31]}}, instruction[31:20]}; // I-type immediate
  wire [31:0] imm_s = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]}; // S-type immediate
  wire [31:0] pc_plus_4 = pc + 4;
  wire [31:0] alu_out;       // Saída da ALU (wire)

  // Regs para saídas de lógica combinacional do always@(*)
  reg [31:0] imm_comb;        // Imediato selecionado
  reg [31:0] pc_next;         // Próximo valor do PC (saída do Mux PC)
  reg [31:0] alu_in_a;        // Entrada A da ALU (saída do Mux A)
  reg [31:0] alu_in_b;        // Entrada B da ALU (saída do Mux B)
  reg [31:0] write_back_data; // Dado a ser escrito no registrador (saída do Mux WB)

  // --- Saídas do Módulo (definidas como reg e atribuídas no always@(*) ---
  reg        we_reg;
  reg [31:0] address_reg;
  reg [31:0] data_out_reg;

  // --- Variáveis de Loop ---
  integer i; // <<<<<<<<<<< MOVIDO PARA CÁ <<<<<<<<<<<<<<
  integer k; // Para o loop initial

  // Instância da ALU
  alu alu_inst (
    .a(alu_in_a),
    .b(alu_in_b),
    .alu_control(ALUControl),
    .result(alu_out) // Conecta à saída wire da ALU
  );

  // --- Lógica Combinacional (Unidade de Controle + Muxes Datapath + Saídas) ---
  always @(*) begin
    // Valores Padrão para Sinais de Controle
    PCWrite     = 1'b0;
    IRWrite     = 1'b0;
    RegWrite    = 1'b0;
    MemWrite    = 1'b0;
    ALUSrcA     = 2'b01; // Default: Reg[rs1]
    ALUSrcB     = 2'b00; // Default: Reg[rs2]
    ALUControl  = 4'b0000; // Default: ADD
    AdrSrc      = 1'b0;  // Default: PC
    MemToReg    = 1'b0;  // Default: ALU Result
    next_state  = state; // Default: fica no mesmo estado

    // --- Lógica da Unidade de Controle + Próximo Estado ---
    case (state)
      FETCH: begin
        IRWrite    = 1'b1;
        AdrSrc     = 1'b0; // Endereço = PC
        next_state = DECODE;
      end

      DECODE: begin
        // Determina próximo estado
        case (opcode)
          7'b0110011: next_state = EXECUTE_R; // R-type (ADD, SUB, AND, OR, XOR)
          7'b0010011: next_state = EXECUTE_I; // I-type (ADDI, ANDI, ORI, XORI)
          7'b0000011: next_state = (funct3 == 3'b010) ? MEM_ADDR_CALC : FETCH; // LW
          7'b0100011: next_state = (funct3 == 3'b010) ? MEM_ADDR_CALC : FETCH; // SW
          7'b1110011: begin // SYSTEM (EBREAK)
             if (funct3 == 3'b000 && instruction[31:20] == 12'b0) begin // EBREAK (Sim)
                $display("EBREAK (Sim) encountered at PC=0x%h. Stopping simulation.", pc);
                $finish;
             end
             next_state = FETCH; // Se não for EBREAK, volta
          end
          default: next_state = FETCH; // Instrução não implementada
        endcase
      end

      EXECUTE_R: begin // ADD, SUB, AND, OR, XOR
        ALUSrcA    = 2'b01; // rs1
        ALUSrcB    = 2'b00; // rs2
        case ({funct7[5], funct3})
           4'b0_000: ALUControl = 4'b0000; // ADD
           4'b1_000: ALUControl = 4'b0001; // SUB
           4'b0_111: ALUControl = 4'b0010; // AND
           4'b0_110: ALUControl = 4'b0011; // OR
           4'b0_100: ALUControl = 4'b0100; // XOR
           default:  ALUControl = 4'b1111; // Inválido
        endcase
        next_state = WB_REG;
      end

      EXECUTE_I: begin // ADDI, ANDI, ORI, XORI
        ALUSrcA    = 2'b01; // rs1
        ALUSrcB    = 2'b01; // Imm
        case (funct3)
           3'b000: ALUControl = 4'b0000; // ADDI
           3'b111: ALUControl = 4'b0010; // ANDI
           3'b110: ALUControl = 4'b0011; // ORI
           3'b100: ALUControl = 4'b0100; // XORI
           default: ALUControl = 4'b1111; // Inválido
        endcase
        next_state = WB_REG;
      end

      MEM_ADDR_CALC: begin // LW, SW
        ALUSrcA    = 2'b01; // rs1
        ALUSrcB    = 2'b01; // Imm (I ou S)
        ALUControl = 4'b0000; // ADD para calcular endereço
        next_state = (opcode == 7'b0000011) ? MEM_READ : MEM_WRITE;
      end

      MEM_READ: begin // LW
        AdrSrc     = 1'b1; // Endereço = Saída da ALU (registrada)
        next_state = WB_MEM;
      end

      MEM_WRITE: begin // SW
        AdrSrc     = 1'b1;  // Endereço = Saída da ALU (registrada)
        MemWrite   = 1'b1;
        PCWrite    = 1'b1;
        next_state = FETCH;
      end

      WB_REG: begin // Write-Back de R-type, I-type ALU
        RegWrite   = 1'b1;
        MemToReg   = 1'b0;  // Dado vem da ALU (registrada)
        PCWrite    = 1'b1;
        next_state = FETCH;
      end

      WB_MEM: begin // Write-Back de LW
        RegWrite   = 1'b1;
        MemToReg   = 1'b1;  // Dado vem da Memória (registrado)
        PCWrite    = 1'b1;
        next_state = FETCH;
      end

      default: begin
        next_state = FETCH; // Estado inválido
      end
    endcase

    // --- Lógica Combinacional do Datapath (Muxes) ---

    // Seleção do Imediato (agora atribuído a imm_comb reg)
     case (opcode) // Baseado na instrução *atual* (no registrador instruction)
        7'b0010011: imm_comb = imm_i; // I-type ALU
        7'b0000011: imm_comb = imm_i; // LW
        7'b0100011: imm_comb = imm_s; // SW
        default:    imm_comb = 32'b0;
     endcase

    // Mux Entrada A da ALU (agora atribuído a alu_in_a reg)
    // Usa read_data1 (registrado no ciclo anterior)
    alu_in_a = (ALUSrcA == 2'b00) ? pc : read_data1;

    // Mux Entrada B da ALU (agora atribuído a alu_in_b reg)
    // Usa read_data2 e imm_reg (registrados no ciclo anterior)
    case (ALUSrcB)
      2'b00:  alu_in_b = read_data2;
      2'b01:  alu_in_b = imm_reg;
      2'b10:  alu_in_b = 32'd4;
      default: alu_in_b = 32'b0;
    endcase

    // Mux Seleção Próximo PC (agora atribuído a pc_next reg)
    pc_next = pc_plus_4;

    // Mux Seleção Dado Write Back (agora atribuído a write_back_data reg)
    // Usa alu_result_reg e mem_data_reg (registrados em ciclos anteriores)
    write_back_data = (MemToReg == 1'b1) ? mem_data_reg : alu_result_reg;

    // --- Geração das Saídas Combinacionais (atribuído aos regs de saída) ---
    // Saída de Endereço
    address_reg = (AdrSrc == 1'b1) ? alu_result_reg : pc; // Usa PC ou resultado da ALU *registrado*

    // Saída WE
    we_reg = MemWrite;

    // Saída de Dado (Só relevante quando we=1)
    data_out_reg = read_data2; // Usa valor de rs2 *registrado*

  end // Fim do always @(*)

  // --- Atribuições Contínuas para Saídas do Módulo ---
  assign address = address_reg;
  assign we = we_reg;
  assign data_out = data_out_reg;

  // --- Lógica Sequencial (Atualização de Registradores) ---
  always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      // Reset assíncrono
      pc <= 32'b0;
      state <= FETCH;
      // Zera registradores do banco
      for (i = 0; i < 32; i = i + 1) begin // Loop usa 'i' declarado no escopo do módulo
        registers[i] <= 32'b0;
      end
       // Zera regs de pipeline
       instruction <= 32'b0;
       read_data1 <= 32'b0;
       read_data2 <= 32'b0;
       imm_reg <= 32'b0;
       alu_result_reg <= 32'b0;
       mem_data_reg <= 32'b0;
       rd_reg <= 5'b0;

    end else begin
      // Atualizações síncronas baseadas nos sinais de controle

      // Atualiza Estado
      state <= next_state;

      // Atualiza PC
      if (PCWrite) begin
        pc <= pc_next;
      end

      // Atualiza Registrador de Instrução
      if (IRWrite) begin
        instruction <= data_in;
      end

      // Atualiza Registradores de Pipeline (no final do ciclo DECODE)
      // Esses valores são lidos/calculados combinacionalmente durante DECODE
      // e registrados aqui para serem usados nos ciclos seguintes.
      if (state == DECODE) begin
         read_data1 <= registers[rs1]; // Lê valor atual do banco
         read_data2 <= registers[rs2]; // Lê valor atual do banco
         imm_reg    <= imm_comb;     // Registra o imediato calculado combinacionalmente
         rd_reg     <= rd;           // Registra o rd da instrução atual
      end

      // Atualiza Registrador de Resultado da ALU (no final da EXECUTE/MEM_ADDR_CALC)
      // Registra a saída *atual* da ALU (alu_out wire)
      if (state == EXECUTE_R || state == EXECUTE_I || state == MEM_ADDR_CALC) begin
         alu_result_reg <= alu_out;
      end

      // Atualiza Registrador de Dado da Memória (no final do MEM_READ)
       if (state == MEM_READ) begin
         mem_data_reg <= data_in; // Registra dado lido da memória
       end

      // Escreve no Banco de Registradores (no final do WB_REG/WB_MEM)
      // Usa rd_reg (registrado no Decode) e write_back_data (calculado combinacionalmente)
      if (RegWrite && rd_reg != 5'b0) begin
        registers[rd_reg] <= write_back_data;
      end

    end // Fim else do reset
  end // Fim do always @(posedge clk or negedge resetn)

  // Inicialização do banco de registradores (executa uma vez no início da simulação)
   initial begin
       for (k = 0; k < 32; k = k + 1) begin
           registers[k] = 32'b0;
       end
   end

endmodule // Fim do core