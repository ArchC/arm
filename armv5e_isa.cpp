/********************************************************/
/* The ArchC ARMv5e functional model.                   */
/* Author: Danilo Marcolin Caravana                     */
/*                                                      */
/* For more information on ArchC, please visit:         */
/* http://www.archc.org                                 */
/*                                                      */
/* The ArchC Team                                       */
/* Computer Systems Laboratory (LSC)                    */
/* IC-UNICAMP                                           */
/* http://www.lsc.ic.unicamp.br                         */
/********************************************************/

#include "armv5e_isa.H"
#include "armv5e_isa_init.cpp"
#include "armv5e_bhv_macros.H"

using namespace armv5e_parms;

//DEBUG
static const int DEBUG_INSTR = 1;

//#define DEBUG

#ifdef DEBUG
#include <stdarg.h>
inline int dprintf(const char *format, ...) {
  int ret;
  if (DEBUG_INSTR) {
    va_list args;
    va_start(args, format);
    ret = vfprintf(ac_err, format, args);
    va_end(args);
  }
  return ret;
}
#else
inline void dprintf(const char *format, ...) {}
#endif

// Macros: Position == 0 (Right most bit)
#define isBitSet(variable, position) (((variable & (1 << position)) != 0) ? true : false) 
#define getBit(variable, position) (((variable & (1 << position)) != 0) ? true : false)
#define setBit(variable, position) variable = variable | (1 << position)
#define clearBit(variable, position) variable = variable & (~(1 << position))

// Declaracoes de tipos e variaveis
#define LR 14 // link return
#define PC 15 // program counter

typedef struct flag_s {
  bool N; // Negative
  bool Z; // Zero
  bool C; // Carry
  bool V; // Overflow
  bool Q; // DSP
  bool T; // Thumb
} flag_t;

typedef union {
  char byte[4];
  long entire;
} reg_t;

typedef union {
  long reg[2];
  long long hilo;
} r64bit_t;

static flag_t flags;
static bool execute;

static reg_t dpi_shiftop;
static bool dpi_shiftopcarry;

static reg_t ls_address;
static reg_t lsm_startaddress;
static reg_t lsm_endaddress;

static reg_t OP1;
static reg_t OP2;

// Funcoes auxiliares
inline reg_t ArithmeticShiftRight(int shiftamount, reg_t reg) {

  reg_t tmp = reg;
  tmp.entire = ((signed long)tmp.entire) >> shiftamount;
  return tmp;
}

inline reg_t RotateRight(int shiftamount, reg_t reg) {

  reg_t ret;
  ret.entire = (((unsigned long)reg.entire) >> shiftamount) | (((unsigned long)reg.entire) << (32 - shiftamount));
 
  return ret;
}

inline long SignExtend(signed long word, int word_length) {
  return (signed long)((signed long)(word << (32 - word_length))) >> (32 - word_length);
}

inline int LSM_CountSetBits(reg_t registerList) {
  int i, count;

  count = 0;
  for (i=0; i<16; i++) { // verificar os limites de acordo com big/little endian
    if (isBitSet(registerList.entire,i)) count++;
  }
  return count;
}

reg_t CPSRBuild() {

  reg_t CPSR;

  CPSR.entire = 0;
  setBit(CPSR.entire,4); // user mode
  setBit(CPSR.entire,6); // FIQ disable
  setBit(CPSR.entire,7); // IRQ disable
  if (flags.N) setBit(CPSR.entire,31); // N flag
  else clearBit(CPSR.entire,31);
  if (flags.Z) setBit(CPSR.entire,30); // Z flag
  else clearBit(CPSR.entire,30);
  if (flags.C) setBit(CPSR.entire,29); // C flag
  else clearBit(CPSR.entire,29);
  if (flags.V) setBit(CPSR.entire,28); // V flag
  else clearBit(CPSR.entire,28);
  if (flags.Q) setBit(CPSR.entire,27); // Q flag
  else clearBit(CPSR.entire,27);
  if (flags.T) setBit(CPSR.entire, 5); // T flag

  return CPSR;
}

//!Generic instruction behavior method.
void ac_behavior( instruction ) {

  dprintf("----- PC=%#x ----- %lld\n", (unsigned int)ac_pc, ac_instr_counter);

  // Tratamento do campo COND
  execute = false;
  dprintf("cond=%d\n", cond);

  switch(cond) {
    case  0: if (flags.Z == true) execute = true; break;
    case  1: if (flags.Z == false) execute = true; break;
    case  2: if (flags.C == true) execute = true; break;
    case  3: if (flags.C == false) execute = true; break;
    case  4: if (flags.N == true) execute = true; break;
    case  5: if (flags.N == false) execute = true; break; 
    case  6: if (flags.V == true) execute = true; break;
    case  7: if (flags.V == false) execute = true; break; 
    case  8: if ((flags.C == true)&&(flags.Z == false)) execute = true; break; 
    case  9: if ((flags.C == false)||(flags.Z == true)) execute = true; break;
    case 10: if (flags.N == flags.V) execute = true; break;
    case 11: if (flags.N != flags.V) execute = true; break; 
    case 12: if ((flags.Z == false)&&(flags.N == flags.V)) execute = true; break;
    case 13: if ((flags.Z == true)||(flags.N != flags.V)) execute = true;  break;
    case 14: execute = true; break;
    default: execute = false;
  }

  // Incremento do PC
  ac_pc += 4;
  RB.write(PC, ac_pc);

  if (execute) 
    dprintf("Instrucao sera executada\n");
  else {
    dprintf("Instrucao NAO sera executada\n");
    ac_annul();
  }
}
 
// Instruction Format behavior methods.

// DPI1 - Segundo operador �registrador com shift por imediato
void ac_behavior( Type_DPI1 ) {

  reg_t RM2;

  dprintf("Tipo da instrucao: DPI1\n");
  
  // Caso especial: rm = 15
  if (rm == 15) {
    dprintf("Rm=PC -> usa-se Rm=Rm+8\n");
    // PC j�esta somado de 4 do behavior geral de instru�o
    RM2.entire = RB.read(rm) + 4;
  }
  else RM2.entire = RB.read(rm);
      
  switch(shift) {
  case 0: // Logical shift left
    dprintf("shift=00 -> Logical Shift Left\nshiftamount=%d\n", shiftamount);
    if ((shiftamount >= 0) && (shiftamount <= 31)) {
      if (shiftamount == 0) {
	dpi_shiftop.entire = RM2.entire;
	dpi_shiftopcarry = flags.C;
      } else {
	dpi_shiftop.entire = RM2.entire << shiftamount;
	dpi_shiftopcarry = getBit(RM2.entire, 32 - shiftamount);
      }
    }
    break;
  case 1: // Logical shift right
    dprintf("shift=01 -> Logical Shift Right\nshiftamount=%d\n", shiftamount);
    if ((shiftamount >= 0) && (shiftamount <= 31)) {
      if (shiftamount == 0) {
	dpi_shiftop.entire = 0;
	dpi_shiftopcarry = getBit(RM2.entire, 31);
      } else {
	dpi_shiftop.entire = ((unsigned long) RM2.entire) >> shiftamount;
	dpi_shiftopcarry = getBit(RM2.entire, shiftamount - 1);
      }
    }
    break;
  case 2: // Arithmetic shift right
    dprintf("shift=10 -> Arithmetic Shift Right\nshiftamount=%d\n", shiftamount);
    if ((shiftamount >= 0) && (shiftamount <= 31)) {
      if (shiftamount == 0) {
	if (!isBitSet(RM2.entire, 31)) {
	  dpi_shiftop.entire = 0;
	  dpi_shiftopcarry = getBit(RM2.entire, 31);
	} else {
	  dpi_shiftop.entire = 0xFFFFFFFF;
	  dpi_shiftopcarry = getBit(RM2.entire, 31);
	}
      } else {
	dpi_shiftop.entire = ((signed long) RM2.entire) >> shiftamount;
	dpi_shiftopcarry = getBit(RM2.entire, shiftamount - 1);
      }
    }
    break;
  default: // Rotate right
    dprintf("shift=11 -> Rotate Right\nshiftamount=%d\n", shiftamount);
    if ((shiftamount >= 0) && (shiftamount <= 31)) {
      if (shiftamount == 0) { //Rotate right with extend
	dpi_shiftopcarry = getBit(RM2.entire, 0);
	dpi_shiftop.entire = (((unsigned long)RM2.entire) >> 1);
	if (flags.C) setBit(dpi_shiftop.entire, 31);
      } else {
	dpi_shiftop.entire = (RotateRight(shiftamount, RM2)).entire;
	dpi_shiftopcarry = getBit(RM2.entire, shiftamount - 1);
      }
    }
  }
  dprintf("Valores finais do shifter operand:\ndpi_shiftop=%ld\ndpi_shiftopcarry=%d\n", dpi_shiftop.entire,dpi_shiftopcarry);
}

// DPI2 - Segundo operador �registrador com shift por registrador
void ac_behavior( Type_DPI2 ) {

  int rs40;
  reg_t RS2, RM2;

  dprintf("Tipo da instrucao: DPI2\n");

  // Caso especial: r* = 15
  if ((rd == 15)||(rm == 15)||(rn == 15)||(rs == 15)) {
    printf("O registrador 15 nao pode ser usado nessa instrucao\n");
    ac_annul(); 
  }

  RM2.entire = RB.read(rm);
  RS2.entire = RB.read(rs);
  rs40 = ((unsigned int)RS2.entire) & 0x0000000F;

  switch(shift){
  case 0: // Logical shift left
    dprintf("shift=00 -> Logical Shift Left\nRS2.byte[0]=%d\n", RS2.byte[0]);
    if (RS2.byte[0] == 0) {
      dpi_shiftop.entire = RM2.entire;
      dpi_shiftopcarry = flags.C;
    }
    else if (((unsigned char)RS2.byte[0]) < 32) {
      dpi_shiftop.entire = RM2.entire << (unsigned char)RS2.byte[0];
      dpi_shiftopcarry = getBit(RM2.entire, 32 - ((unsigned char)RS2.byte[0]));
    }
    else if (RS2.byte[0] == 32) {
      dpi_shiftop.entire = 0;
      dpi_shiftopcarry = getBit(RM2.entire, 0);
    }
    else { // rs > 32
      dpi_shiftop.entire = 0;
      dpi_shiftopcarry = 0;
    }  
    break;
  case 1: // Logical shift right
    dprintf("shift=01 -> Logical Shift Right\nRS2.byte[0]=%d\n", RS2.byte[0]);
    if (RS2.byte[0] == 0) {
      dpi_shiftop.entire = RM2.entire;
      dpi_shiftopcarry = flags.C;
    }
    else if (((unsigned char)RS2.byte[0]) < 32) {
      dpi_shiftop.entire = ((unsigned long) RM2.entire) >> ((unsigned char)RS2.byte[0]);
      dpi_shiftopcarry = getBit(RM2.entire, (unsigned char)RS2.byte[0] - 1);
    }
    else if (RS2.byte[0] == 32) {
      dpi_shiftop.entire = 0;
      dpi_shiftopcarry = getBit(RM2.entire, 31);
    }
    else { // rs > 32
      dpi_shiftop.entire = 0;
      dpi_shiftopcarry = 0;
    }  
    break;
  case 2: // Arithmetical shift right
    dprintf("shift=10 -> Arithmetic Shift Right\nRS2.byte[0]=%d\nrs40=%d\n", RS2.byte[0],rs40);
    if (RS2.byte[0] == 0) {
      dpi_shiftop.entire = RM2.entire;
      dpi_shiftopcarry = flags.C;
    }
    else if (((unsigned char)RS2.byte[0]) < 32) {
      dpi_shiftop.entire = ((signed long) RM2.entire) >> ((unsigned char)RS2.byte[0]);
      dpi_shiftopcarry = getBit(RM2.entire, ((unsigned char)RS2.byte[0]) - 1);
    } else { // rs >= 32
      if (!isBitSet(RM2.entire, 31)) {
	dpi_shiftop.entire = 0;
	dpi_shiftopcarry = getBit(RM2.entire, 31);
      }
      else { // rm_31 == 1
	dpi_shiftop.entire = 0xFFFFFFFF;
	dpi_shiftopcarry = getBit(RM2.entire, 31);
      }
    }
    break;
  default: // Rotate right
    dprintf("shift=11 -> Rotate Right\nRS2.byte[0]=%d\nrs40=%d\n", RS2.byte[0],rs40);
    if (RS2.byte[0] == 0) {
      dpi_shiftop.entire = RM2.entire;
      dpi_shiftopcarry = flags.C;
    }
    else if (rs40 == 0) {
      dpi_shiftop.entire = RM2.entire;
      dpi_shiftopcarry = getBit(RM2.entire, 31);
    }
    else { // rs40 > 0 
      dpi_shiftop.entire = (RotateRight(rs40, RM2)).entire;
      dpi_shiftopcarry = getBit(RM2.entire, rs40 - 1);
    }
  }
  dprintf("Valores finais do shifter operand:\ndpi_shiftop=%ld\ndpi_shiftopcarry=%d\n", dpi_shiftop.entire,dpi_shiftopcarry);
}

// DPI3 - Segundo operador �imediato com shift por imediato
void ac_behavior( Type_DPI3 ){

  long tmp;

  dprintf("Tipo da instrucao: DPI3\n");
  dprintf("rotate=%d\nimm8=%d\n", rotate,imm8);

  tmp = (unsigned long)imm8;
  dpi_shiftop.entire = (((unsigned long)tmp) >> (2 * rotate)) | (((unsigned long)tmp) << (32 - (2 * rotate)));

  if (rotate == 0) 
    dpi_shiftopcarry = flags.C;
  else 
    dpi_shiftopcarry = getBit(dpi_shiftop.entire, 31);
    
  dprintf("Valores finais do shifter operand:\ndpi_shiftop=%ld\ndpi_shiftopcarry=%d\n", dpi_shiftop.entire,dpi_shiftopcarry);
}

void ac_behavior( Type_BBL ) {
  dprintf("Tipo da instrucao: BBL\n");
  // nada a ser implementado
}
void ac_behavior( Type_BBLT ) {
  dprintf("Tipo da instrucao: BBLT\n");
  // nada a ser implementado
}
void ac_behavior( Type_MBXBLX ) {
  dprintf("Tipo da instrucao: MBXBLX\n");
  // nada a ser implementado
}

// MULT1 - Multiplicacao com resultado de 32 bits
void ac_behavior( Type_MULT1 ) {
  dprintf("Tipo da instrucao: MULT1\n");
  // nada a ser implementado
}

// MULT2 - Multiplicacao com resultado de 64 bits
void ac_behavior( Type_MULT2 ) {
  dprintf("Tipo da instrucao: MULT2\n");
  // nada a ser implementado
}

// LSI - Load Store Immediate Offset/Index
void ac_behavior( Type_LSI ) {

  reg_t RN2;
  RN2.entire = RB.read(rn);
  
  dprintf("rn=%d, contendo %ld\n", rn,RN2.entire);

  dprintf("Tipo de instrucao: LSI\n");
  ls_address.entire = 0;
    
  if((p == 1)&&(w == 0)) { // immediate pre-indexed without writeback
    dprintf("Caso: Immediate pre-indexed without writeback\n");
    
    // Caso especial: Rn = PC
    if (rn == PC) 
      ls_address.entire = 4;
    
    if(u == 1) {
      dprintf("Soma imediato (u == 1)\n");
      //checar se imm12 soma direto
      ls_address.entire += RN2.entire + (unsigned long) imm12;
    } else {
      dprintf("Subtrai imediato (u == 0)\n");
      ls_address.entire += RN2.entire - (unsigned long) imm12;
    }
    dprintf("Endereco (ls_address) = %ld\n", ls_address.entire);
  }

  else if((p == 1)&&(w == 1)) { // immediate pre-indexed with writeback
    dprintf("Caso: Immediate pre-indexed with writeback\n");

    // Caso especial: Rn = PC
    if (rn == PC) {
      printf("Resultado do tipo LSI imprevisivel (Can't Writeback to PC, Rn = PC)\n");
      ac_annul();
      return;
    }
    // Caso especial Rn = Rd
    if (rn == rd) {
      printf("Resultado do tipo LSI imprevisivel (Can't Writeback to loaded Register, Rn = Rd)\n");
      ac_annul();
      return;
    }
    
    if(u == 1) {
      //checar se imm12 soma direto   
      dprintf("Soma imediato (u == 1)\n");
      ls_address.entire = RN2.entire + (unsigned long) imm12;
    } else {
      dprintf("Subtrai imediato (u == 0)\n");
      ls_address.entire = RN2.entire - (unsigned long) imm12;
    }
    RB.write(rn,ls_address.entire);
    dprintf("Endereco (ls_address) = %ld\n", ls_address.entire);
  }

  else if((p == 0)&&(w == 0)) { // immediate post-indexed (writeback)
    dprintf("Caso: Immediate post-indexed\n");    

    // Caso especial: Rn = PC
    if (rn == PC) {
      printf("Resultado do tipo LSI imprevisivel (Can't Writeback to PC, Rn = PC)\n");
      ac_annul();
      return;
    }
    // Caso especial Rn = Rd
    if (rn == rd) {
      printf("Resultado do tipo LSI imprevisivel (Can't Writeback to loaded Register, Rn = Rd)\n");
      ac_annul();
      return;
    }
    
    ls_address.entire = RN2.entire;
    if(u == 1) {
      //checar se imm12 soma direto
      dprintf("Soma imediato (u == 1)\n");
      RB.write(rn, ls_address.entire + (unsigned long) imm12);
    } else {
      dprintf("Subtrai imediato (u == 0)\n");
      RB.write(rn, ls_address.entire - (unsigned long) imm12);
    }
    dprintf("Endereco (ls_address) = %ld\n", ls_address.entire);
  }
  /* FIXME: Checar Alinhamento de Word (Rd = PC) Address[1:0] = 0b00 */

}

// LSR - Scaled Register Offset/Index
void ac_behavior( Type_LSR ) {

  reg_t RM2, RN2, index, tmp;

  dprintf("Tipo de instrucao: LSR\n");
  RM2.entire = RB.read(rm);
  RN2.entire = RB.read(rn);
  dprintf("rm=%d, contendo %ld\nrn=%d, contendo %ld\n", rm,RM2.entire,rn,RN2.entire);
  ls_address.entire = 0;

  if ((p == 1)&&(w == 0)) { // offset
    dprintf("Caso: pre-indexed without writeback\n");

    // Caso especial: PC
    if(rn == PC) 
      ls_address.entire = 4;

    if(rm == PC) {
      printf("Resultado do tipo LSR imprevisivel (Ilegal usage of PC, Rm = PC)\n");
      return;
    }

    switch(shift){
    case 0:
      if(shiftamount == 0) { // Register
	dprintf("shift=00, shiftamount=0 -> Register\n");
	index.entire = RM2.entire;
      } else { // Scalled logical shift left
	dprintf("shift=00, shiftamount=%d -> Scalled logical shift left\n", shiftamount);
	index.entire = RM2.entire << shiftamount;
      }
      break;
    case 1: // logical shift right
      dprintf("shift=01 -> Logical Shift Right\nshiftamount=%d\n", shiftamount);
      if(shiftamount == 0) index.entire = 0;
      else index.entire = ((unsigned long) RM2.entire) >> shiftamount;
      break;
    case 2: // arithmetic shift right
      dprintf("shift=10 -> Arithmetic Shift Right\nshiftamount=%d\n", shiftamount);
      if(shiftamount == 0) {
	if (isBitSet(RM2.entire, 31)) index.entire = 0xFFFFFFFF;
	else index.entire = 0;
      } else index.entire = ((signed long) RM2.entire) >> shiftamount;
      break;
    default:
      if(shiftamount == 0) { // RRX
	dprintf("shift=11, shiftamount=0 -> RRX\nshiftamount=%d\n", shiftamount);
	tmp.entire = 0;
	if(flags.C) setBit(tmp.entire, 31);
	index.entire = tmp.entire | (((unsigned long) RM2.entire) >> 1);
      } else { // rotate right
	dprintf("shift=11, shiftamount=%d -> Rotate Right\n", shiftamount);
	index.entire = (RotateRight(shiftamount, RM2)).entire;
      }
    }

    if(u == 1) {
      dprintf("Soma imediato (u == 1)\n");
      ls_address.entire += (RN2.entire + index.entire);
    } else {
      dprintf("Subtrai imediato (u == 0)\n");
      ls_address.entire += (RN2.entire - index.entire);
    }
  }

  else if((p == 1)&&(w == 1)) { // pre-indexed
    dprintf("Caso: pre-indexed with writeback\n");

    // Caso especial: Rn = PC
    if (rn == PC) {
      printf("Resultado do tipo LSR imprevisivel (Can't Writeback to PC, Rn = PC)\n");
      ac_annul();
      return;
    }
    // Caso especial Rn = Rd
    if (rn == rd) {
      printf("Resultado do tipo LSR imprevisivel (Can't Writeback to loaded Register, Rn = Rd)\n");
      ac_annul();
      return;
    }
    // Caso especial Rm = PC
    if (rm == PC) {
      printf("Resultado do tipo LSR imprevisivel (Ilegal usage of PC, Rm = PC)\n");
      ac_annul();
      return;
    }
    // Caso especial Rn = Rm
    if (rn == rm) {
      printf("Resultado do tipo LSR imprevisivel (Can't use same register for Rn and Rm\n");
      ac_annul();
      return;
    }
    
    switch(shift){
    case 0:
      dprintf("shift=00, shiftamount=0 -> Register\n");
      if(shiftamount == 0) { // Register
	index.entire = RM2.entire;
      } else { // Scaled logical shift left
	dprintf("shift=00, shiftamount=%d -> Scalled logical shift left\n", shiftamount);
	index.entire = RM2.entire << shiftamount;
      } 
      break;
    case 1: // logical shift right
      dprintf("shift=01 -> Logical Shift Right\n");
      if(shiftamount == 0) index.entire = 0;
      else index.entire = ((unsigned long) RM2.entire) >> shiftamount;
      break;
    case 2: // arithmetic shift right
      dprintf("shift=10 -> Arithmetic Shift Right\n");
      if(shiftamount == 0) {
	if (isBitSet(RM2.entire,31)) 
	  index.entire = 0xFFFFFFFF;
	else 
	  index.entire = 0;
      } else index.entire = ((signed long) RM2.entire) >> shiftamount;
      break;
    default:
      if(shiftamount == 0) { // RRX
	dprintf("shift=11, shiftamount=0 -> RRX\n");
	tmp.entire = 0;
	if (flags.C) setBit(tmp.entire,31);
	index.entire = tmp.entire | (((unsigned long) RM2.entire) >> 1);
      } else { // rotate right
	dprintf("shift=11, shiftamount=%d -> Rotate Right\n", shiftamount);
	index.entire = (RotateRight(shiftamount, RM2)).entire;
      }
    }

    if(u == 1) {
      dprintf("Soma imediato (u == 1)\n");
      ls_address.entire = RN2.entire + index.entire;
    } else {
      dprintf("Subtrai imediato (u == 0)\n");
      ls_address.entire = RN2.entire - index.entire;    
    }

    RB.write(rn, ls_address.entire);
  }

  else if((p == 0)&&(w == 0)) { // post-indexed
    dprintf("Caso: post-indexed\n");

    // Caso especial: Rn = PC
    if (rn == PC) {
      printf("Resultado do tipo LSR imprevisivel (Can't Writeback to PC, Rn = PC)\n");
      ac_annul();
      return;
    }
    // Caso especial Rn = Rd
    if (rn == rd) {
      printf("Resultado do tipo LSR imprevisivel (Can't Writeback to loaded Register, Rn = Rd)\n");
      ac_annul();
      return;
    }
    // Caso especial Rm = PC
    if (rm == PC) {
      printf("Resultado do tipo LSR imprevisivel (Ilegal usage of PC, Rm = PC)\n");
      ac_annul();
      return;
    }
    // Caso especial Rn = Rm
    if (rn == rm) {
      printf("Resultado do tipo LSR imprevisivel (Can't use same register for Rn and Rm\n");
      ac_annul();
      return;
    }
    
    ls_address.entire = RN2.entire;

    switch(shift) {
    case 0:
      if(shiftamount == 0) { // Register
	dprintf("shift=00, shiftamount=0 -> Register\n");
	index.entire = RM2.entire;
      } else { // Scaled logical shift left
	dprintf("shift=00, shiftamount=%d -> Scalled logical shift left\n", shiftamount);
	index.entire = RM2.entire << shiftamount;
      }
      break;
    case 1: // logical shift right
      dprintf("shift=01 -> Logical Shift Right\n");
      if(shiftamount == 0) index.entire = 0;
      else index.entire = ((unsigned long) RM2.entire) >> shiftamount;
      break;
    case 2: // arithmetic shift right
      dprintf("shift=10 -> Arithmetic Shift Right\n");
      if(shiftamount == 0) {
	if (isBitSet(RM2.entire, 31)) 
	  index.entire = 0xFFFFFFFF;
	else 
	  index.entire = 0;
      } else index.entire = ((signed long) RM2.entire) >> shiftamount;
      break;
    default:
      if(shiftamount == 0) { // RRX
	dprintf("shift=11, shiftamount=0 -> RRX\n");
	tmp.entire = 0;
	if(flags.C) setBit(tmp.entire, 31);
	index.entire = tmp.entire | (((unsigned long) RM2.entire) >> 1);	
      } else { // rotate right
	dprintf("shift=11, shiftamount=%d -> Rotate Right\n", shiftamount);
	index.entire = (RotateRight(shiftamount, RM2)).entire;
      }
    }

    if(u == 1) {
      dprintf("Soma imediato (u == 1)\n");
      RB.write(rn, RN2.entire + index.entire);
    } else {
      dprintf("Subtrai imediato (u == 0)\n");
      RB.write(rn, RN2.entire - index.entire);
    }
  } 
  dprintf("Endereco calculado: %ld\nrn=%d contendo %ld\n", ls_address.entire,rn,RN2.entire);
}

// LSE - Load Store HalfWord
void ac_behavior( Type_LSE ){

  long off8;
  reg_t RM2, RN2;

  dprintf("Tipo de instrucao: LSE\n");

  // Casos especiais
  if((p == 0)&&(w == 1)) {
    printf("Resultado do tipo LSE imprevisivel");
    ac_annul();
    return;
  }
  if((ss == 0)&&(hh == 0)) {
    printf("Erro de decodificacao: essa instrucao nao eh LSE");
    ac_annul();
    return;
  }
  if((ss == 1)&&(l == 0)) 
    dprintf("Especial DSP\n");
    // FIXME: Testar o segundo Registrador de LDRD e STRD nos casos de Writeback

  RN2.entire = RB.read(rn);
  dprintf("rn=%d, contendo %ld\n", rn,RN2.entire);

  // nos LSE's que usam registrador, o campo addr2 armazena Rm
  RM2.entire = RB.read(addr2);
  off8 = ((unsigned long)(addr1 << 4) | addr2);
  ls_address.entire = 0;

  if(p == 1) { // offset ou pre-indexed
    if((i == 1)&&(w == 0)) { // immediate offset
      dprintf("Caso: Immediate offset without writeback\n");

      if(rn == PC) 
	ls_address.entire = 4;

      dprintf("addr1=%d, addr2=%d, off8=%d\n", addr1,addr2,off8);

      if(u == 1) {
	dprintf("Soma imediato (u == 1)\n");
	ls_address.entire += (RN2.entire + off8);
      } else {
	dprintf("Subtrai imediato (u == 0)\n");
	ls_address.entire += (RN2.entire - off8);
      }
    }

    else if((i == 0)&&(w == 0)) { // register offset
      dprintf("Caso: Register offset without writeback\n");
      dprintf("rm=%d, contendo %ld\n", addr2,RM2.entire);
      
      // Caso especial Rm = PC
      if (addr2 == PC) {
	printf("Resultado do tipo LSE imprevisivel (Ilegal usage of PC, Rm = PC)\n");
	ac_annul();
	return;
      }

      if(rn == PC) 
	ls_address.entire = 4;

      if(u == 1) {
	dprintf("Soma imediato (u == 1)\n");
	ls_address.entire += (RN2.entire + RM2.entire);
      } else  {
	dprintf("Subtrai imediato (u == 0)\n");
	ls_address.entire += (RN2.entire - RM2.entire);
      }
    }
    
    else if ((i == 1)&&(w == 1)) { // immediate pre-indexed
      dprintf("Caso: Immediate pre-indexed with writeback\n");

      // Caso especial: Rn = PC
      if (rn == PC) {
	printf("Resultado do tipo LSE imprevisivel (Can't Writeback to PC, Rn = PC)\n");
	ac_annul();
	return;
      }
      // Caso especial Rn = Rd
      if (rn == rd) {
	printf("Resultado do tipo LSE imprevisivel (Can't Writeback to loaded Register, Rn = Rd)\n");
	ac_annul();
	return;
      }
      
      if(u == 1) {
	dprintf("Soma imediato (u == 1)\n");
	ls_address.entire = (RN2.entire + off8);
      } else {
	dprintf("Subtrai imediato (u == 0)\n");
	ls_address.entire = (RN2.entire - off8);
      }

      RB.write(rn, ls_address.entire);
    }
    
    else { // i == 0 && w == 1: register pre-indexed
      dprintf("Caso: Register pre-indexed with writeback\n");
      dprintf("rm=%d, contendo %ld\n", addr2,RM2.entire);

      // Caso especial: Rn = PC
      if (rn == PC) {
	printf("Resultado do tipo LSE imprevisivel (Can't Writeback to PC, Rn = PC)\n");
	ac_annul();
	return;
      }
      // Caso especial Rn = Rd
      if (rn == rd) {
	printf("Resultado do tipo LSE imprevisivel (Can't Writeback to loaded Register, Rn = Rd)\n");
	ac_annul();
	return;
      }
      // Caso especial Rm = PC
      if (addr2 == PC) {
	printf("Resultado do tipo LSE imprevisivel (Ilegal usage of PC, Rm = PC)\n");
	ac_annul();
	return;
      }
      // Caso especial Rn = Rm
      if (rn == addr2) {
	printf("Resultado do tipo LSE imprevisivel (Can't use same register for Rn and Rm\n");
	ac_annul();
	return;
      }
      
      if(u == 1) {
	dprintf("Soma imediato (u == 1)\n");
	ls_address.entire = (RN2.entire + RM2.entire);
      } else {
	dprintf("Subtrai imediato (u == 0)\n");
	ls_address.entire = (RN2.entire - RM2.entire);
      }

      RB.write(rn, ls_address.entire);
    }

  } else { // p == 0: post-indexed
    if((i == 1)&&(w == 0)) { // immediate post-indexed
      dprintf("Caso: Immediate post-indexed\n");

      if(rn == PC) {
	printf("Resultado do tipo LSE imprevisivel");
	ac_annul();
	return;
      }

      ls_address.entire = RN2.entire;
      if(u == 1) {
	dprintf("Soma imediato (u == 1)\n");
	RB.write(rn, RN2.entire + off8);
      } else {
	dprintf("Subtrai imediato (u == 0)\n");
	RB.write(rn, RN2.entire - off8);
      }
    }
    else if((i == 0)&&(w == 0)) { // register post-indexed
      dprintf("Caso: Register post-indexed\n");
      dprintf("rm=%d, contendo %ld\n", addr2,RM2.entire);

      // Caso especial: Rn = PC
      if (rn == PC) {
	printf("Resultado do tipo LSE imprevisivel (Can't Writeback to PC, Rn = PC)\n");
	ac_annul();
	return;
      }
      // Caso especial Rn = Rd
      if (rn == rd) {
	printf("Resultado do tipo LSE imprevisivel (Can't Writeback to loaded Register, Rn = Rd)\n");
	ac_annul();
	return;
      }
      // Caso especial Rm = PC
      if (addr2 == PC) {
	printf("Resultado do tipo LSE imprevisivel (Ilegal usage of PC, Rm = PC)\n");
	ac_annul();
	return;
      }
      // Caso especial Rn = Rm
      if (rn == addr2) {
	printf("Resultado do tipo LSE imprevisivel (Can't use same register for Rn and Rm\n");
	ac_annul();
	return;
      }
      
      ls_address.entire = RN2.entire;
      if(u == 1) {
	dprintf("Soma imediato (u == 1)\n");
	RB.write(rn, RN2.entire + RM2.entire);
      } else {
	dprintf("Subtrai imediato (u == 0)\n");
	RB.write(rn, RN2.entire - RM2.entire);
      }
    }
  }

  dprintf("Endereco a ser utilizado: %ld\n", ls_address.entire);
}

// LSM - Load Store Multiple
void ac_behavior( Type_LSM ){

  reg_t RN2;
  int setbits;

  dprintf("Tipo da instrucao: LSM\n");

  // coloca a lista de registradores numa variavel que acessa bits
  reg_t registerList;
  dprintf("Lista de registradores: %d\n", rlist);
  registerList.entire = (unsigned long) rlist;

  // Caso especial Lista Vazia
  if (registerList.entire == 0) {
    printf("Resultado do tipo LSM imprevisivel (None Register specified)\n");
    ac_annul();
    return;
  }
  
  RN2.entire = RB.read(rn);
  setbits = LSM_CountSetBits(registerList);
  dprintf("Numero de bits setados: %d\n", setbits);

  if((p == 0)&&(u == 1)) { // increment after
    dprintf("Modo de operacao: IA\n");
    lsm_startaddress.entire = RN2.entire;
    lsm_endaddress.entire = RN2.entire + (setbits * 4) - 4;
    if(w == 1) RN2.entire += (setbits * 4);  
  }
  else if((p == 1)&&(u == 1)) { // increment before
    dprintf("Modo de operacao: IB\n");
    lsm_startaddress.entire = RN2.entire + 4; 
    lsm_endaddress.entire = RN2.entire + (setbits * 4);
    if(w == 1) RN2.entire += (setbits * 4);
  }
  else if((p == 0)&&(u == 0)) { // decrement after
    dprintf("Modo de operacao: DA\n");
    lsm_startaddress.entire = RN2.entire - (setbits * 4) + 4;
    lsm_endaddress.entire = RN2.entire;
    if(w == 1) RN2.entire -= (setbits * 4);
  }
  else { // decrement before
    dprintf("Modo de operacao: DB\n");
    lsm_startaddress.entire = RN2.entire - (setbits * 4);
    lsm_endaddress.entire = RN2.entire - 4;
    if(w == 1) RN2.entire -= (setbits * 4);
  }

  // Caso especial Rn in Rlist
  if((w == 1)&&(isBitSet(rlist,rn))) {
    printf("Resultado do tipo LSM imprevisivel (Can't Writeback to loaded Register, Rn in Rlist)\n");
    ac_annul();
    return;
  }

  dprintf("Enderecos de memoria:\nInicial = %ld\nFinal = %ld\n", lsm_startaddress.entire,lsm_endaddress.entire);

  RB.write(rn,RN2.entire);
}

void ac_behavior( Type_CDP ){
  printf("Tipo da instrucao: CDP\n");
  // nada a ser implementado
}
void ac_behavior( Type_CRT ){
  printf("Tipo da instrucao: CRT\n");
  // nada a ser implementado
}
void ac_behavior( Type_CLS ){
  printf("Tipo da instrucao: CLS\n");
  // nada a ser implementado
}
void ac_behavior( Type_MBKPT ){
  printf("Tipo da instrucao: MBKPT\n");
  // nada a ser implementado
}
void ac_behavior( Type_MSWI ){
  printf("Tipo da instrucao: MSWI\n");
  // nada a ser implementado
}
void ac_behavior( Type_MCLZ ){
  printf("Tipo da instrucao: MCLZ\n");
  // nada a ser implementado
}
void ac_behavior( Type_MMSR1 ){
  printf("Tipo da instrucao: MMSR1\n");
  // nada a ser implementado
}
void ac_behavior( Type_MMSR2 ){
  printf("Tipo da instrucao: MMSR2\n");
  // nada a ser implementado
}

void ac_behavior( Type_DSPSM ){

  reg_t RM2, RS2;
  
  RM2.entire = RB.read(rm);
  RS2.entire = RB.read(rs);
  
  dprintf("Tipo da instrucao: DSPSM\n");
  dprintf("rs=%d, contendo %ld\nrm=%d, contendo %ld\nyy=%d\nxx=%d", rs, RS2.entire, rm, RM2.entire, yy, xx);

  // Casos especias
  if((drd == PC)||(drn == PC)||(rm == PC)||(rs == PC)) {
    printf("Resultado da operacao SMLA<y><x> imprevisivel\n");
    return;  
  }
  
  if(xx == 0)
    OP1.entire = SignExtend(RM2.entire, 16);
  else
    OP1.entire = SignExtend((RM2.entire >> 16), 16);
  
  if(yy == 0)
    OP2.entire = SignExtend(RS2.entire, 16);
  else
    OP2.entire = SignExtend((RS2.entire >> 16), 16);
}


//! Behavior Methods

//------------------------------------------------------
void ADC(int rd, int rn, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2;
  r64bit_t soma;

  dprintf("Instrucao: ADC\n");
  RN2.entire = RB.read(rn);
  if(rn == PC) RN2.entire += 4;
  dprintf("Operadores:\nRn=%d, contendo %ld\nShiftOp contendo %ld\nC=%d\nDestino: Rd=%d\n", rn,RN2.entire,dpi_shiftop.entire,flags.C,rd);
  soma.hilo = ((unsigned long long)(unsigned long)RN2.entire + (unsigned long long)(unsigned long)dpi_shiftop.entire);
  if (flags.C) soma.hilo++;
  RD2.entire = soma.reg[0];
  RB.write(rd, RD2.entire);
  if ((s == 1)&&(rd == PC)) {
    printf("Resultado da operacao ADC imprevisivel\n");
    return;
  } else {
    if (s == 1) {
      flags.N = getBit(RD2.entire,31);
      flags.Z = ((RD2.entire == 0) ? true : false);
      flags.C = ((soma.reg[1] != 0) ? true : false);
      flags.V = (((getBit(RN2.entire,31) && getBit(dpi_shiftop.entire,31) && (!getBit(RD2.entire,31))) ||
		  ((!getBit(RN2.entire,31)) && (!getBit(dpi_shiftop.entire,31)) && getBit(RD2.entire,31))) ? true : false);
    }
  }
  dprintf("Resultado: %ld\nFlags: N=%d, Z=%d, C=%d, V=%d\n", RD2.entire,flags.N,flags.Z,flags.C,flags.V);
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void ADD(int rd, int rn, bool s,
    ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
    ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2;
  r64bit_t soma;

  dprintf("Instrucao: ADD\n");
  RN2.entire = RB.read(rn);
  if(rn == PC) RN2.entire += 4;
  dprintf("Operadores:\nRn=%d, contendo %ld\nShiftOp contendo %ld\nDestino: Rd=%d\n", rn,RN2.entire,dpi_shiftop.entire,rd);
  soma.hilo = ((unsigned long long)(unsigned long)RN2.entire + (unsigned long long)(unsigned long)dpi_shiftop.entire);
  RD2.entire = soma.reg[0];
  RB.write(rd, RD2.entire);
  if ((s == 1)&&(rd == PC)) {
    printf("Resultado da operacao ADD imprevisivel\n");
    return;
  } else {
    if (s == 1) {
      flags.N = getBit(RD2.entire,31);
      flags.Z = ((RD2.entire == 0) ? true : false);
      flags.C = ((soma.reg[1] != 0) ? true : false);
      flags.V = (((getBit(RN2.entire,31) && getBit(dpi_shiftop.entire,31) && (!getBit(RD2.entire,31))) ||
		  ((!getBit(RN2.entire,31)) && (!getBit(dpi_shiftop.entire,31)) && getBit(RD2.entire,31))) ? true : false);
    }
  }
  dprintf("Resultado: %ld\nFlags: N=%d, Z=%d, C=%d, V=%d\n", RD2.entire,flags.N,flags.Z,flags.C,flags.V);
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void AND(int rd, int rn, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2;
  
  dprintf("Instrucao: AND\n");
  RN2.entire = RB.read(rn);
  dprintf("Operadores:\nRn=%d, contendo %ld\nShiftOp contendo %ld\nDestino: Rd=%d\n", rn,RN2.entire,dpi_shiftop.entire,rd);
  RD2.entire = RN2.entire & dpi_shiftop.entire;
  RB.write(rd, RD2.entire);

  if ((s == 1)&&(rd == PC)) {
    printf("Resultado da operacao AND imprevisivel\n");
    return;
  } else {
    if (s == 1) {
      flags.N = getBit(RD2.entire, 31);
      flags.Z = ((RD2.entire == 0) ? true : false);
      flags.C = dpi_shiftopcarry;
      // nada acontece com flags.V 
    }
  }   
  dprintf("Resultado: %ld\nFlags: N=%d, Z=%d, C=%d, V=%d\n", RD2.entire,flags.N,flags.Z,flags.C,flags.V);
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void B(int h, int offset,
       ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
       ac_reg<unsigned>& ac_pc) {

  long long mem_pos;
  long s_extend;

  // Lembrar que PC ja esta somado com 4, ou seja, ja esta na proxima instrucao

  if(h == 1) { // h?? leia-se "l"
    dprintf("Instrucao: BL\n");
    RB.write(LR, RB.read(PC));
    dprintf("Endereco de retorno do BL: %ld\n", RB.read(LR));
  } else dprintf("Instrucao: B\n");

  dprintf("offset=%ld\n",offset);
  s_extend = SignExtend((long) (offset << 2),26);
  dprintf("s_extend=%ld\n",s_extend);

  mem_pos = (long long) RB.read(PC) + 4 + s_extend;
  dprintf("Destino do branch calculado: %lld\n", mem_pos);
  if((mem_pos < 0)) {
    printf("Destino do branch fora dos limites de memoria\n");
    return;
  } else RB.write(PC,(long) mem_pos);

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void BX(int rm,
        ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
        ac_reg<unsigned>& ac_pc) {

  dprintf("Instrucao: BX\n");

  // Faz a verificao da entranda ou nao no modo Thumb
  if(isBitSet(rm,0)) {
    dprintf("Change to Thumb nao implementado\n");
    return;
  } else 
    dprintf("Instrucao normal ARM\n");

  flags.T = isBitSet(rm, 0);
  ac_pc = RB.read(rm) & 0xFFFFFFFE;

  //dprintf("Pc = %X",ac_pc);
}

//------------------------------------------------------
void BIC(int rd, int rn, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2;
  
  dprintf("Instrucao: BIC\n");
  RN2.entire = RB.read(rn);
  RD2.entire = RN2.entire & ~dpi_shiftop.entire;
  dprintf("Operadores:\nRn=%d, contendo %ld\nShiftOp contendo %ld\nDestino: Rd=%d\n", rn,RN2.entire,dpi_shiftop.entire,rd);
  RB.write(rd,RD2.entire);

  if ((s == 1)&&(rd == PC)) {
    printf("Resultado da operacao BIC imprevisivel\n");
    return;
  } else {
    if (s == 1) {
      flags.N = getBit(RD2.entire,31);
      flags.Z = ((RD2.entire == 0) ? true : false);
      flags.C = dpi_shiftopcarry;
      // nada acontece com flags.V 
    }
  }   
  dprintf("Resultado: %ld\nFlags: N=%d, Z=%d, C=%d, V=%d\n", RD2.entire,flags.N,flags.Z,flags.C,flags.V);
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void CDP(){
  dprintf("Instrucao: CDP\n");
  printf("Instrucao CDP nao implementada");
}

//------------------------------------------------------
void CLZ(int rd, int rm,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RM2;
  int i;

  dprintf("Instrucao: CLZ\n");

  // Casos especiais
  if((rd == PC)||(rm == PC)) {
    printf("Resultado da operacao CLZ imprevisivel\n");
    return;
  }

  RM2.entire = RB.read(rm);

  if(RM2.entire == 0) RD2.entire = 32;
  else {
    i = 31;
    while((i>=0)&&(!isBitSet(RM2.entire,i))) i--;
    RD2.entire = 31 - i;
  }

  dprintf("Resultado: %ld\n", RD2.entire);
    
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void CMN(int rn,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RN2, alu_out;
  r64bit_t soma;

  dprintf("Instrucao: CMN\n");
  RN2.entire = RB.read(rn);
  dprintf("Operadores:\nRn=%d, contendo %ld\nShiftOp contendo %ld\n", rn,RN2.entire,dpi_shiftop.entire);
  soma.hilo = ((unsigned long long)(unsigned long)RN2.entire + (unsigned long long)(unsigned long)dpi_shiftop.entire);
  alu_out.entire = soma.reg[0];

  flags.N = getBit(alu_out.entire,31);
  flags.Z = ((alu_out.entire == 0) ? true : false);
  flags.C = ((soma.reg[1] != 0) ? true : false);
  flags.V = (((getBit(RN2.entire,31) && getBit(dpi_shiftop.entire,31) && (!getBit(alu_out.entire,31))) ||
	      ((!getBit(RN2.entire,31)) && (!getBit(dpi_shiftop.entire,31)) && getBit(alu_out.entire,31))) ? true : false);

  dprintf("Resultado: %ld\nFlags: N=%d, Z=%d, C=%d, V=%d\n", alu_out.entire,flags.N,flags.Z,flags.C,flags.V);    
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void CMP(int rn,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RN2, alu_out, neg_shiftop;
  r64bit_t result;

  dprintf("Instrucao: CMP\n");
  RN2.entire = RB.read(rn);
  dprintf("Operadores:\nRn=%d, contendo %ld\nShiftOp contendo %ld\n", rn,RN2.entire,dpi_shiftop.entire);
  neg_shiftop.entire = - dpi_shiftop.entire;
  result.hilo = ((unsigned long long)(unsigned long)RN2.entire + (unsigned long long)(unsigned long) neg_shiftop.entire);
  alu_out.entire = result.reg[0];

  flags.N = getBit(alu_out.entire,31);
  flags.Z = ((alu_out.entire == 0) ? true : false);
  flags.C = !(((unsigned int) dpi_shiftop.entire > (unsigned int) RN2.entire) ? true : false);
  flags.V = (((getBit(RN2.entire,31) && getBit(neg_shiftop.entire,31) && (!getBit(alu_out.entire,31))) ||
	      ((!getBit(RN2.entire,31)) && (!getBit(neg_shiftop.entire,31)) && getBit(alu_out.entire,31))) ? true : false);
  
  dprintf("Resultado: %ld\nFlags: N=%d, Z=%d, C=%d, V=%d\n", alu_out.entire,flags.N,flags.Z,flags.C,flags.V);     
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void EOR(int rd, int rn, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2;
  
  dprintf("Instrucao: EOR\n");
  RN2.entire = RB.read(rn);
  dprintf("Operadores:\nRn=%d, contendo %ld\nShiftOp contendo %ld\nDestino: Rd=%d\n", rn,RN2.entire,dpi_shiftop.entire,rd);
  RD2.entire = RN2.entire ^ dpi_shiftop.entire;
  RB.write(rd, RD2.entire);

  if ((s == 1)&&(rd == PC)) {
    printf("Resultado da operacao EOR imprevisivel\n");
    return;
  } else {
    if (s == 1) {
      flags.N = getBit(RD2.entire, 31);
      flags.Z = ((RD2.entire == 0) ? true : false);
      flags.C = dpi_shiftopcarry;
      // nada acontece com flags.V 
    }
  }   
  dprintf("Resultado: %ld\nFlags: N=%d, Z=%d, C=%d, V=%d\n", RD2.entire,flags.N,flags.Z,flags.C,flags.V); 
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void LDC(){
  dprintf("Instrucao: LDC\n");
  printf("Instrucao LDC nao implementada");
}

//------------------------------------------------------
void LDM(int rlist, bool r,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  // fazer casos especiais

  int i;
  long value;

  dprintf("Instrucao: LDM\n");
  ls_address = lsm_startaddress;
  dprintf("Endereco inicial: %ld\n",ls_address.entire);
  for(i=0;i<15;i++){
    if(isBitSet(rlist,i)) {
      RB.write(i,MEM.read(ls_address.entire));
      ls_address.entire += 4;
      dprintf("Registrador carregado: %d; Valor: %d; proximo endereco: %ld\n", i,RB.read(i),ls_address.entire);
    }
  }
    
  if((r == 0)&&(isBitSet(rlist,PC))) { // LDM(1)
    value = MEM.read(ls_address.entire);
    RB.write(PC,value & 0xFFFFFFFE);
    ls_address.entire += 4;
    dprintf("Registrador carregado: PC; proximo endereco: %ld\n", ls_address.entire);
  }
    
  // LDM(2) igual a LDM(1) exceto pelo "if" acima
  // LDM(3) imprevisivel no User Mode
    
  //assert lsm_endaddress.entire == ls_address.entire - 4; // perguntar pro Rodolfo

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void LDR(int rd, int rn,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  long value;
  reg_t tmp;
  int addr10;

  dprintf("Instrucao: LDR\n");
  addr10 = (unsigned int) ls_address.entire & 0x00000003;

  // Casos Especiais
  // verificar caso do coprocessador (alinhamento)
      
  switch(addr10) {
  case 0:
    dprintf("Caso de enderecamento(addr10): %d -> 1o byte\n",addr10);
    value = MEM.read(ls_address.entire);
    break;
  case 1:
    dprintf("Caso de enderecamento(addr10): %d -> 2o byte\n",addr10);
    tmp.entire = MEM.read(ls_address.entire);
    value = (RotateRight(8,tmp)).entire;
    break;
  case 2:
    dprintf("Caso de enderecamento(addr10): %d -> 3o byte\n",addr10);
    tmp.entire = MEM.read(ls_address.entire);
    value = (RotateRight(16,tmp)).entire;
    break;
  default:
    dprintf("Caso de enderecamento(addr10): %d -> 4o byte\n",addr10);
    tmp.entire = MEM.read(ls_address.entire);
    value = (RotateRight(24,tmp)).entire;
  }
    
  dprintf("Valor calculado: %ld\n",value);
  if(rd == PC) {
    RB.write(PC,(value & 0xFFFFFFFE));
    flags.T = isBitSet(value,0);
  }
  else 
    RB.write(rd,value);

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void LDRB(int rd, int rn,
          ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
          ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  dprintf("Instrucao: LDRB\n");

  // Casos Especiais
  
  dprintf("Byte lido: %d\n", MEM.read_byte(ls_address.entire));
  RB.write(rd, ((unsigned long)MEM.read_byte(ls_address.entire)));

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void LDRBT(int rd, int rn,
           ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
           ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  dprintf("Instrucao: LDRBT\n");

  // Casos Especiais
  
  dprintf("Byte lido: %d\n",MEM.read_byte(ls_address.entire));
  RB.write(rd, ((unsigned long)MEM.read_byte(ls_address.entire)));

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void LDRD(int rd, int rn,
          ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
          ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  dprintf("Instrucao: LDRD\n");

  // Casos Especiais
  // Registrador destino deve ser par
  if(isBitSet(rd,0)){
    printf("Resultado da operacao LDRD indefinido (Rd must be even)\n");
    return;
  }
  // Verificar alinhamento do doubleword
  if((rd == LR)||(ls_address.entire & 0x00000007)){
    printf("Resultado da operacao LDRD imprevisivel (Address is not Doubleword Aligned)\n");
    return;
  }

  //FIXME: Verificar se o writeback recebe o +4 do endereco
  RB.write(rd,MEM.read(ls_address.entire));
  RB.write(rd+1,MEM.read(ls_address.entire+4));

  ac_pc = RB.read(PC);
}
//------------------------------------------------------
void LDRH(int rd, int rn,
          ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
          ac_reg<unsigned>& ac_pc, ac_memory& MEM) {
 
  dprintf("Instrucao: LDRH\n");

  // Casos Especiais
  // verificar caso do coprocessador (alinhamento)
  // verificar alinhamento do halfword
  if(isBitSet(ls_address.entire,0)){
    printf("Resultado da operacao LDRH imprevisivel (Address is not Halfword Aligned)\n");
    return;
  }

  RB.write(rd, ((unsigned long)MEM.read_half(ls_address.entire)));

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void LDRSB(int rd, int rn,
           ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
           ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  long data;

  dprintf("Instrucao: LDRSB\n");
    
  // Casos Especiais
  
  data = ((unsigned long)MEM.read_byte(ls_address.entire));
  RB.write(rd, SignExtend(data,8));
 
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void LDRSH(int rd, int rn,
           ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
           ac_reg<unsigned>& ac_pc, ac_memory& MEM){

  long data;

  dprintf("Instrucao: LDRSH\n");
    
  // Casos Especiais
  // verificar alinhamento do halfword
  if(isBitSet(ls_address.entire, 0)) {
    printf("Resultado da operacao LDRSH imprevisivel (Address is not Halfword Aligned)\n");
    return;
  }
  // verificar caso do coprocessador (alinhamento)

  data = ((unsigned long) MEM.read_half(ls_address.entire));
  RB.write(rd, SignExtend(data,16));
    
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void LDRT(int rd, int rn,
          ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
          ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  int addr10;
  reg_t tmp;

  dprintf("Instrucao: LDRT\n");

  addr10 = (int) ls_address.entire & 0x00000003;
    
  // Casos Especiais
  // verificar caso do coprocessador (alinhamento)
    
  switch(addr10) {
  case 0:
    RB.write(rd,MEM.read(ls_address.entire));
    break;
  case 1:
    tmp.entire = MEM.read(ls_address.entire);
    RB.write(rd,(RotateRight(8,tmp)).entire);
    break;
  case 2:
    tmp.entire = MEM.read(ls_address.entire);
    RB.write(rd,(RotateRight(16,tmp)).entire);
    break;
  default:
    tmp.entire = MEM.read(ls_address.entire);
    RB.write(rd,(RotateRight(24,tmp)).entire);
  }

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void MCR(){
  dprintf("Instrucao: MCR\n");
  printf("Instrucao MCR nao implementada\n");
}

//------------------------------------------------------
void MLA(int rd, int rn, int rm, int rs, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2, RM2, RS2;
  RN2.entire = RB.read(rn);
  RM2.entire = RB.read(rm);
  RS2.entire = RB.read(rs);

  dprintf("Instrucao: MLA\n");
  dprintf("Operadores:\nrm=%d, contendo %ld\nrs=%d, contendo %ld\nrn=%d, contendo %ld\nDestino: Rd=%d\n", rm,RM2.entire,rs,RS2.entire,rn,RN2.entire,rd);

  // Casos especiais
  if((rd == PC)||(rm == PC)||(rs == PC)||(rn == PC)||(rd == rm)) {
    printf("Resultado da operacao MLA imprevisivel\n");
    return;    
  }

  RD2.entire = (long)((RM2.entire * RS2.entire) + RN2.entire);
  if(s == 1) {
    flags.N = getBit(RD2.entire,31);
    flags.Z = ((RD2.entire == 0) ? true : false);
    // nada acontece com flags.C e flags.V
  }
  dprintf("Flags: N=%d, Z=%d, C=%d, V=%d\n",flags.N,flags.Z,flags.C,flags.V);
  RB.write(rd,RD2.entire);
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void MOV(int rd, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {
  
  dprintf("Instrucao: MOV\n");
  dprintf("Operadores:\nShiftOp contendo %ld\nDestino: Rd=%d\n",dpi_shiftop.entire,rd);
  RB.write(rd, dpi_shiftop.entire);

  if ((s == 1)&&(rd == PC)) {
    printf("Resultado da operacao MOV imprevisivel\n");
    return;
  } else {
    if (s == 1) {
      flags.N = getBit(dpi_shiftop.entire, 31);
      flags.Z = ((dpi_shiftop.entire == 0) ? true : false);
      flags.C = dpi_shiftopcarry;
      // nada acontece com flags.V 
    }
  }   
  dprintf("Flags: N=%d, Z=%d, C=%d, V=%d\n",flags.N,flags.Z,flags.C,flags.V);
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void MRC(){
  dprintf("Instrucao: MRC\n");
  printf("Instrucao MCR nao implementada\n");
}

//------------------------------------------------------
void MRS(int rd, bool r, int zero3, int subop2, int func2, int subop1, int rm, int field,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t CPSR;

  dprintf("Instrucao: MRS\n");

  // Casos especiais
  if((rd == PC)||((zero3 != 0)&&(subop2 != 0)&&(func2 != 0)&&(subop1 != 0)&&(rm != 0))||
     (field != 15)||(r == 1)) {
    printf("Resultado da operacao MRS imprevisivel\n");
    return;
  }

  CPSR = CPSRBuild();
  RB.write(rd,CPSR.entire);

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void MUL(int rd, int rm, int rs, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RM2, RS2;
  RM2.entire = RB.read(rm);
  RS2.entire = RB.read(rs);

  dprintf("Instrucao: MUL\n");
  dprintf("Operadores:\nrm=%d, contendo %ld\nrs=%d, contendo %ld\nDestino: Rd=%d\n",rm,RM2.entire,rs,RS2.entire,rd);

  // Casos especiais
  if((rd == PC)||(rm == PC)||(rs == PC)||(rd == rm)) {
    printf("Resultado da operacao MUL imprevisivel\n");
    return;    
  }

  RD2.entire = (long)(RM2.entire * RS2.entire);
  if(s == 1) {
    flags.N = getBit(RD2.entire, 31);
    flags.Z = ((RD2.entire == 0) ? true : false);
    // nada acontece com flags.C e flags.V
  }
  dprintf("Flags: N=%d, Z=%d, C=%d, V=%d\n",flags.N,flags.Z,flags.C,flags.V);
  RB.write(rd, RD2.entire);
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void MVN(int rd, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  dprintf("Instrucao: MVN\n");
  dprintf("Operadores:\nShiftOp contendo %ld\nDestino: Rd=%d\n",dpi_shiftop.entire,rd);
  RB.write(rd,~dpi_shiftop.entire);

  if ((s == 1)&&(rd == PC)) {
    printf("Resultado da operacao MVN imprevisivel\n");
    return;
  } else {
    if (s == 1) {
      flags.N = getBit(~dpi_shiftop.entire,31);
      flags.Z = ((~dpi_shiftop.entire == 0) ? true : false);
      flags.C = dpi_shiftopcarry;
      // nada acontece com flags.V 
    }
  }   
  dprintf("Flags: N=%d, Z=%d, C=%d, V=%d\n",flags.N,flags.Z,flags.C,flags.V);
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void ORR(int rd, int rn, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2;
  
  dprintf("Instrucao: ORR\n");
  RN2.entire = RB.read(rn);
  dprintf("Operadores:\nRn=%d, contendo %ld\nShiftOp contendo %ld\nDestino: Rd=%d\n", rn,RN2.entire,dpi_shiftop.entire,rd);
  RD2.entire = RN2.entire | dpi_shiftop.entire;
  RB.write(rd,RD2.entire);

  if ((s == 1)&&(rd == PC)) {
    printf("Resultado da operacao ORR imprevisivel\n");
    return;
  } else {
    if (s == 1) {
      flags.N = getBit(RD2.entire,31);
      flags.Z = ((RD2.entire == 0) ? true : false);
      flags.C = dpi_shiftopcarry;
      // nada acontece com flags.V 
    }
  }  
  dprintf("Resultado: %ld\nFlags: N=%d, Z=%d, C=%d, V=%d\n",RD2.entire,flags.N,flags.Z,flags.C,flags.V);  
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void RSB(int rd, int rn, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2, neg_RN2;
  r64bit_t result;

  dprintf("Instrucao: RSB\n");
  RN2.entire = RB.read(rn);
  if(rn == PC) RN2.entire += 4;
  dprintf("Operadores:\nRn=%d, contendo %ld\nShiftOp contendo %ld\nDestino: Rd=%d\n", rn,RN2.entire,dpi_shiftop.entire,rd);
  neg_RN2.entire = - RN2.entire;
  result.hilo = ((unsigned long long)(unsigned long)dpi_shiftop.entire + (unsigned long long)(unsigned long)neg_RN2.entire);
  RD2.entire = result.reg[0];
  RB.write(rd, RD2.entire);
  if ((s == 1) && (rd == PC)) {
    printf("Resultado da operacao RSB imprevisivel\n");
    return;
  } else {
    if (s == 1) {
      flags.N = getBit(RD2.entire,31);
      flags.Z = ((RD2.entire == 0) ? true : false);
      flags.C = !(((unsigned int) RN2.entire > (unsigned int) dpi_shiftop.entire) ? true : false);
      flags.V = (((getBit(neg_RN2.entire,31) && getBit(dpi_shiftop.entire,31) && (!getBit(RD2.entire,31))) ||
		  ((!getBit(neg_RN2.entire,31)) && (!getBit(dpi_shiftop.entire,31)) && getBit(RD2.entire,31))) ? true : false);
    }
  }
  dprintf("Resultado: %ld\nFlags: N=%d, Z=%d, C=%d, V=%d\n",RD2.entire,flags.N,flags.Z,flags.C,flags.V);
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void RSC(int rd, int rn, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2, neg_RN2;
  r64bit_t result;

  dprintf("Instrucao: RSC\n");
  RN2.entire = RB.read(rn);
  if(rn == PC) RN2.entire += 4;
  dprintf("Operadores:\nRn=%d, contendo %ld\nShiftOp contendo %ld\nC=%d\nDestino: Rd=%d\n", rn,RN2.entire,dpi_shiftop.entire,flags.C,rd);
  neg_RN2.entire = - RN2.entire;
  if (!flags.C) neg_RN2.entire--;
  result.hilo = ((unsigned long long)(unsigned long)dpi_shiftop.entire + (unsigned long long)(unsigned long)neg_RN2.entire);
  RD2.entire = result.reg[0];

  RB.write(rd, RD2.entire);
  if ((s == 1)&&(rd == PC)) {
    printf("Resultado da operacao RSC imprevisivel\n");
    return;
  } else {
    if (s == 1) {
      flags.N = getBit(RD2.entire,31);
      flags.Z = ((RD2.entire == 0) ? true : false);
      flags.C = !(((unsigned int) RN2.entire > (unsigned int) dpi_shiftop.entire) ? true : false);
      flags.V = (((getBit(neg_RN2.entire,31) && getBit(dpi_shiftop.entire,31) && (!getBit(RD2.entire,31))) ||
		  ((!getBit(neg_RN2.entire,31)) && (!getBit(dpi_shiftop.entire,31)) && getBit(RD2.entire,31))) ? true : false);
    }
  }
  dprintf("Resultado: %ld\nFlags: N=%d, Z=%d, C=%d, V=%d\n",RD2.entire,flags.N,flags.Z,flags.C,flags.V);
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void SBC(int rd, int rn, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2, neg_shiftop;
  r64bit_t result;

  dprintf("Instrucao: SBC\n");
  RN2.entire = RB.read(rn);
  if(rn == PC) RN2.entire += 4;
  dprintf("Operadores:\nRn=%d, contendo %ld\nShiftOp contendo %ld\nC=%d\nDestino: Rd=%d\n", rn,RN2.entire,dpi_shiftop.entire,flags.C,rd);
  neg_shiftop.entire = - dpi_shiftop.entire; 
  if (!flags.C) neg_shiftop.entire--;
  result.hilo = ((unsigned long long)(unsigned long)RN2.entire + (unsigned long long)(unsigned long)neg_shiftop.entire);
  RD2.entire = result.reg[0];
  RB.write(rd, RD2.entire);
  if ((s == 1)&&(rd == PC)) {
    printf("Resultado da operacao SBC imprevisivel\n");
    return;
  } else {
    if (s == 1) {
      flags.N = getBit(RD2.entire,31);
      flags.Z = ((RD2.entire == 0) ? true : false);
      flags.C = !(((unsigned int) dpi_shiftop.entire > (unsigned int) RN2.entire) ? true : false);
      flags.V = (((getBit(RN2.entire,31) && getBit(neg_shiftop.entire,31) && (!getBit(RD2.entire,31))) ||
		  ((!getBit(RN2.entire,31)) && (!getBit(neg_shiftop.entire,31)) && getBit(RD2.entire,31))) ? true : false);
    }
  }
  dprintf("Resultado: %ld\nFlags: N=%d, Z=%d, C=%d, V=%d\n",RD2.entire,flags.N,flags.Z,flags.C,flags.V);
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void SMLAL(int rdhi, int rdlo, int rm, int rs, bool s,
           ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
           ac_reg<unsigned>& ac_pc) {

  r64bit_t result, acc;
  reg_t RM2, RS2;

  RM2.entire = RB.read(rm);
  RS2.entire = RB.read(rs);
  acc.reg[0] = RB.read(rdlo);
  acc.reg[1] = RB.read(rdhi);

  dprintf("Instrucao: SMLAL\n");
  dprintf("Operadores:\nrm=%d, contendo %ld\nrs=%d, contendo %ld\nSomar resultado da multiplicacao com %lld\nDestino(Hi): Rdhi=%d, Rdlo=%d\n", rm,RM2.entire,rs,RS2.entire,acc.hilo,rdhi,rdlo);

  // Casos especiais
  if((rdhi == PC)||(rdlo == PC)||(rm == PC)||(rs == PC)||(rdhi == rdlo)||(rdhi == rm)||(rdlo == rm)) {
    printf("Resultado da operacao SMLAL imprevisivel\n");
    return;  
  }

  result.hilo = ((long long)((long long)RM2.entire * (long long)RS2.entire)) + (long long)acc.hilo;
  RB.write(rdhi,result.reg[1]);
  RB.write(rdlo,result.reg[0]);
  if(s == 1){
    flags.N = getBit(result.reg[1],31);
    flags.Z = ((result.hilo == 0) ? true : false);
    // nada acontece com flags.C e flags.V
  }
  dprintf("Resultado: %lld\nFlags: N=%d, Z=%d, C=%d, V=%d\n",result.hilo,flags.N,flags.Z,flags.C,flags.V);
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void SMULL(int rdhi, int rdlo, int rm, int rs, bool s,
           ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
           ac_reg<unsigned>& ac_pc) {

  r64bit_t result;
  reg_t RM2, RS2;

  RM2.entire = RB.read(rm);
  RS2.entire = RB.read(rs);

  dprintf("Instrucao: SMULL\n");
  dprintf("Operadores:\nrm=%d, contendo %ld\nrs=%d, contendo %ld\nDestino(Hi): Rdhi=%d, Rdlo=%d\n", rm,RM2.entire,rs,RS2.entire,rdhi,rdlo);

  // Casos especiais
  if((rdhi == PC)||(rdlo == PC)||(rm == PC)||(rs == PC)||(rdhi == rdlo)||(rdhi == rm)||(rdlo == rm)) {
    printf("Resultado da operacao SMULL imprevisivel\n");
    return;  
  }

  result.hilo = (long long)((long long)RM2.entire * (long long)RS2.entire);
  RB.write(rdhi,result.reg[1]);
  RB.write(rdlo,result.reg[0]);
  if(s == 1){
    flags.N = getBit(result.reg[1],31);
    flags.Z = ((result.hilo == 0) ? true : false);
    // nada acontece com flags.C e flags.V
  }
  dprintf("Resultado: %lld\nFlags: N=%d, Z=%d, C=%d, V=%d\n",result.hilo,flags.N,flags.Z,flags.C,flags.V);
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void STC(){
  dprintf("Instrucao: STC\n");
  printf("Instrucao STC nao implementada\n");
}

//------------------------------------------------------
void STM(int rlist,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  // fazer casos especiais

  int i;

  dprintf("Instrucao: STM\n");
  ls_address = lsm_startaddress;
  for(i=0;i<16;i++){
    if(isBitSet(rlist,i)) {
      MEM.write(ls_address.entire,RB.read(i));
      ls_address.entire += 4;
      dprintf("Registrador armazenado: %d; valor: %d; proximo endereco: %ld\n",i,RB.read(i),ls_address.entire);
    }
  }

  // STM(2) imprevisivel no User Mode
    
  // assert lsm_endaddress.entire == ls_address.entire - 4; // perguntar pro Rodolfo

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void STR(int rd, int rn,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  dprintf("Instrucao: STR\n");

  // Casos Especiais
  // verificar caso do coprocessador (alinhamento)
  
  dprintf("Endereco a escrever: %ld\nConteudo rd: %ld\n",ls_address.entire,RB.read(rd));
  MEM.write(ls_address.entire, RB.read(rd));

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void STRB(int rd, int rn,
          ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
          ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  reg_t RD2;

  dprintf("Instrucao: STRB\n");

  // Casos Especiais

  RD2.entire = RB.read(rd);
  dprintf("Endereco a escrever: %ld\nConteudo rd: %ld\n",ls_address.entire,RD2.byte[0]);
  MEM.write_byte(ls_address.entire, RD2.byte[0]);

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void STRBT(int rd, int rn,
           ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
           ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  reg_t RD2;

  dprintf("Instrucao: STRBT\n");

  // Casos Especiais
  
  RD2.entire = RB.read(rd);
  dprintf("Endereco a escrever: %ld\nConteudo rd: %ld\n",ls_address.entire,RD2.byte[0]);
  MEM.write_byte(ls_address.entire, RD2.byte[0]);

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void STRD(int rd, int rn,
          ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
          ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  dprintf("Instrucao: STRD\n");

  // Casos Especiais
  // Registrador destino deve ser par
  if(isBitSet(rd,0)){
    printf("Resultado da operacao STRD indefinido (Rd must be even)\n");
    return;
  }
  // verificar alinhamento do doubleword
  if((rd == LR)||(ls_address.entire & 0x00000007)){
    printf("Resultado da operacao STRD imprevisivel (Address is not Doubleword Aligned)\n");
    return;
  }

  //FIXME: Descobrir se o writeback recebe o +4 do segundo endereco
  MEM.write(ls_address.entire,RB.read(rd));
  MEM.write(ls_address.entire+4,RB.read(rd+1));

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void STRH(int rd, int rn,
          ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
          ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  short int data;

  dprintf("Instrucao: STRH\n");
    
  // Casos Especiais
  // verificar caso do coprocessador (alinhamento)
  // verificar alinhamento do halfword
  if(isBitSet(ls_address.entire,0)){
    printf("Resultado da operacao STRH imprevisivel (Address is not Halfword Aligned)\n");
    return;
  }

  data = (short int) RB.read(rd) & 0x0000FFFF;
  MEM.write_half(ls_address.entire, (short int)data);
    
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void STRT(int rd, int rn,
          ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
          ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  dprintf("Instrucao: STRT\n");

  // Casos Especiais
  // verificar caso do coprocessador (alinhamento)
  
  MEM.write(ls_address.entire, RB.read(rd));

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void SUB(int rd, int rn, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2, neg_shiftop;
  r64bit_t result;

  dprintf("Instrucao: SUB\n");
  RN2.entire = RB.read(rn);
  if(rn == PC) RN2.entire += 4;
  dprintf("Operadores:\nRn=%d, contendo %ld\nShiftOp contendo %ld\nDestino: Rd=%d\n", rn,RN2.entire,dpi_shiftop.entire,rd);
  neg_shiftop.entire = - dpi_shiftop.entire;
  result.hilo = ((unsigned long long)(unsigned long)RN2.entire + (unsigned long long)(unsigned long)neg_shiftop.entire);
  RD2.entire = result.reg[0];
  RB.write(rd, RD2.entire);
  if ((s == 1)&&(rd == PC)) {
    printf("Resultado da operacao SUB imprevisivel\n");
    return;
  } else {
    if (s == 1) {
      flags.N = getBit(RD2.entire,31);
      flags.Z = ((RD2.entire == 0) ? true : false);
      flags.C = !(((unsigned int) dpi_shiftop.entire > (unsigned int) RN2.entire) ? true : false);
      flags.V = (((getBit(RN2.entire,31) && getBit(neg_shiftop.entire,31) && (!getBit(RD2.entire,31))) ||
		  ((!getBit(RN2.entire,31)) && (!getBit(neg_shiftop.entire,31)) && getBit(RD2.entire,31))) ? true : false);
    }
  }
  dprintf("Resultado: %ld\nFlags: N=%d, Z=%d, C=%d, V=%d\n",RD2.entire,flags.N,flags.Z,flags.C,flags.V);
  ac_pc = RB.read(PC);
 
}

//------------------------------------------------------
void SWP(int rd, int rn, int rm,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  reg_t RN2, RM2, rtmp;
  long tmp;
  int rn10;

  dprintf("Instrucao: SWP\n");

  // Casos Especiais
  // verificar caso do coprocessador (alinhamento)
  if((rd == PC)||(rm == PC)||(rn == PC)||(rm == rn)||(rn == rd)){
    printf("Resultado da operacao SWP imprevisivel\n");
    return;
  }

  RN2.entire = RB.read(rn);
  RM2.entire = RB.read(rm);
  dprintf("rn=%d, contendo %ld\nrm=%d, contendo %ld\n", rn,RN2.entire,rm,RM2.entire);
  rn10 = (int) RN2.entire & 0x00000003;

  switch(rn10) {
  case 0:
    dprintf("Caso 0\n");
    tmp = MEM.read(RN2.entire);
    break;
  case 1:
    dprintf("Caso 1\n");
    rtmp.entire = MEM.read(RN2.entire);
    tmp = (RotateRight(8,rtmp)).entire;
    break;
  case 2:
    dprintf("Caso 2\n");
    rtmp.entire = MEM.read(RN2.entire);
    tmp = (RotateRight(16,rtmp)).entire;
    break;
  default:
    dprintf("Caso 3\n");
    rtmp.entire = MEM.read(RN2.entire);
    tmp = (RotateRight(24,rtmp)).entire;
  }
    
  dprintf("tmp contem %ld, rtmp contem %ld\n", tmp,rtmp.entire);
  MEM.write(RN2.entire,RM2.entire);
  RB.write(rd,tmp);

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void SWPB(int rd, int rn, int rm,
          ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
          ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  long tmp;
  reg_t RM2, RN2;

  dprintf("Instrucao: SWPB\n");

  // Casos Especiais
  if((rd == PC)||(rm == PC)||(rn == PC)||(rm == rn)||(rn == rd)){
    printf("Resultado da operacao SWPB imprevisivel\n");
    return;
  }

  RM2.entire = RB.read(rm);
  RN2.entire = RB.read(rn);
  dprintf("rn=%d, contendo %ld\nrm=%d, contendo %ld\n", rn,RN2.entire,rm,RM2.entire);

  tmp = (unsigned long)MEM.read_byte(RN2.entire);
  dprintf("tmp contem %ld\n",tmp);
  MEM.write_byte(RN2.entire,RM2.byte[0]);
  RB.write(rd,tmp);

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void TEQ(int rn,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RN2, alu_out;

  dprintf("Instrucao: TEQ\n");
  RN2.entire = RB.read(rn);
  dprintf("Operadores:\nRn=%d, contendo %ld\nShiftOp contendo %ld\n", rn,RN2.entire,dpi_shiftop.entire);
  alu_out.entire = RN2.entire ^ dpi_shiftop.entire;

  flags.N = getBit(alu_out.entire,31);
  flags.Z = ((alu_out.entire == 0) ? true : false);
  flags.C = dpi_shiftopcarry;
  // nada ocorre com flags.V
    
  dprintf("Flags: N=%d, Z=%d, C=%d, V=%d\n",flags.N,flags.Z,flags.C,flags.V);  
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void TST(int rn,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RN2, alu_out;

  dprintf("Instrucao: TST\n");
  RN2.entire = RB.read(rn);
  dprintf("Operadores:\nRn=%d, contendo %ld\nShiftOp contendo %ld\n", rn,RN2.entire,dpi_shiftop.entire);
  alu_out.entire = RN2.entire & dpi_shiftop.entire;

  flags.N = getBit(alu_out.entire, 31);
  flags.Z = ((alu_out.entire == 0) ? true : false);
  flags.C = dpi_shiftopcarry;
  // nada ocorre com flags.V
    
  dprintf("Flags: N=%d, Z=%d, C=%d, V=%d\n",flags.N,flags.Z,flags.C,flags.V); 
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void UMLAL(int rdhi, int rdlo, int rm, int rs, bool s,
           ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
           ac_reg<unsigned>& ac_pc) {

  r64bit_t result, acc;
  reg_t RM2, RS2;

  RM2.entire = RB.read(rm);
  RS2.entire = RB.read(rs);
  acc.reg[0] = RB.read(rdlo);
  acc.reg[1] = RB.read(rdhi);

  dprintf("Instrucao: UMLAL\n");
  dprintf("Operadores:\nrm=%d, contendo %ld\nrs=%d, contendo %ld\nSomar resultado da multiplicacao com %lld\nDestino(Hi): Rdhi=%d, Rdlo=%d\n", rm,RM2.entire,rs,RS2.entire,acc.hilo,rdhi,rdlo);

  // Casos especiais
  if((rdhi == PC)||(rdlo == PC)||(rm == PC)||(rs == PC)||(rdhi == rdlo)||(rdhi == rm)||(rdlo == rm)) {
    printf("Resultado da operacao UMLAL imprevisivel\n");
    return;  
  }

  result.hilo = ((unsigned long long)(unsigned long)RM2.entire * (unsigned long long)(unsigned long)RS2.entire) + acc.hilo;
  RB.write(rdhi,result.reg[1]);
  RB.write(rdlo,result.reg[0]);
  if(s == 1){
    flags.N = getBit(result.reg[1],31);
    flags.Z = ((result.hilo == 0) ? true : false);
    // Nada acontece com flags.C e flags.V
  }
  dprintf("Resultado: %lld\nFlags: N=%d, Z=%d, C=%d, V=%d\n",result.hilo,flags.N,flags.Z,flags.C,flags.V);
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void UMULL(int rdhi, int rdlo, int rm, int rs, bool s,
           ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
           ac_reg<unsigned>& ac_pc) {

  
  r64bit_t result;
  reg_t RM2, RS2;
  
  RM2.entire = RB.read(rm);
  RS2.entire = RB.read(rs);
  
  dprintf("Instrucao: UMULL\n");
  dprintf("Operadores:\nrm=%d, contendo %ld\nrs=%d, contendo %ld\nDestino(Hi): Rdhi=%d, Rdlo=%d\n", rm,RM2.entire,rs,RS2.entire,rdhi,rdlo);

  // Casos especiais
  if((rdhi == PC)||(rdlo == PC)||(rm == PC)||(rs == PC)||(rdhi == rdlo)||(rdhi == rm)||(rdlo == rm)) {
    printf("Resultado da operacao UMULL imprevisivel\n");
    return;  
  }
  
  result.hilo = ((unsigned long long)(unsigned long)RM2.entire * (unsigned long long)(unsigned long)RS2.entire);
  RB.write(rdhi,result.reg[1]);
  RB.write(rdlo,result.reg[0]);
  if(s == 1){
    flags.N = getBit(result.reg[1],31);
    flags.Z = ((result.hilo == 0) ? true : false);
    // Nada acontece com flags.C e flags.V
  }
  dprintf("Resultado: %lld\nFlags: N=%d, Z=%d, C=%d, V=%d\n",result.hilo,flags.N,flags.Z,flags.C,flags.V);
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
void DSMLA(int rd, int rn,
           ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
           ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2;

  RN2.entire = RB.read(rn);

  dprintf("Instrucao: SMLA<y><x>\n");
  dprintf("Operadores:\nrn=%d, contendo %ld\noperando1 contendo %ld\noperando2 contendo %ld\nrd=%d, contendo %ld\n", rn, RN2.entire, OP1.entire, OP2.entire, rd, RD2.entire);
  
  RD2.entire = (long)((OP1.entire * OP2.entire) + RN2.entire);

  RB.write(rd, RD2.entire);

  // SETAR FLAG Q
}

//------------------------------------------------------
void DSMUL(int rd,
           ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
           ac_reg<unsigned>& ac_pc) {

  reg_t RD2;

  dprintf("Instrucao: SMUL<y><x>\n");
  dprintf("Operadores:\noperando1 contendo %ld\noperando2 contendo %ld\nrd=%d, contendo %ld\n", OP1.entire, OP2.entire, rd, RD2.entire);
  
  RD2.entire = (long)(OP1.entire * OP2.entire);

  RB.write(rd, RD2.entire);

  // SETAR FLAG Q
}

//------------------------------------------------------


//!Instruction and1 behavior method.
void ac_behavior( and1 ){ AND(rd, rn, s, RB, ac_pc);}

//!Instruction eor1 behavior method.
void ac_behavior( eor1 ){ EOR(rd, rn, s, RB, ac_pc);}

//!Instruction sub1 behavior method.
void ac_behavior( sub1 ){ SUB(rd, rn, s, RB, ac_pc);}

//!Instruction rsb1 behavior method.
void ac_behavior( rsb1 ){ RSB(rd, rn, s, RB, ac_pc);}

//!Instruction add1 behavior method.
void ac_behavior( add1 ){ ADD(rd, rn, s, RB, ac_pc);}

//!Instruction adc1 behavior method.
void ac_behavior( adc1 ){ ADC(rd, rn, s, RB, ac_pc);}

//!Instruction sbc1 behavior method.
void ac_behavior( sbc1 ){ SBC(rd, rn, s, RB, ac_pc);}

//!Instruction rsc1 behavior method.
void ac_behavior( rsc1 ){ RSC(rd, rn, s, RB, ac_pc);}

//!Instruction tst1 behavior method.
void ac_behavior( tst1 ){ TST(rn, RB, ac_pc);}

//!Instruction teq1 behavior method.
void ac_behavior( teq1 ){ TEQ(rn, RB, ac_pc);}

//!Instruction cmp1 behavior method.
void ac_behavior( cmp1 ){ CMP(rn, RB, ac_pc);}

//!Instruction cmn1 behavior method.
void ac_behavior( cmn1 ){ CMN(rn, RB, ac_pc);}

//!Instruction orr1 behavior method.
void ac_behavior( orr1 ){ ORR(rd, rn, s, RB, ac_pc);}

//!Instruction mov1 behavior method.
void ac_behavior( mov1 ){ MOV(rd, s, RB, ac_pc);}

//!Instruction bic1 behavior method.
void ac_behavior( bic1 ){ BIC(rd, rn, s, RB, ac_pc);}

//!Instruction mvn1 behavior method.
void ac_behavior( mvn1 ){ MVN(rd, s, RB, ac_pc);}

//!Instruction and2 behavior method.
void ac_behavior( and2 ){ AND(rd, rn, s, RB, ac_pc);}

//!Instruction eor2 behavior method.
void ac_behavior( eor2 ){ EOR(rd, rn, s, RB, ac_pc);}

//!Instruction sub2 behavior method.
void ac_behavior( sub2 ){ SUB(rd, rn, s, RB, ac_pc);}

//!Instruction rsb2 behavior method.
void ac_behavior( rsb2 ){ RSB(rd, rn, s, RB, ac_pc);}

//!Instruction add2 behavior method.
void ac_behavior( add2 ){ ADD(rd, rn, s, RB, ac_pc);}

//!Instruction adc2 behavior method.
void ac_behavior( adc2 ){ ADC(rd, rn, s, RB, ac_pc);}

//!Instruction sbc2 behavior method.
void ac_behavior( sbc2 ){ SBC(rd, rn, s, RB, ac_pc);}

//!Instruction rsc2 behavior method.
void ac_behavior( rsc2 ){ RSC(rd, rn, s, RB, ac_pc);}

//!Instruction tst2 behavior method.
void ac_behavior( tst2 ){ TST(rn, RB, ac_pc);}

//!Instruction teq2 behavior method.
void ac_behavior( teq2 ){ TEQ(rn, RB, ac_pc);}

//!Instruction cmp2 behavior method.
void ac_behavior( cmp2 ){ CMP(rn, RB, ac_pc);}

//!Instruction cmn2 behavior method.
void ac_behavior( cmn2 ){ CMN(rn, RB, ac_pc);}

//!Instruction orr2 behavior method.
void ac_behavior( orr2 ){ ORR(rd, rn, s, RB, ac_pc);}

//!Instruction mov2 behavior method.
void ac_behavior( mov2 ){ MOV(rd, s, RB, ac_pc);}

//!Instruction bic2 behavior method.
void ac_behavior( bic2 ){ BIC(rd, rn, s, RB, ac_pc);}

//!Instruction mvn2 behavior method.
void ac_behavior( mvn2 ){ MVN(rd, s, RB, ac_pc);}

//!Instruction and3 behavior method.
void ac_behavior( and3 ){ AND(rd, rn, s, RB, ac_pc);}

//!Instruction eor3 behavior method.
void ac_behavior( eor3 ){ EOR(rd, rn, s, RB, ac_pc);}

//!Instruction sub3 behavior method.
void ac_behavior( sub3 ){ SUB(rd, rn, s, RB, ac_pc);}

//!Instruction rsb3 behavior method.
void ac_behavior( rsb3 ){ RSB(rd, rn, s, RB, ac_pc);}

//!Instruction add3 behavior method.
void ac_behavior( add3 ){ ADD(rd, rn, s, RB, ac_pc);}

//!Instruction adc3 behavior method.
void ac_behavior( adc3 ){ ADC(rd, rn, s, RB, ac_pc);}

//!Instruction sbc3 behavior method.
void ac_behavior( sbc3 ){ SBC(rd, rn, s, RB, ac_pc);}

//!Instruction rsc3 behavior method.
void ac_behavior( rsc3 ){ RSC(rd, rn, s, RB, ac_pc);}

//!Instruction tst3 behavior method.
void ac_behavior( tst3 ){ TST(rn, RB, ac_pc);}

//!Instruction teq3 behavior method.
void ac_behavior( teq3 ){ TEQ(rn, RB, ac_pc);}

//!Instruction cmp3 behavior method.
void ac_behavior( cmp3 ){ CMP(rn, RB, ac_pc);}

//!Instruction cmn3 behavior method.
void ac_behavior( cmn3 ){ CMN(rn, RB, ac_pc);}

//!Instruction orr3 behavior method.
void ac_behavior( orr3 ){ ORR(rd, rn, s, RB, ac_pc);}

//!Instruction mov3 behavior method.
void ac_behavior( mov3 ){ MOV(rd, s, RB, ac_pc);}

//!Instruction bic3 behavior method.
void ac_behavior( bic3 ){ BIC(rd, rn, s, RB, ac_pc);}

//!Instruction mvn3 behavior method.
void ac_behavior( mvn3 ){ MVN(rd, s, RB, ac_pc);}

//!Instruction b behavior method.
void ac_behavior( b ){ B(h, offset, RB, ac_pc);}

//!Instruction blx1 behavior method.
void ac_behavior( blx1 ){
  printf("BLX nao sera implementada\n");
}

//!Instruction bx behavior method.
void ac_behavior( bx ){ BX(rm, RB, ac_pc); }

//!Instruction blx2 behavior method.
void ac_behavior( blx2 ){
  printf("BLX nao sera implementada\n");
}

//!Instruction swp behavior method.
void ac_behavior( swp ){ SWP(rd, rn, rm, RB, ac_pc, MEM); }

//!Instruction swpb behavior method.
void ac_behavior( swpb ){ SWPB(rd, rn, rm, RB, ac_pc, MEM); }

//!Instruction mla behavior method.
void ac_behavior( mla ){ MLA(rn, rd, rm, rs, s, RB, ac_pc);}
// OBS: inversao dos parametros proposital ("fields with the same name...")

//!Instruction mul behavior method.
void ac_behavior( mul ){ MUL(rn, rm, rs, s, RB, ac_pc);}
// OBS: inversao dos parametros proposital ("fields with the same name...")

//!Instruction smlal behavior method.
void ac_behavior( smlal ){ SMLAL(rdhi, rdlo, rm, rs, s, RB, ac_pc);}

//!Instruction smull behavior method.
void ac_behavior( smull ){ SMULL(rdhi, rdlo, rm, rs, s, RB, ac_pc);}

//!Instruction umlal behavior method.
void ac_behavior( umlal ){ UMLAL(rdhi, rdlo, rm, rs, s, RB, ac_pc);}

//!Instruction umull behavior method.
void ac_behavior( umull ){ UMULL(rdhi, rdlo, rm, rs, s, RB, ac_pc);}

//!Instruction ldr1 behavior method.
void ac_behavior( ldr1 ){ LDR(rd, rn, RB, ac_pc, MEM);  }

//!Instruction ldrt1 behavior method.
void ac_behavior( ldrt1 ){ LDRT(rd, rn, RB, ac_pc, MEM); }

//!Instruction ldrb1 behavior method.
void ac_behavior( ldrb1 ){ LDRB(rd, rn, RB, ac_pc, MEM); }

//!Instruction ldrbt1 behavior method.
void ac_behavior( ldrbt1 ){ LDRBT(rd, rn, RB, ac_pc, MEM); }

//!Instruction str1 behavior method.
void ac_behavior( str1 ){ STR(rd, rn, RB, ac_pc, MEM); }

//!Instruction strt1 behavior method.
void ac_behavior( strt1 ){ STRT(rd, rn, RB, ac_pc, MEM); }

//!Instruction strb1 behavior method.
void ac_behavior( strb1 ){ STRB(rd, rn, RB, ac_pc, MEM); }

//!Instruction strbt1 behavior method.
void ac_behavior( strbt1 ){ STRBT(rd, rn, RB, ac_pc, MEM); }

//!Instruction ldr2 behavior method.
void ac_behavior( ldr2 ){ LDR(rd, rn, RB, ac_pc, MEM); }

//!Instruction ldrt2 behavior method.
void ac_behavior( ldrt2 ){ LDRT(rd, rn, RB, ac_pc, MEM); }

//!Instruction ldrb2 behavior method.
void ac_behavior( ldrb2 ){ LDRB(rd, rn, RB, ac_pc, MEM); }

//!Instruction ldrbt2 behavior method.
void ac_behavior( ldrbt2 ){ LDRBT(rd, rn, RB, ac_pc, MEM); }

//!Instruction str2 behavior method.
void ac_behavior( str2 ){ STR(rd, rn, RB, ac_pc, MEM); }

//!Instruction strt2 behavior method.
void ac_behavior( strt2 ){ STRT(rd, rn, RB, ac_pc, MEM); }

//!Instruction strb2 behavior method.
void ac_behavior( strb2 ){ STRB(rd, rn, RB, ac_pc, MEM); }

//!Instruction strbt2 behavior method.
void ac_behavior( strbt2 ){ STRBT(rd, rn, RB, ac_pc, MEM); }

//!Instruction ldrh behavior method.
void ac_behavior( ldrh ){ LDRH(rd, rn, RB, ac_pc, MEM); }

//!Instruction ldrsb behavior method.
void ac_behavior( ldrsb ){ LDRSB(rd, rn, RB, ac_pc, MEM); }

//!Instruction ldrsh behavior method.
void ac_behavior( ldrsh ){ LDRSH(rd, rn, RB, ac_pc, MEM); }

//!Instruction strh behavior method.
void ac_behavior( strh ){ STRH(rd, rn, RB, ac_pc, MEM); }

//!Instruction ldm behavior method.
void ac_behavior( ldm ){ LDM(rlist,r, RB, ac_pc, MEM); }

//!Instruction stm behavior method.
void ac_behavior( stm ){ STM(rlist, RB, ac_pc, MEM); }

//!Instruction cdp behavior method.
void ac_behavior( cdp ){ CDP();}

//!Instruction mcr behavior method.
void ac_behavior( mcr ){ MCR();}

//!Instruction mrc behavior method.
void ac_behavior( mrc ){ MRC();}

//!Instruction ldc behavior method.
void ac_behavior( ldc ){ LDC();}

//!Instruction stc behavior method.
void ac_behavior( stc ){ STC();}

//!Instruction bkpt behavior method.
void ac_behavior( bkpt ){
  printf("BKPT nao sera implementada\n");
}

//!Instruction swi behavior method.
void ac_behavior( swi ){
  printf("SWI nao sera implementada\n");
}

//!Instruction clz behavior method.
void ac_behavior( clz ){ CLZ(rd, rm, RB, ac_pc);}

//!Instruction mrs behavior method.
void ac_behavior( mrs ){ MRS(rd,r,zero3,subop2,func2,subop1,rm,field, RB, ac_pc);}

//!Instruction msr1 behavior method.
void ac_behavior( msr1 ){
  printf("MSR nao sera implementada\n");
}

//!Instruction msr2 behavior method.
void ac_behavior( msr2 ){
  printf("MSR nao sera implementada\n");
}

//!Instruction ldrd2 behavior method.
void ac_behavior( ldrd ){ LDRD(rd, rn, RB, ac_pc, MEM); }

//!Instruction ldrd2 behavior method.
void ac_behavior( strd ){ STRD(rd, rn, RB, ac_pc, MEM); }

//!Instruction dsmla behavior method.
void ac_behavior( dsmla ){ DSMLA(drd, drn, RB, ac_pc); }

//!Instruction dsmlal behavior method.
void ac_behavior( dsmlal ){
  printf("SMLAL<y><x> nao implementada\n");
}

//!Instruction dsmul behavior method.
void ac_behavior( dsmul ){ DSMUL(drd, RB, ac_pc); }

//!Instruction dsmlaw behavior method.
void ac_behavior( dsmlaw ){
  printf("SMLAW<y><x> nao implementada\n");
}

//!Instruction dsmulw behavior method.
void ac_behavior( dsmulw ){
  printf("SMULW<y><x> nao implementada\n");
}

///Behaviors begin and end
void ac_behavior( begin ) { }
void ac_behavior( end ) { }
