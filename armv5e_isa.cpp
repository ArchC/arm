/**
 * @file      armv5e_isa.cpp
 * @author    Danilo Marcolin Caravana
 *            Rafael Auler
 *
 *            The ArchC Team
 *            http://www.archc.org/
 *
 *            Computer Systems Laboratory (LSC)
 *            IC-UNICAMP
 *            http://www.lsc.ic.unicamp.br/
 *
 * @version   0.7
 * @date      Jul 2009
 * 
 * @brief     The ArchC ARMv5e functional model.
 * 
 * @attention Copyright (C) 2002-2009 --- The ArchC Team
 *
 */

#include "armv5e_isa.H"
#include "armv5e_isa_init.cpp"
#include "armv5e_bhv_macros.H"

using namespace armv5e_parms;

//DEBUG
static const int DEBUG_INSTR = 1;

/* Uncomment the line below to enable debugging info for this model. */
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

//! User defined macros to access a single bit
#define isBitSet(variable, position) (((variable & (1 << (position))) != 0) ? true : false) 
#define getBit(variable, position) (((variable & (1 << (position))) != 0) ? true : false)
#define setBit(variable, position) variable = variable | (1 << (position))
#define clearBit(variable, position) variable = variable & (~(1 << (position)))

//! User defined macros to reference registers
#define LR 14 // link return
#define PC 15 // program counter

//! Useful abstract data types defining ARM flags and register access
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

//! Global instances used throughout the model.
static flag_t flags;
static bool execute;

static reg_t dpi_shiftop;
static bool dpi_shiftopcarry;

static reg_t ls_address;
static reg_t lsm_startaddress;
static reg_t lsm_endaddress;

static reg_t OP1;
static reg_t OP2;

//! Useful functions to easily describe arm instructions behavior

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
  for (i=0; i<16; i++) { // Verify limits for big/little endian
    if (isBitSet(registerList.entire,i)) count++;
  }
  return count;
}

inline reg_t CPSRBuild() {

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

  dprintf("----- PC=%#x ----- %lld\nins:%lX", (unsigned int)ac_pc, ac_instr_counter, (unsigned int)MEM.read(ac_pc));

  // Conditionally executes instruction based on COND field, common to all ARM instructions.
  execute = false;
  dprintf("cond=0x%X\n", cond);

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

  // PC increment
  ac_pc += 4;
  RB.write(PC, ac_pc);

  if(!execute) {
    dprintf("Instruction will not be executed due to condition flags.\n");
    ac_annul();
  }
}
 
// Instruction Format behavior methods.

//!DPI1 - Second operand is register with imm shift
void ac_behavior( Type_DPI1 ) {

  reg_t RM2;

  dprintf("Instruction type: DPI1\n");
  
  // Special case: rm = 15
  if (rm == 15) {
    dprintf("Rm=PC -> Rm=Rm+8\n");
    // PC is already incremented by four, so only add 4 again (not 8)
    RM2.entire = RB.read(rm) + 4;
  }
  else RM2.entire = RB.read(rm);
      
  switch(shift) {
  case 0: // Logical shift left
    dprintf("shift=00 -> Logical shift left\nshiftamount=0x%X\n", shiftamount);
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
    dprintf("shift=01 -> Logical shift right\nshiftamount=0x%X\n", shiftamount);
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
    dprintf("shift=10 -> Arithmetic shift right\nshiftamount=0x%X\n", shiftamount);
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
    dprintf("shift=11 -> Rotate right\nshiftamount=0x%X\n", shiftamount);
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
  dprintf("Shifted operand value:\noperand=0x%lX\ncarry=0x%X\n", dpi_shiftop.entire,dpi_shiftopcarry);
}

//!DPI2 - Second operand is shifted (shift amount given by third register operand)
void ac_behavior( Type_DPI2 ) {

  int rs40;
  reg_t RS2, RM2;

  dprintf("Instruction type: DPI2\n");

  // Special case: r* = 15
  if ((rd == 15)||(rm == 15)||(rn == 15)||(rs == 15)) {
    printf("Register 15 cannot be used in this instruction.\n");
    ac_annul(); 
  }

  RM2.entire = RB.read(rm);
  RS2.entire = RB.read(rs);
  rs40 = ((unsigned int)RS2.entire) & 0x0000000F;

  switch(shift){
  case 0: // Logical shift left
    dprintf("shift=00 -> Logical shift left\nRS2.byte[0]=0x%X\n", RS2.byte[0]);
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
    dprintf("shift=01 -> Logical shift right\nRS2.byte[0]=0x%X\n", RS2.byte[0]);
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
    dprintf("shift=10 -> Arithmetic shift right\nRS2.byte[0]=0x%X\nrs40=0x%X\n", RS2.byte[0],rs40);
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
    dprintf("shift=11 -> Rotate right\nRS2.byte[0]=0x%X\nrs40=0x%X\n", RS2.byte[0],rs40);
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
  dprintf("Shifted operand value:\noperand=0x%lX\ncarry=0x%X\n", dpi_shiftop.entire,dpi_shiftopcarry);
}

//!DPI3 - Second operand is immediate shifted by another imm
void ac_behavior( Type_DPI3 ){

  long tmp;

  dprintf("Instruction type: DPI3\n");
  dprintf("rotate=0x%X\nimm8=0x%X\n", rotate,imm8);

  tmp = (unsigned long)imm8;
  dpi_shiftop.entire = (((unsigned long)tmp) >> (2 * rotate)) | (((unsigned long)tmp) << (32 - (2 * rotate)));

  if (rotate == 0) 
    dpi_shiftopcarry = flags.C;
  else 
    dpi_shiftopcarry = getBit(dpi_shiftop.entire, 31);
    
  dprintf("Shifted operand value:\noperand=0x%lX\ncarry=0x%X\n", dpi_shiftop.entire,dpi_shiftopcarry);
}

void ac_behavior( Type_BBL ) {
  dprintf("Instruction type: BBL\n");
  // no special actions necessary
}
void ac_behavior( Type_BBLT ) {
  dprintf("Instruction type: BBLT\n");
  // no special actions necessary
}
void ac_behavior( Type_MBXBLX ) {
  dprintf("Instruction type: MBXBLX\n");
  // no special actions necessary
}

//!MULT1 - 32-bit result multiplication
void ac_behavior( Type_MULT1 ) {
  dprintf("Instruction type: MULT1\n");
  // no special actions necessary
}

//!MULT2 - 64-bit result multiplication
void ac_behavior( Type_MULT2 ) {
  dprintf("Instruction type: MULT2\n");
  // no special actions necessary
}

//!LSI - Load Store Immediate Offset/Index
void ac_behavior( Type_LSI ) {

  reg_t RN2;
  RN2.entire = RB.read(rn);
  
  dprintf("rn=0x%X, contains 0x%lX\n", rn,RN2.entire);

  dprintf("Instruction type: LSI\n");
  ls_address.entire = 0;
    
  if((p == 1)&&(w == 0)) { // immediate pre-indexed without writeback
    dprintf("Mode: Immediate pre-indexed without writeback\n");
    
    // Special case: Rn = PC
    if (rn == PC) 
      ls_address.entire = 4;
    
    if(u == 1) {
      dprintf("Add imm (u == 1)\n");
      ls_address.entire += RN2.entire + (unsigned long) imm12;
    } else {
      dprintf("Subtract imm (u == 0)\n");
      ls_address.entire += RN2.entire - (unsigned long) imm12;
    }
    dprintf("ls_address = 0x%lX\n", ls_address.entire);
  }

  else if((p == 1)&&(w == 1)) { // immediate pre-indexed with writeback
    dprintf("Mode: Immediate pre-indexed with writeback\n");

    // Special case: Rn = PC
    if (rn == PC) {
      printf("Unpredictable LSI instruction result (Can't writeback to PC, Rn = PC)\n");
      ac_annul();
      return;
    }
    // Special case: Rn = Rd
    if (rn == rd) {
      printf("Unpredictable LSI instruction result  (Can't writeback to loaded register, Rn = Rd)\n");
      ac_annul();
      return;
    }
    
    if(u == 1) {
      dprintf("Add imm (u == 1)\n");
      ls_address.entire = RN2.entire + (unsigned long) imm12;
    } else {
      dprintf("Subtract imm (u == 0)\n");
      ls_address.entire = RN2.entire - (unsigned long) imm12;
    }
    RB.write(rn,ls_address.entire);
    dprintf("ls_address = 0x%lX\n", ls_address.entire);
  }

  else if((p == 0)&&(w == 0)) { // immediate post-indexed (writeback)
    dprintf("Mode: Immediate post-indexed\n");    

    // Special case: Rn = PC
    if (rn == PC) {
      printf("Unpredictable LSI instruction result (Can't writeback to PC, Rn = PC)\n");
      ac_annul();
      return;
    }
    // Special case Rn = Rd
    if (rn == rd) {
      printf("Unpredictable LSI instruction result (Can't writeback to loaded register, Rn = Rd)\n");
      ac_annul();
      return;
    }
    
    ls_address.entire = RN2.entire;
    if(u == 1) {
      //checar se imm12 soma direto
      dprintf("Add imm (u == 1)\n");
      RB.write(rn, ls_address.entire + (unsigned long) imm12);
    } else {
      dprintf("Subtract imm (u == 0)\n");
      RB.write(rn, ls_address.entire - (unsigned long) imm12);
    }
    dprintf("ls_address = 0x%lX\n", ls_address.entire);
  }
  /* FIXME: Check word alignment (Rd = PC) Address[1:0] = 0b00 */

}

//!LSR - Scaled Register Offset/Index
void ac_behavior( Type_LSR ) {

  reg_t RM2, RN2, index, tmp;

  dprintf("Instruction type: LSR\n");
  RM2.entire = RB.read(rm);
  RN2.entire = RB.read(rn);
  dprintf("rm=0x%X, contains 0x%lX\nrn=0x%X, contains 0x%lX\n", rm,RM2.entire,rn,RN2.entire);
  ls_address.entire = 0;

  if ((p == 1)&&(w == 0)) { // offset
    dprintf("Mode: pre-indexed without writeback\n");

    // Special case: PC
    if(rn == PC) 
      ls_address.entire = 4;

    if(rm == PC) {
      printf("Unpredictable LSR instruction result (Illegal usage of PC, Rm = PC)\n");
      return;
    }

    switch(shift){
    case 0:
      if(shiftamount == 0) { // Register
	dprintf("shift=00, shiftamount=0 -> Register\n");
	index.entire = RM2.entire;
      } else { // Scalled logical shift left
	dprintf("shift=00, shiftamount=0x%X -> Scalled logical shift left\n", shiftamount);
	index.entire = RM2.entire << shiftamount;
      }
      break;
    case 1: // logical shift right
      dprintf("shift=01 -> Logical Shift Right\nshiftamount=0x%X\n", shiftamount);
      if(shiftamount == 0) index.entire = 0;
      else index.entire = ((unsigned long) RM2.entire) >> shiftamount;
      break;
    case 2: // arithmetic shift right
      dprintf("shift=10 -> Arithmetic Shift Right\nshiftamount=0x%X\n", shiftamount);
      if(shiftamount == 0) {
	if (isBitSet(RM2.entire, 31)) index.entire = 0xFFFFFFFF;
	else index.entire = 0;
      } else index.entire = ((signed long) RM2.entire) >> shiftamount;
      break;
    default:
      if(shiftamount == 0) { // RRX
	dprintf("shift=11, shiftamount=0 -> RRX\nshiftamount=0x%X\n", shiftamount);
	tmp.entire = 0;
	if(flags.C) setBit(tmp.entire, 31);
	index.entire = tmp.entire | (((unsigned long) RM2.entire) >> 1);
      } else { // rotate right
	dprintf("shift=11, shiftamount=0x%X -> Rotate Right\n", shiftamount);
	index.entire = (RotateRight(shiftamount, RM2)).entire;
      }
    }

    if(u == 1) {
      dprintf("Add imm (u == 1)\n");
      ls_address.entire += (RN2.entire + index.entire);
    } else {
      dprintf("Subtract imm (u == 0)\n");
      ls_address.entire += (RN2.entire - index.entire);
    }
  }

  else if((p == 1)&&(w == 1)) { // pre-indexed
    dprintf("Mode: pre-indexed with writeback\n");

    // Special case: Rn = PC
    if (rn == PC) {
      printf("Unpredictable LSR instruction result (Can't writeback to PC, Rn = PC)\n");
      ac_annul();
      return;
    }
    // Special case Rn = Rd
    if (rn == rd) {
      printf("Unpredictable LSR instruction result (Can't writeback to loaded register, Rn = Rd)\n");
      ac_annul();
      return;
    }
    // Special case Rm = PC
    if (rm == PC) {
      printf("Unpredictable LSR instruction result (Illegal usage of PC, Rm = PC)\n");
      ac_annul();
      return;
    }
    // Special case Rn = Rm
    if (rn == rm) {
      printf("Unpredictable LSR instruction result (Can't use the same register for Rn and Rm\n");
      ac_annul();
      return;
    }
    
    switch(shift){
    case 0:
      dprintf("shift=00, shiftamount=0 -> Register\n");
      if(shiftamount == 0) { // Register
	index.entire = RM2.entire;
      } else { // Scaled logical shift left
	dprintf("shift=00, shiftamount=0x%X -> Scalled logical shift left\n", shiftamount);
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
	dprintf("shift=11, shiftamount=0x%X -> Rotate Right\n", shiftamount);
	index.entire = (RotateRight(shiftamount, RM2)).entire;
      }
    }

    if(u == 1) {
      dprintf("Add imm (u == 1)\n");
      ls_address.entire = RN2.entire + index.entire;
    } else {
      dprintf("Subtract imm (u == 0)\n");
      ls_address.entire = RN2.entire - index.entire;    
    }

    RB.write(rn, ls_address.entire);
  }

  else if((p == 0)&&(w == 0)) { // post-indexed
    dprintf("Caso: post-indexed\n");

    // Special case: Rn = PC
    if (rn == PC) {
      printf("Unpredictable LSR instruction result (Can't writeback to PC, Rn = PC)\n");
      ac_annul();
      return;
    }
    // Special case Rn = Rd
    if (rn == rd) {
      printf("Unpredictable LSR instruction result (Can't writeback to loaded register, Rn = Rd)\n");
      ac_annul();
      return;
    }
    // Special case Rm = PC
    if (rm == PC) {
      printf("Unpredictable LSR instruction result (Illegal usage of PC, Rm = PC)\n");
      ac_annul();
      return;
    }
    // Special case Rn = Rm
    if (rn == rm) {
      printf("Unpredictable LSR instruction result (Can't use the same register for Rn and Rm\n");
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
	dprintf("shift=00, shiftamount=0x%X -> Scalled logical shift left\n", shiftamount);
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
	dprintf("shift=11, shiftamount=0x%X -> Rotate Right\n", shiftamount);
	index.entire = (RotateRight(shiftamount, RM2)).entire;
      }
    }

    if(u == 1) {
      dprintf("Add imm (u == 1)\n");
      RB.write(rn, RN2.entire + index.entire);
    } else {
      dprintf("Subtract imm (u == 0)\n");
      RB.write(rn, RN2.entire - index.entire);
    }
  } 
  dprintf("Calculated address: 0x%lX\nrn=0x%X contains 0x%lX\n", ls_address.entire,rn,RN2.entire);
}

//!LSE - Load Store HalfWord
void ac_behavior( Type_LSE ){

  long off8;
  reg_t RM2, RN2;

  dprintf("Instruction type: LSE\n");

  // Special cases handling
  if((p == 0)&&(w == 1)) {
    printf("Unpredictable LSE instruction result");
    ac_annul();
    return;
  }
  if((ss == 0)&&(hh == 0)) {
    printf("Decoding error: this is not a LSE instruction");
    ac_annul();
    return;
  }
  if((ss == 1)&&(l == 0)) 
    dprintf("Special DSP\n");
    // FIXME: Test LDRD and STRD second registers in case of writeback

  RN2.entire = RB.read(rn);
  dprintf("rn=0x%X, contains 0x%lX\n", rn,RN2.entire);

  // nos LSE's que usam registrador, o campo addr2 armazena Rm
  RM2.entire = RB.read(addr2);
  off8 = ((unsigned long)(addr1 << 4) | addr2);
  ls_address.entire = 0;

  if(p == 1) { // offset ou pre-indexed
    if((i == 1)&&(w == 0)) { // immediate offset
      dprintf("Mode: Immediate offset without writeback\n");

      if(rn == PC) 
	ls_address.entire = 4;

      dprintf("addr1=0x%X, addr2=0x%X, off8=0x%X\n", addr1,addr2,off8);

      if(u == 1) {
	dprintf("Add imm (u == 1)\n");
	ls_address.entire += (RN2.entire + off8);
      } else {
	dprintf("Subtract imm (u == 0)\n");
	ls_address.entire += (RN2.entire - off8);
      }
    }

    else if((i == 0)&&(w == 0)) { // register offset
      dprintf("Mode: Register offset without writeback\n");
      dprintf("rm=0x%X, contains 0x%lX\n", addr2,RM2.entire);
      
      // Special case Rm = PC
      if (addr2 == PC) {
	printf("Unpredictable LSE instruction result (Illegal usage of PC, Rm = PC)\n");
	ac_annul();
	return;
      }

      if(rn == PC) 
	ls_address.entire = 4;

      if(u == 1) {
	dprintf("Add imm (u == 1)\n");
	ls_address.entire += (RN2.entire + RM2.entire);
      } else  {
	dprintf("Subtract imm (u == 0)\n");
	ls_address.entire += (RN2.entire - RM2.entire);
      }
    }
    
    else if ((i == 1)&&(w == 1)) { // immediate pre-indexed
      dprintf("Mode: Immediate pre-indexed with writeback\n");

      // Special case: Rn = PC
      if (rn == PC) {
	printf("Unpredictable LSE instruction result (Can't writeback to PC, Rn = PC)\n");
	ac_annul();
	return;
      }
      // Special case Rn = Rd
      if (rn == rd) {
	printf("Unpredictable LSE instruction result (Can't writeback to loaded register, Rn = Rd)\n");
	ac_annul();
	return;
      }
      
      if(u == 1) {
	dprintf("Add imm (u == 1)\n");
	ls_address.entire = (RN2.entire + off8);
      } else {
	dprintf("Subtract imm (u == 0)\n");
	ls_address.entire = (RN2.entire - off8);
      }

      RB.write(rn, ls_address.entire);
    }
    
    else { // i == 0 && w == 1: register pre-indexed
      dprintf("Mode: Register pre-indexed with writeback\n");
      dprintf("rm=0x%X, contains 0x%lX\n", addr2,RM2.entire);

      // Special case: Rn = PC
      if (rn == PC) {
	printf("Unpredictable LSE instruction result (Can't writeback to PC, Rn = PC)\n");
	ac_annul();
	return;
      }
      // Special case Rn = Rd
      if (rn == rd) {
	printf("Unpredictable LSE instruction result (Can't writeback to loaded register, Rn = Rd)\n");
	ac_annul();
	return;
      }
      // Special case Rm = PC
      if (addr2 == PC) {
	printf("Unpredictable LSE instruction result (Illegal usage of PC, Rm = PC)\n");
	ac_annul();
	return;
      }
      // Special case Rn = Rm
      if (rn == addr2) {
	printf("Unpredictable LSE instruction result (Can't use the same register for Rn and Rm\n");
	ac_annul();
	return;
      }
      
      if(u == 1) {
	dprintf("Add imm (u == 1)\n");
	ls_address.entire = (RN2.entire + RM2.entire);
      } else {
	dprintf("Subtract imm (u == 0)\n");
	ls_address.entire = (RN2.entire - RM2.entire);
      }

      RB.write(rn, ls_address.entire);
    }

  } else { // p == 0: post-indexed
    if((i == 1)&&(w == 0)) { // immediate post-indexed
      dprintf("Mode: Immediate post-indexed\n");

      if(rn == PC) {
	printf("Unpredictable LSE instruction result");
	ac_annul();
	return;
      }

      ls_address.entire = RN2.entire;
      if(u == 1) {
	dprintf("Add imm (u == 1)\n");
	RB.write(rn, RN2.entire + off8);
      } else {
	dprintf("Subtract imm (u == 0)\n");
	RB.write(rn, RN2.entire - off8);
      }
    }
    else if((i == 0)&&(w == 0)) { // register post-indexed
      dprintf("Mode: Register post-indexed\n");
      dprintf("rm=0x%X, contains 0x%lX\n", addr2,RM2.entire);

      // Special case: Rn = PC
      if (rn == PC) {
	printf("Unpredictable LSE instruction result (Can't writeback to PC, Rn = PC)\n");
	ac_annul();
	return;
      }
      // Special case Rn = Rd
      if (rn == rd) {
	printf("Unpredictable LSE instruction result (Can't writeback to loaded register, Rn = Rd)\n");
	ac_annul();
	return;
      }
      // Special case Rm = PC
      if (addr2 == PC) {
	printf("Unpredictable LSE instruction result (Illegal usage of PC, Rm = PC)\n");
	ac_annul();
	return;
      }
      // Special case Rn = Rm
      if (rn == addr2) {
	printf("Unpredictable LSE instruction result (Can't use the same register for Rn and Rm\n");
	ac_annul();
	return;
      }
      
      ls_address.entire = RN2.entire;
      if(u == 1) {
	dprintf("Add imm (u == 1)\n");
	RB.write(rn, RN2.entire + RM2.entire);
      } else {
	dprintf("Subtract imm (u == 0)\n");
	RB.write(rn, RN2.entire - RM2.entire);
      }
    }
  }

  dprintf("Calculated address: 0x%lX\n", ls_address.entire);
}

//!LSM - Load Store Multiple
void ac_behavior( Type_LSM ){

  reg_t RN2;
  int setbits;

  dprintf("Instruction type: LSM\n");

  // Put registers list in a variable capable of addressing individual bits
  reg_t registerList;
  dprintf("Registers list: 0x%X\n", rlist);
  registerList.entire = (unsigned long) rlist;

  // Special case - empty list
  if (registerList.entire == 0) {
    printf("Unpredictable LSM instruction result (No register specified)\n");
    ac_annul();
    return;
  }
  
  RN2.entire = RB.read(rn);
  setbits = LSM_CountSetBits(registerList);
  dprintf("Bits set: 0x%X\n", setbits);

  if((p == 0)&&(u == 1)) { // increment after
    dprintf("Operation mode: IA\n");
    lsm_startaddress.entire = RN2.entire;
    lsm_endaddress.entire = RN2.entire + (setbits * 4) - 4;
    if(w == 1) RN2.entire += (setbits * 4);  
  }
  else if((p == 1)&&(u == 1)) { // increment before
    dprintf("Operation mode: IB\n");
    lsm_startaddress.entire = RN2.entire + 4; 
    lsm_endaddress.entire = RN2.entire + (setbits * 4);
    if(w == 1) RN2.entire += (setbits * 4);
  }
  else if((p == 0)&&(u == 0)) { // decrement after
    dprintf("Operation mode: DA\n");
    lsm_startaddress.entire = RN2.entire - (setbits * 4) + 4;
    lsm_endaddress.entire = RN2.entire;
    if(w == 1) RN2.entire -= (setbits * 4);
  }
  else { // decrement before
    dprintf("Operation mode: DB\n");
    lsm_startaddress.entire = RN2.entire - (setbits * 4);
    lsm_endaddress.entire = RN2.entire - 4;
    if(w == 1) RN2.entire -= (setbits * 4);
  }

  // Special case Rn in Rlist
  if((w == 1)&&(isBitSet(rlist,rn))) {
    printf("Unpredictable LSM instruction result (Can't writeback to loaded register, Rn in Rlist)\n");
    ac_annul();
    return;
  }

  dprintf("Memory addresses:\nInitial = 0x%lX\nFinal = 0x%lX\n", lsm_startaddress.entire,lsm_endaddress.entire);

  RB.write(rn,RN2.entire);
}

void ac_behavior( Type_CDP ){
  dprintf("Instruction type: CDP\n");
  // no special actions necessary
}
void ac_behavior( Type_CRT ){
  dprintf("Instruction type: CRT\n");
  // no special actions necessary
}
void ac_behavior( Type_CLS ){
  dprintf("Instruction type: CLS\n");
  // no special actions necessary
}
void ac_behavior( Type_MBKPT ){
  dprintf("Instruction type: MBKPT\n");
  // no special actions necessary
}
void ac_behavior( Type_MSWI ){
  dprintf("Instruction type: MSWI\n");
  // no special actions necessary
}
void ac_behavior( Type_MCLZ ){
  dprintf("Instruction type: MCLZ\n");
  // no special actions necessary
}
void ac_behavior( Type_MMSR1 ){
  dprintf("Instruction type: MMSR1\n");
  // no special actions necessary
}
void ac_behavior( Type_MMSR2 ){
  dprintf("Instruction type: MMSR2\n");
  // no special actions necessary
}

void ac_behavior( Type_DSPSM ){

  reg_t RM2, RS2;
  
  RM2.entire = RB.read(rm);
  RS2.entire = RB.read(rs);
  
  dprintf("Instruction type: DSPSM\n");
  dprintf("rs=0x%X, contains 0x%lX\nrm=0x%X, contains 0x%lX\nyy=0x%X\nxx=0x%X", rs, RS2.entire, rm, RM2.entire, yy, xx);

  // Special cases
  if((drd == PC)||(drn == PC)||(rm == PC)||(rs == PC)) {
    printf("Unpredictable SMLA<y><x> instruction result\n");
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
inline void ADC(int rd, int rn, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2;
  r64bit_t soma;

  dprintf("Instruction: ADC\n");
  RN2.entire = RB.read(rn);
  if(rn == PC) RN2.entire += 4;
  dprintf("Operands:\nRn=0x%X, contains 0x%lX\nShiftOp contains 0x%lX\nC=0x%X\nDestination: Rd=0x%X\n", rn,RN2.entire,dpi_shiftop.entire,flags.C,rd);
  soma.hilo = ((unsigned long long)(unsigned long)RN2.entire + (unsigned long long)(unsigned long)dpi_shiftop.entire);
  if (flags.C) soma.hilo++;
  RD2.entire = soma.reg[0];
  RB.write(rd, RD2.entire);
  if ((s == 1)&&(rd == PC)) {
    printf("Unpredictable ADC instruction result\n");
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
  dprintf("Results: 0x%lX\nFlags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n", RD2.entire,flags.N,flags.Z,flags.C,flags.V);
  dprintf(" *  R%d <= 0x%08X (%d)\n", rd, RD2.entire, RD2.entire); 
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void ADD(int rd, int rn, bool s,
    ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
    ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2;
  r64bit_t soma;

  dprintf("Instruction: ADD\n");
  RN2.entire = RB.read(rn);
  if(rn == PC) RN2.entire += 4;
  dprintf("Operands:\nRn=0x%X, contains 0x%lX\nShiftOp contains 0x%lX\nDestination: Rd=0x%X\n", rn,RN2.entire,dpi_shiftop.entire,rd);
  soma.hilo = ((unsigned long long)(unsigned long)RN2.entire + (unsigned long long)(unsigned long)dpi_shiftop.entire);
  RD2.entire = soma.reg[0];
  RB.write(rd, RD2.entire);
  if ((s == 1)&&(rd == PC)) {
    printf("Unpredictable ADD instruction result\n");
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
  dprintf("Results: 0x%lX\nFlags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n", RD2.entire,flags.N,flags.Z,flags.C,flags.V);
  dprintf(" *  R%d <= 0x%08X (%d)\n", rd, RD2.entire, RD2.entire); 
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void AND(int rd, int rn, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2;
  
  dprintf("Instruction: AND\n");
  RN2.entire = RB.read(rn);
  dprintf("Operands:\nRn=0x%X, contains 0x%lX\nShiftOp contains 0x%lX\nDestination: Rd=0x%X\n", rn,RN2.entire,dpi_shiftop.entire,rd);
  RD2.entire = RN2.entire & dpi_shiftop.entire;
  RB.write(rd, RD2.entire);

  if ((s == 1)&&(rd == PC)) {
    printf("Unpredictable AND instruction result\n");
    return;
  } else {
    if (s == 1) {
      flags.N = getBit(RD2.entire, 31);
      flags.Z = ((RD2.entire == 0) ? true : false);
      flags.C = dpi_shiftopcarry;
      // nothing happens with flags.V 
    }
  }   
  dprintf("Results: 0x%lX\nFlags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n", RD2.entire,flags.N,flags.Z,flags.C,flags.V);
  dprintf(" *  R%d <= 0x%08X (%d)\n", rd, RD2.entire, RD2.entire); 
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void B(int h, int offset,
       ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
       ac_reg<unsigned>& ac_pc) {

  long long mem_pos;
  long s_extend;

  // Note that PC is already incremented by 4, i.e., pointing to the next instruction

  if(h == 1) { // h? it is really "l"
    dprintf("Instruction: BL\n");
    RB.write(LR, RB.read(PC));
    dprintf("Branch return address: 0x%lX\n", RB.read(LR));
  } else dprintf("Instruction: B\n");

  dprintf("offset=0x%lX\n",offset);
  s_extend = SignExtend((long) (offset << 2),26);
  dprintf("s_extend=0x%lX\n",s_extend);

  mem_pos = (long long) RB.read(PC) + 4 + s_extend;
  dprintf("Calculated branch destination: 0x%llX\n", mem_pos);
  if((mem_pos < 0)) {
    fprintf(stderr, "Branch destination out of bounds\n");
    exit(EXIT_FAILURE);
    return;
  } else RB.write(PC,(long) mem_pos);

  //fprintf(stderr, "0x%X\n", (unsigned int)mem_pos);

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void BX(int rm,
        ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
        ac_reg<unsigned>& ac_pc) {

  dprintf("Instruction: BX\n");

  if(isBitSet(rm,0)) {
    dprintf("Change to thumb not implemented in this model. PC=%X\n", ac_pc.read());
    return;
  } else 
    dprintf("Regular ARM instruction\n");

  flags.T = isBitSet(rm, 0);
  ac_pc = RB.read(rm) & 0xFFFFFFFE;

  //dprintf("Pc = 0x%X",ac_pc);
}

//------------------------------------------------------
inline void BIC(int rd, int rn, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2;
  
  dprintf("Instruction: BIC\n");
  RN2.entire = RB.read(rn);
  RD2.entire = RN2.entire & ~dpi_shiftop.entire;
  dprintf("Operands:\nRn=0x%X, contains 0x%lX\nShiftOp contains 0x%lX\nDestination: Rd=0x%X\n", rn,RN2.entire,dpi_shiftop.entire,rd);
  RB.write(rd,RD2.entire);

  if ((s == 1)&&(rd == PC)) {
    printf("Unpredictable BIC instruction result\n");
    return;
  } else {
    if (s == 1) {
      flags.N = getBit(RD2.entire,31);
      flags.Z = ((RD2.entire == 0) ? true : false);
      flags.C = dpi_shiftopcarry;
      // nothing happens with flags.V 
    }
  }   
  dprintf("Results: 0x%lX\nFlags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n", RD2.entire,flags.N,flags.Z,flags.C,flags.V);
  dprintf(" *  R%d <= 0x%08X (%d)\n", rd, RD2.entire, RD2.entire); 
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void CDP(){
  dprintf("Instruction: CDP\n");
  fprintf(stderr,"Warning: CDP is not implemented in this model.\n");
}

//------------------------------------------------------
inline void CLZ(int rd, int rm,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RM2;
  int i;

  dprintf("Instruction: CLZ\n");

  // Special cases
  if((rd == PC)||(rm == PC)) {
    printf("Unpredictable CLZ instruction result\n");
    return;
  }

  RM2.entire = RB.read(rm);

  if(RM2.entire == 0) RD2.entire = 32;
  else {
    i = 31;
    while((i>=0)&&(!isBitSet(RM2.entire,i))) i--;
    RD2.entire = 31 - i;
  }

  dprintf("Results: 0x%lX\n", RD2.entire);
  RB.write(rd, RD2.entire);
  dprintf(" *  R%d <= 0x%08X (%d)\n", rd, RD2.entire, RD2.entire); 
    
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void CMN(int rn,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RN2, alu_out;
  r64bit_t soma;

  dprintf("Instruction: CMN\n");
  RN2.entire = RB.read(rn);
  dprintf("Operands:\nRn=0x%X, contains 0x%lX\nShiftOp contains 0x%lX\n", rn,RN2.entire,dpi_shiftop.entire);
  soma.hilo = ((unsigned long long)(unsigned long)RN2.entire + (unsigned long long)(unsigned long)dpi_shiftop.entire);
  alu_out.entire = soma.reg[0];

  flags.N = getBit(alu_out.entire,31);
  flags.Z = ((alu_out.entire == 0) ? true : false);
  flags.C = ((soma.reg[1] != 0) ? true : false);
  flags.V = (((getBit(RN2.entire,31) && getBit(dpi_shiftop.entire,31) && (!getBit(alu_out.entire,31))) ||
	      ((!getBit(RN2.entire,31)) && (!getBit(dpi_shiftop.entire,31)) && getBit(alu_out.entire,31))) ? true : false);

  dprintf("Results: 0x%lX\nFlags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n", alu_out.entire,flags.N,flags.Z,flags.C,flags.V);    
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void CMP(int rn,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RN2, alu_out, neg_shiftop;
  r64bit_t result;

  dprintf("Instruction: CMP\n");
  RN2.entire = RB.read(rn);
  dprintf("Operands:\nRn=0x%X, contains 0x%lX\nShiftOp contains 0x%lX\n", rn,RN2.entire,dpi_shiftop.entire);
  neg_shiftop.entire = - dpi_shiftop.entire;
  result.hilo = ((unsigned long long)(unsigned long)RN2.entire + (unsigned long long)(unsigned long) neg_shiftop.entire);
  alu_out.entire = result.reg[0];

  flags.N = getBit(alu_out.entire,31);
  flags.Z = ((alu_out.entire == 0) ? true : false);
  flags.C = !(((unsigned int) dpi_shiftop.entire > (unsigned int) RN2.entire) ? true : false);
  flags.V = (((getBit(RN2.entire,31) && getBit(neg_shiftop.entire,31) && (!getBit(alu_out.entire,31))) ||
	      ((!getBit(RN2.entire,31)) && (!getBit(neg_shiftop.entire,31)) && getBit(alu_out.entire,31))) ? true : false);
  
  dprintf("Results: 0x%lX\nFlags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n", alu_out.entire,flags.N,flags.Z,flags.C,flags.V);     
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void EOR(int rd, int rn, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2;
  
  dprintf("Instruction: EOR\n");
  RN2.entire = RB.read(rn);
  dprintf("Operands:\nRn=0x%X, contains 0x%lX\nShiftOp contains 0x%lX\nDestination: Rd=0x%X\n", rn,RN2.entire,dpi_shiftop.entire,rd);
  RD2.entire = RN2.entire ^ dpi_shiftop.entire;
  RB.write(rd, RD2.entire);

  if ((s == 1)&&(rd == PC)) {
    printf("Unpredictable EOR instruction result\n");
    return;
  } else {
    if (s == 1) {
      flags.N = getBit(RD2.entire, 31);
      flags.Z = ((RD2.entire == 0) ? true : false);
      flags.C = dpi_shiftopcarry;
      // nothing happens with flags.V 
    }
  }   
  dprintf("Results: 0x%lX\nFlags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n", RD2.entire,flags.N,flags.Z,flags.C,flags.V); 
  dprintf(" *  R%d <= 0x%08X (%d)\n", rd, RD2.entire, RD2.entire); 
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void LDC(){
  dprintf("Instruction: LDC\n");
  fprintf(stderr,"Warning: LDC instruction is not implemented in this model.\n");
}

//------------------------------------------------------
inline void LDM(int rlist, bool r,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  // todo special cases

  int i;
  long value;

  dprintf("Instruction: LDM\n");
  ls_address = lsm_startaddress;
  dprintf("Initial address: 0x%lX\n",ls_address.entire);
  for(i=0;i<15;i++){
    if(isBitSet(rlist,i)) {
      RB.write(i,MEM.read(ls_address.entire));
      ls_address.entire += 4;
      dprintf("Loaded register: 0x%X; Value: 0x%X; Next address: 0x%lX\n", i,RB.read(i),ls_address.entire);
    }
  }
    
  if((isBitSet(rlist,PC))) { // LDM(1)
    value = MEM.read(ls_address.entire);
    RB.write(PC,value & 0xFFFFFFFE);
    ls_address.entire += 4;
    dprintf("Loaded register: PC; Next address: 0x%lX\n", ls_address.entire);
  }
    
  // LDM(2) similar to LDM(1), except for the above "if"
  // LDM(3) unpredictable in user mode

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void LDR(int rd, int rn,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  long value;
  reg_t tmp;
  int addr10;

  dprintf("Instruction: LDR\n");
  addr10 = (unsigned int) ls_address.entire & 0x00000003;
  ls_address.entire &= 0xFFFFFFFC;

  // Special cases
  // TODO: Verify coprocessor cases (alignment)
      
  switch(addr10) {
  case 0:
    dprintf("Address mode(addr10): 0x%X -> 1st byte\n",addr10);
    value = MEM.read(ls_address.entire);
    break;
  case 1:
    dprintf("Address mode(addr10): 0x%X -> 2nd byte\n",addr10);
    tmp.entire = MEM.read(ls_address.entire);
    value = (RotateRight(8,tmp)).entire;
    break;
  case 2:
    dprintf("Address mode(addr10): 0x%X -> 3rd byte\n",addr10);
    tmp.entire = MEM.read(ls_address.entire);
    value = (RotateRight(16,tmp)).entire;
    break;
  default:
    dprintf("Address mode(addr10): 0x%X -> 4th byte\n",addr10);
    tmp.entire = MEM.read(ls_address.entire);
    value = (RotateRight(24,tmp)).entire;
  }
    
  dprintf("Value fetched from memory: 0x%lX\n",value);
  if(rd == PC) {
    RB.write(PC,(value & 0xFFFFFFFE));
    flags.T = isBitSet(value,0);
    dprintf(" *  PC <= 0x%08X\n", value & 0xFFFFFFFE);
  }
  else 
    {
      RB.write(rd,value);
      dprintf(" *  R%d <= 0x%08X\n", rd, value);
    }

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void LDRB(int rd, int rn,
          ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
          ac_reg<unsigned>& ac_pc, ac_memory& MEM) {
  unsigned char value;

  dprintf("Instruction: LDRB\n");

  // Special cases

  value = (unsigned char) MEM.read_byte(ls_address.entire);
  
  dprintf("Byte: 0x%X\n", value);
  RB.write(rd, ((unsigned long)value));

  dprintf(" *  R%d <= 0x%02X\n", rd, value);

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void LDRBT(int rd, int rn,
           ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
           ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  unsigned char value;

  dprintf("Instruction: LDRBT\n");

  // Special cases

  value = (unsigned char) MEM.read_byte(ls_address.entire);
  
  dprintf("Byte: 0x%X\n",MEM.read_byte(ls_address.entire));
  RB.write(rd, ((unsigned long)MEM.read_byte(ls_address.entire)));

  dprintf(" *  R%d <= 0x%02X\n", rd, value);

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void LDRD(int rd, int rn,
          ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
          ac_reg<unsigned>& ac_pc, ac_memory& MEM) {
  unsigned long value1, value2;

  dprintf("Instruction: LDRD\n");

  value1 = (unsigned long) MEM.read_byte(ls_address.entire);
  value2 = (unsigned long) MEM.read_byte(ls_address.entire+4);

  // Special cases
  // Registrador destino deve ser par
  if(isBitSet(rd,0)){
    printf("Undefined LDRD instruction result (Rd must be even)\n");
    return;
  }
  // Verificar alinhamento do doubleword
  if((rd == LR)||(ls_address.entire & 0x00000007)){
    printf("Unpredictable LDRD instruction result (Address is not doubleword aligned) @ 0x%08X\n", RB.read(PC)-4);
    return;
  }

  //FIXME: Verify if writeback receives +4 from address
  RB.write(rd, value1);
  RB.write(rd+1, value2);

  dprintf(" *  R%d <= 0x%08X\n *  R%d <= 0x%08X\n (little) value = 0x%08X%08X\n (big) value = 0x%08X08X\n", rd, value1, rd+1, value2, value2, value1, value1, value2);

  ac_pc = RB.read(PC);
}
//------------------------------------------------------
inline void LDRH(int rd, int rn,
          ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
          ac_reg<unsigned>& ac_pc, ac_memory& MEM) {
  unsigned long value;

  dprintf("Instruction: LDRH\n");

  // Special cases
  // verify coprocessor alignment
  // verify halfword alignment
  if(isBitSet(ls_address.entire,0)){
    printf("Unpredictable LDRH instruction result (Address is not Halfword Aligned)\n");
    return;
  }
  value = (unsigned long) MEM.read(ls_address.entire);
  value &= 0xFFFF; /* Zero extends halfword value 
		      BUG: Model must be little endian in order to the code work  */

  RB.write(rd, value);

  dprintf(" *  R%d <= 0x%04X\n", rd, value); 

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void LDRSB(int rd, int rn,
           ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
           ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  unsigned long data;

  dprintf("Instruction: LDRSB\n");
    
  // Special cases
  
  data = ((unsigned long)MEM.read_byte(ls_address.entire));
  data = SignExtend(data,8);

  RB.write(rd, SignExtend(data,8));

  dprintf(" *  R%d <= 0x%08X\n", rd, data); 
 
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void LDRSH(int rd, int rn,
           ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
           ac_reg<unsigned>& ac_pc, ac_memory& MEM){

  unsigned long data;

  dprintf("Instruction: LDRSH\n");
    
  // Special cases
  // verificar alinhamento do halfword
  if(isBitSet(ls_address.entire, 0)) {
    printf("Unpredictable LDRSH instruction result (Address is not halfword aligned)\n");
    return;
  }
  // Verify coprocessor alignment

  data = ((unsigned long) MEM.read(ls_address.entire));
  data &= 0xFFFF; /* Extracts halfword 
		     BUG: Model must be little endian */
  data = SignExtend(data,16);
  RB.write(rd, data);

  dprintf(" *  R%d <= 0x%08X\n", rd, data); 
    
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void LDRT(int rd, int rn,
          ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
          ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  int addr10;
  reg_t tmp;
  unsigned long value;

  dprintf("Instruction: LDRT\n");

  addr10 = (int) ls_address.entire & 0x00000003;
  ls_address.entire &= 0xFFFFFFFC;
    
  // Special cases
  // Verify coprocessor alignment
    
  switch(addr10) {
  case 0:
    value = MEM.read(ls_address.entire);
    RB.write(rd, value);
    break;
  case 1:
    tmp.entire = MEM.read(ls_address.entire);
    value = RotateRight(8,tmp).entire;
    RB.write(rd, value);
    break;
  case 2:
    tmp.entire = MEM.read(ls_address.entire);
    value = RotateRight(16,tmp).entire;
    RB.write(rd, value);
    break;
  default:
    tmp.entire = MEM.read(ls_address.entire);
    value = RotateRight(24, tmp).entire;
    RB.write(rd, value);
  }

  dprintf(" *  R%d <= 0x%08X\n", rd, value); 

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void MCR(){
  dprintf("Instruction: MCR\n");
  fprintf(stderr, "Warning: MCR instruction is not implemented in this model.\n");
}

//------------------------------------------------------
inline void MLA(int rd, int rn, int rm, int rs, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2, RM2, RS2;
  RN2.entire = RB.read(rn);
  RM2.entire = RB.read(rm);
  RS2.entire = RB.read(rs);

  dprintf("Instruction: MLA\n");
  dprintf("Operands:\nrm=0x%X, contains 0x%lX\nrs=0x%X, contains 0x%lX\nrn=0x%X, contains 0x%lX\nDestination: Rd=0x%X\n", rm,RM2.entire,rs,RS2.entire,rn,RN2.entire,rd);

  // Special cases
  if((rd == PC)||(rm == PC)||(rs == PC)||(rn == PC)||(rd == rm)) {
    fprintf(stderr, "Unpredictable MLA instruction result\n");
    return;    
  }

  RD2.entire = (long)((RM2.entire * RS2.entire) + RN2.entire);
  if(s == 1) {
    flags.N = getBit(RD2.entire,31);
    flags.Z = ((RD2.entire == 0) ? true : false);
    // nothing happens with flags.C and flags.V
  }
  dprintf("Flags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n",flags.N,flags.Z,flags.C,flags.V);
  RB.write(rd,RD2.entire);

  dprintf(" *  R%d <= 0x%08X (%d)\n", rd, RD2.entire, RD2.entire); 
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void MOV(int rd, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {
  
  dprintf("Instruction: MOV\n");
  dprintf("Operands:\nShiftOp contains 0x%lX\nDestination: Rd=0x%X\n",dpi_shiftop.entire,rd);
  RB.write(rd, dpi_shiftop.entire);

  if ((s == 1)&&(rd == PC)) {
    //  fprintf(stderr, "Unpredictable MOV instruction result\n");
    //SPSR must be copied to CPSR
  } 

  if (s == 1) {
    flags.N = getBit(dpi_shiftop.entire, 31);
    flags.Z = ((dpi_shiftop.entire == 0) ? true : false);
    flags.C = dpi_shiftopcarry;
    // nothing happens with flags.V 
  }
     
  dprintf("Flags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n",flags.N,flags.Z,flags.C,flags.V);
  dprintf(" *  R%d <= 0x%08X (%d)\n", rd, dpi_shiftop.entire, dpi_shiftop.entire); 
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void MRC(){
  dprintf("Instruction: MRC\n");
  fprintf(stderr, "Warning: MRC instruction is not implemented in this model.\n");
}

//------------------------------------------------------
inline void MRS(int rd, bool r, int zero3, int subop2, int func2, int subop1, int rm, int field,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t CPSR;

  dprintf("Instruction: MRS\n");

  // Special cases
  if((rd == PC)||((zero3 != 0)&&(subop2 != 0)&&(func2 != 0)&&(subop1 != 0)&&(rm != 0))||
     (field != 15)||(r == 1)) {
    printf("Unpredictable MRS instruction result\n");
    return;
  }

  CPSR = CPSRBuild();
  RB.write(rd,CPSR.entire);

  dprintf(" *  R%d <= 0x%08X\n", rd, CPSR.entire); 

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void MUL(int rd, int rm, int rs, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RM2, RS2;
  RM2.entire = RB.read(rm);
  RS2.entire = RB.read(rs);

  dprintf("Instruction: MUL\n");
  dprintf("Operands:\nrm=0x%X, contains 0x%lX\nrs=0x%X, contains 0x%lX\nDestination: Rd=0x%X\n",rm,RM2.entire,rs,RS2.entire,rd);

  // Special cases
  if((rd == PC)||(rm == PC)||(rs == PC)||(rd == rm)) {
    printf("Unpredictable MUL instruction result\n");
    return;    
  }

  RD2.entire = (long)(RM2.entire * RS2.entire);
  if(s == 1) {
    flags.N = getBit(RD2.entire, 31);
    flags.Z = ((RD2.entire == 0) ? true : false);
    // nothing happens with flags.C and flags.V
  }
  dprintf("Flags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n",flags.N,flags.Z,flags.C,flags.V);
  RB.write(rd, RD2.entire);

  dprintf(" *  R%d <= 0x%08X (%d)\n", rd, RD2.entire, RD2.entire); 
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void MVN(int rd, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  dprintf("Instruction: MVN\n");
  dprintf("Operands:\nShiftOp contains 0x%lX\nDestination: Rd=0x%X\n",dpi_shiftop.entire,rd);
  RB.write(rd,~dpi_shiftop.entire);

  if ((s == 1)&&(rd == PC)) {
    printf("Unpredictable MVN instruction result\n");
    return;
  } else {
    if (s == 1) {
      flags.N = getBit(~dpi_shiftop.entire,31);
      flags.Z = ((~dpi_shiftop.entire == 0) ? true : false);
      flags.C = dpi_shiftopcarry;
      // nothing happens with flags.V 
    }
  }   
  dprintf("Flags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n",flags.N,flags.Z,flags.C,flags.V);
  dprintf(" *  R%d <= 0x%08X (%d)\n", rd, ~dpi_shiftop.entire, ~dpi_shiftop.entire); 
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void ORR(int rd, int rn, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2;
  
  dprintf("Instruction: ORR\n");
  RN2.entire = RB.read(rn);
  dprintf("Operands:\nRn=0x%X, contains 0x%lX\nShiftOp contains 0x%lX\nDestination: Rd=0x%X\n", rn,RN2.entire,dpi_shiftop.entire,rd);
  RD2.entire = RN2.entire | dpi_shiftop.entire;
  RB.write(rd,RD2.entire);

  if ((s == 1)&&(rd == PC)) {
    printf("Unpredictable ORR instruction result\n");
    return;
  } else {
    if (s == 1) {
      flags.N = getBit(RD2.entire,31);
      flags.Z = ((RD2.entire == 0) ? true : false);
      flags.C = dpi_shiftopcarry;
      // nothing happens with flags.V 
    }
  }  
  dprintf("Results: 0x%lX\nFlags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n",RD2.entire,flags.N,flags.Z,flags.C,flags.V);  
  dprintf(" *  R%d <= 0x%08X (%d)\n", rd, RD2.entire, RD2.entire); 
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void RSB(int rd, int rn, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2, neg_RN2;
  r64bit_t result;

  dprintf("Instruction: RSB\n");
  RN2.entire = RB.read(rn);
  if(rn == PC) RN2.entire += 4;
  dprintf("Operands:\nRn=0x%X, contains 0x%lX\nShiftOp contains 0x%lX\nDestination: Rd=0x%X\n", rn,RN2.entire,dpi_shiftop.entire,rd);
  neg_RN2.entire = - RN2.entire;
  result.hilo = ((unsigned long long)(unsigned long)dpi_shiftop.entire + (unsigned long long)(unsigned long)neg_RN2.entire);
  RD2.entire = result.reg[0];
  RB.write(rd, RD2.entire);
  if ((s == 1) && (rd == PC)) {
    printf("Unpredictable RSB instruction result\n");
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
  dprintf("Results: 0x%lX\nFlags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n",RD2.entire,flags.N,flags.Z,flags.C,flags.V);
  dprintf(" *  R%d <= 0x%08X (%d)\n", rd, RD2.entire, RD2.entire); 
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void RSC(int rd, int rn, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2, neg_RN2;
  r64bit_t result;

  dprintf("Instruction: RSC\n");
  RN2.entire = RB.read(rn);
  if(rn == PC) RN2.entire += 4;
  dprintf("Operands:\nRn=0x%X, contains 0x%lX\nShiftOp contains 0x%lX\nC=0x%X\nDestination: Rd=0x%X\n", rn,RN2.entire,dpi_shiftop.entire,flags.C,rd);
  neg_RN2.entire = - RN2.entire;
  if (!flags.C) neg_RN2.entire--;
  result.hilo = ((unsigned long long)(unsigned long)dpi_shiftop.entire + (unsigned long long)(unsigned long)neg_RN2.entire);
  RD2.entire = result.reg[0];

  RB.write(rd, RD2.entire);
  if ((s == 1)&&(rd == PC)) {
    printf("Unpredictable RSC instruction result\n");
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
  dprintf("Results: 0x%lX\nFlags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n",RD2.entire,flags.N,flags.Z,flags.C,flags.V);
  dprintf(" *  R%d <= 0x%08X (%d)\n", rd, RD2.entire, RD2.entire); 
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void SBC(int rd, int rn, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2, neg_shiftop;
  r64bit_t result;

  dprintf("Instruction: SBC\n");
  RN2.entire = RB.read(rn);
  if(rn == PC) RN2.entire += 4;
  dprintf("Operands:\nRn=0x%X, contains 0x%lX\nShiftOp contains 0x%lX\nC=0x%X\nDestination: Rd=0x%X\n", rn,RN2.entire,dpi_shiftop.entire,flags.C,rd);
  neg_shiftop.entire = - dpi_shiftop.entire; 
  if (!flags.C) neg_shiftop.entire--;
  result.hilo = ((unsigned long long)(unsigned long)RN2.entire + (unsigned long long)(unsigned long)neg_shiftop.entire);
  RD2.entire = result.reg[0];
  RB.write(rd, RD2.entire);
  if ((s == 1)&&(rd == PC)) {
    printf("Unpredictable SBC instruction result\n");
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
  dprintf("Results: 0x%lX\nFlags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n",RD2.entire,flags.N,flags.Z,flags.C,flags.V);
  dprintf(" *  R%d <= 0x%08X (%d)\n", rd, RD2.entire, RD2.entire); 
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void SMLAL(int rdhi, int rdlo, int rm, int rs, bool s,
           ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
           ac_reg<unsigned>& ac_pc) {

  r64bit_t result, acc;
  reg_t RM2, RS2;

  RM2.entire = RB.read(rm);
  RS2.entire = RB.read(rs);
  acc.reg[0] = RB.read(rdlo);
  acc.reg[1] = RB.read(rdhi);

  dprintf("Instruction: SMLAL\n");
  dprintf("Operands:\nrm=0x%X, contains 0x%lX\nrs=0x%X, contains 0x%lX\nAdd multiply result to %lld\nDestination(Hi): Rdhi=0x%X, Rdlo=0x%X\n", rm,RM2.entire,rs,RS2.entire,acc.hilo,rdhi,rdlo);

  // Special cases
  if((rdhi == PC)||(rdlo == PC)||(rm == PC)||(rs == PC)||(rdhi == rdlo)||(rdhi == rm)||(rdlo == rm)) {
    printf("Unpredictable SMLAL instruction result\n");
    return;  
  }

  result.hilo = ((long long)((long long)RM2.entire * (long long)RS2.entire)) + (long long)acc.hilo;
  RB.write(rdhi,result.reg[1]);
  RB.write(rdlo,result.reg[0]);
  if(s == 1){
    flags.N = getBit(result.reg[1],31);
    flags.Z = ((result.hilo == 0) ? true : false);
    // nothing happens with flags.C and flags.V
  }
  dprintf("Results: %lld\nFlags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n",result.hilo,flags.N,flags.Z,flags.C,flags.V);
  dprintf(" *  R%d(high) R%d(low) <= 0x%08X%08X (%d)\n", rdhi, rdlo, result.reg[1], result.reg[0], result.reg[0]); 
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void SMULL(int rdhi, int rdlo, int rm, int rs, bool s,
           ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
           ac_reg<unsigned>& ac_pc) {

  r64bit_t result;
  reg_t RM2, RS2;

  RM2.entire = RB.read(rm);
  RS2.entire = RB.read(rs);

  dprintf("Instruction: SMULL\n");
  dprintf("Operands:\nrm=0x%X, contains 0x%lX\nrs=0x%X, contains 0x%lX\nDestination(Hi): Rdhi=0x%X, Rdlo=0x%X\n", rm,RM2.entire,rs,RS2.entire,rdhi,rdlo);

  // Special cases
  if((rdhi == PC)||(rdlo == PC)||(rm == PC)||(rs == PC)||(rdhi == rdlo)||(rdhi == rm)||(rdlo == rm)) {
    printf("Unpredictable SMULL instruction result\n");
    return;  
  }

  result.hilo = (long long)((long long)RM2.entire * (long long)RS2.entire);
  RB.write(rdhi,result.reg[1]);
  RB.write(rdlo,result.reg[0]);
  if(s == 1){
    flags.N = getBit(result.reg[1],31);
    flags.Z = ((result.hilo == 0) ? true : false);
    // nothing happens with flags.C and flags.V
  }
  dprintf("Results: %lld\nFlags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n",result.hilo,flags.N,flags.Z,flags.C,flags.V);
  dprintf(" *  R%d(high) R%d(low) <= 0x%08X%08X (%d)\n", rdhi, rdlo, result.reg[1], result.reg[0], result.reg[0]);
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void STC(){
  dprintf("Instruction: STC\n");
  fprintf(stderr,"Warning: STC instruction is not implemented in this model.\n");
}

//------------------------------------------------------
inline void STM(int rlist,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  // todo special cases

  int i;

  dprintf("Instruction: STM\n");
  ls_address = lsm_startaddress;
  for(i=0;i<16;i++){
    if(isBitSet(rlist,i)) {
      MEM.write(ls_address.entire,RB.read(i));
      ls_address.entire += 4;
      dprintf("Stored register: 0x%X; value: 0x%X; address: 0x%lX\n",i,RB.read(i),ls_address.entire-4);
    }
  }

  // STM(2) unpredictable in User Mode

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void STR(int rd, int rn,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  dprintf("Instruction: STR\n");

  // Special cases
  // verify coprocessor alignment
  
  dprintf("Write address: 0x%lX\nContents rd: 0x%lX\n",ls_address.entire,RB.read(rd));
  MEM.write(ls_address.entire, RB.read(rd));

  dprintf(" *  MEM[0x%08X] <= 0x%08X\n", ls_address.entire, RB.read(rd)); 

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void STRB(int rd, int rn,
          ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
          ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  reg_t RD2;

  dprintf("Instruction: STRB\n");

  // Special cases

  RD2.entire = RB.read(rd);
  dprintf("Write address: 0x%lX\nContents rd: 0x%lX\n",ls_address.entire,RD2.byte[0]);
  MEM.write_byte(ls_address.entire, RD2.byte[0]);

  dprintf(" *  MEM[0x%08X] <= 0x%02X\n", ls_address.entire, RD2.byte[0]); 

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void STRBT(int rd, int rn,
           ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
           ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  reg_t RD2;

  dprintf("Instruction: STRBT\n");

  // Special cases
  
  RD2.entire = RB.read(rd);
  dprintf("Write address: 0x%lX\nContents rd: 0x%lX\n",ls_address.entire,RD2.byte[0]);
  MEM.write_byte(ls_address.entire, RD2.byte[0]);

  dprintf(" *  MEM[0x%08X] <= 0x%02X\n", ls_address.entire, RD2.byte[0]); 

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void STRD(int rd, int rn,
          ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
          ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  dprintf("Instruction: STRD\n");

  // Special cases
  // Destination register must be even
  if(isBitSet(rd,0)){
    printf("Undefined STRD instruction result (Rd must be even)\n");
    return;
  }
  // Check doubleword alignment
  if((rd == LR)||(ls_address.entire & 0x00000007)){
    printf("Unpredictable STRD instruction result (Address is not doubleword aligned)\n");
    return;
  }

  //FIXME: Check if writeback receives +4 from second address
  MEM.write(ls_address.entire,RB.read(rd));
  MEM.write(ls_address.entire+4,RB.read(rd+1));

  dprintf(" *  MEM[0x%08X], MEM[0x%08X] <= 0x%08X %08X\n", ls_address.entire, ls_address.entire+4, RB.read(rd+1), RB.read(rd)); 

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void STRH(int rd, int rn,
          ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
          ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  short int data;

  dprintf("Instruction: STRH\n");
    
  // Special cases
  // verify coprocessor alignment
  // verify halfword alignment
  if(isBitSet(ls_address.entire,0)){
    printf("Unpredictable STRH instruction result (Address is not halfword aligned)\n");
    return;
  }

  data = (short int) RB.read(rd) & 0x0000FFFF;
  MEM.write_half(ls_address.entire, (short int)data);

  dprintf(" *  MEM[0x%08X] <= 0x%04X\n", ls_address.entire, data); 
    
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void STRT(int rd, int rn,
          ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
          ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  dprintf("Instruction: STRT\n");

  // Special cases
  // verificar caso do coprocessador (alinhamento)
  
  MEM.write(ls_address.entire, RB.read(rd));

  dprintf(" *  MEM[0x%08X] <= 0x%08X\n", ls_address.entire, RB.read(rd)); 

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void SUB(int rd, int rn, bool s,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2, neg_shiftop;
  r64bit_t result;

  dprintf("Instruction: SUB\n");
  RN2.entire = RB.read(rn);
  if(rn == PC) RN2.entire += 4;
  dprintf("Operands:\nRn=0x%X, contains 0x%lX\nShiftOp contains 0x%lX\nDestination: Rd=0x%X\n", rn,RN2.entire,dpi_shiftop.entire,rd);
  neg_shiftop.entire = - dpi_shiftop.entire;
  result.hilo = ((unsigned long long)(unsigned long)RN2.entire + (unsigned long long)(unsigned long)neg_shiftop.entire);
  RD2.entire = result.reg[0];
  RB.write(rd, RD2.entire);
  if ((s == 1)&&(rd == PC)) {
    printf("Unpredictable SUB instruction result\n");
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
  dprintf("Results: 0x%lX\nFlags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n",RD2.entire,flags.N,flags.Z,flags.C,flags.V);
  dprintf(" *  R%d <= 0x%08X (%d)\n", rd, RD2.entire, RD2.entire); 
  ac_pc = RB.read(PC);
 
}

//------------------------------------------------------
inline void SWP(int rd, int rn, int rm,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  reg_t RN2, RM2, rtmp;
  long tmp;
  int rn10;

  dprintf("Instruction: SWP\n");

  // Special cases
  // verify coprocessor alignment
  if((rd == PC)||(rm == PC)||(rn == PC)||(rm == rn)||(rn == rd)){
    printf("Unpredictable SWP instruction result\n");
    return;
  }

  RN2.entire = RB.read(rn);
  RM2.entire = RB.read(rm);
  dprintf("rn=0x%X, contains 0x%lX\nrm=0x%X, contains 0x%lX\n", rn,RN2.entire,rm,RM2.entire);
  rn10 = (int) RN2.entire & 0x00000003;

  switch(rn10) {
  case 0:
    dprintf("Mode 0\n");
    tmp = MEM.read(RN2.entire);
    break;
  case 1:
    dprintf("Mode 1\n");
    rtmp.entire = MEM.read(RN2.entire);
    tmp = (RotateRight(8,rtmp)).entire;
    break;
  case 2:
    dprintf("Mode 2\n");
    rtmp.entire = MEM.read(RN2.entire);
    tmp = (RotateRight(16,rtmp)).entire;
    break;
  default:
    dprintf("Mode 3\n");
    rtmp.entire = MEM.read(RN2.entire);
    tmp = (RotateRight(24,rtmp)).entire;
  }
    
  dprintf("tmp contains 0x%lX, rtmp contains 0x%lX\n", tmp,rtmp.entire);
  MEM.write(RN2.entire,RM2.entire);
  RB.write(rd,tmp);

  dprintf(" *  MEM[0x%08X] <= 0x%08X (%d)\n", RN2.entire, RM2.entire, RM2.entire); 
  dprintf(" *  R%d <= 0x%08X (%d)\n", rd, tmp, tmp); 

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void SWPB(int rd, int rn, int rm,
          ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
          ac_reg<unsigned>& ac_pc, ac_memory& MEM) {

  long tmp;
  reg_t RM2, RN2;

  dprintf("Instruction: SWPB\n");

  // Special cases
  if((rd == PC)||(rm == PC)||(rn == PC)||(rm == rn)||(rn == rd)){
    printf("Unpredictable SWPB instruction result\n");
    return;
  }

  RM2.entire = RB.read(rm);
  RN2.entire = RB.read(rn);
  dprintf("rn=0x%X, contains 0x%lX\nrm=0x%X, contains 0x%lX\n", rn,RN2.entire,rm,RM2.entire);

  tmp = (unsigned long)MEM.read_byte(RN2.entire);
  dprintf("tmp contains 0x%lX\n",tmp);
  MEM.write_byte(RN2.entire,RM2.byte[0]);
  RB.write(rd,tmp);

  dprintf(" *  MEM[0x%08X] <= 0x%02X (%d)\n", RN2.entire, RM2.byte[0], RM2.byte[0]); 
  dprintf(" *  R%d <= 0x%02X (%d)\n", rd, tmp, tmp); 

  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void TEQ(int rn,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RN2, alu_out;

  dprintf("Instruction: TEQ\n");
  RN2.entire = RB.read(rn);
  dprintf("Operands:\nRn=0x%X, contains 0x%lX\nShiftOp contains 0x%lX\n", rn,RN2.entire,dpi_shiftop.entire);
  alu_out.entire = RN2.entire ^ dpi_shiftop.entire;

  flags.N = getBit(alu_out.entire,31);
  flags.Z = ((alu_out.entire == 0) ? true : false);
  flags.C = dpi_shiftopcarry;
  // nothing happens with flags.V
    
  dprintf("Flags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n",flags.N,flags.Z,flags.C,flags.V);  
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void TST(int rn,
         ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
         ac_reg<unsigned>& ac_pc) {

  reg_t RN2, alu_out;

  dprintf("Instruction: TST\n");
  RN2.entire = RB.read(rn);
  dprintf("Operands:\nRn=0x%X, contains 0x%lX\nShiftOp contains 0x%lX\n", rn,RN2.entire,dpi_shiftop.entire);
  alu_out.entire = RN2.entire & dpi_shiftop.entire;

  flags.N = getBit(alu_out.entire, 31);
  flags.Z = ((alu_out.entire == 0) ? true : false);
  flags.C = dpi_shiftopcarry;
  // nothing happens with flags.V
    
  dprintf("Flags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n",flags.N,flags.Z,flags.C,flags.V); 
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void UMLAL(int rdhi, int rdlo, int rm, int rs, bool s,
           ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
           ac_reg<unsigned>& ac_pc) {

  r64bit_t result, acc;
  reg_t RM2, RS2;

  RM2.entire = RB.read(rm);
  RS2.entire = RB.read(rs);
  acc.reg[0] = RB.read(rdlo);
  acc.reg[1] = RB.read(rdhi);

  dprintf("Instruction: UMLAL\n");
  dprintf("Operands:\nrm=0x%X, contains 0x%lX\nrs=0x%X, contains 0x%lX\nAdd multiply result to %lld\nDestination(Hi): Rdhi=0x%X, Rdlo=0x%X\n", rm,RM2.entire,rs,RS2.entire,acc.hilo,rdhi,rdlo);

  // Special cases
  if((rdhi == PC)||(rdlo == PC)||(rm == PC)||(rs == PC)||(rdhi == rdlo)||(rdhi == rm)||(rdlo == rm)) {
    printf("Unpredictable UMLAL instruction result\n");
    return;  
  }

  result.hilo = ((unsigned long long)(unsigned long)RM2.entire * (unsigned long long)(unsigned long)RS2.entire) + acc.hilo;
  RB.write(rdhi,result.reg[1]);
  RB.write(rdlo,result.reg[0]);
  if(s == 1){
    flags.N = getBit(result.reg[1],31);
    flags.Z = ((result.hilo == 0) ? true : false);
    // nothing happens with flags.C and flags.V
  }
  dprintf("Results: %lld\nFlags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n",result.hilo,flags.N,flags.Z,flags.C,flags.V);

  dprintf(" *  R%d(high) R%d(low) <= 0x%08X%08X (%d)\n", rdhi, rdlo, result.reg[1], result.reg[0], result.reg[0]); 
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void UMULL(int rdhi, int rdlo, int rm, int rs, bool s,
           ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
           ac_reg<unsigned>& ac_pc) {

  
  r64bit_t result;
  reg_t RM2, RS2;
  
  RM2.entire = RB.read(rm);
  RS2.entire = RB.read(rs);
  
  dprintf("Instruction: UMULL\n");
  dprintf("Operands:\nrm=0x%X, contains 0x%lX\nrs=0x%X, contains 0x%lX\nDestination(Hi): Rdhi=0x%X, Rdlo=0x%X\n", rm,RM2.entire,rs,RS2.entire,rdhi,rdlo);

  // Special cases
  if((rdhi == PC)||(rdlo == PC)||(rm == PC)||(rs == PC)||(rdhi == rdlo)||(rdhi == rm)||(rdlo == rm)) {
    printf("Unpredictable UMULL instruction result\n");
    return;  
  }
  
  result.hilo = ((unsigned long long)(unsigned long)RM2.entire * (unsigned long long)(unsigned long)RS2.entire);
  RB.write(rdhi,result.reg[1]);
  RB.write(rdlo,result.reg[0]);
  if(s == 1){
    flags.N = getBit(result.reg[1],31);
    flags.Z = ((result.hilo == 0) ? true : false);
    // nothing happens with flags.C and flags.V
  }
  dprintf("Results: %lld\nFlags: N=0x%X, Z=0x%X, C=0x%X, V=0x%X\n",result.hilo,flags.N,flags.Z,flags.C,flags.V);
  dprintf(" *  R%d(high) R%d(low) <= 0x%08X%08X (%d)\n", rdhi, rdlo, result.reg[1], result.reg[0], result.reg[0]); 
  ac_pc = RB.read(PC);
}

//------------------------------------------------------
inline void DSMLA(int rd, int rn,
           ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
           ac_reg<unsigned>& ac_pc) {

  reg_t RD2, RN2;

  RN2.entire = RB.read(rn);

  dprintf("Instruction: SMLA<y><x>\n");
  dprintf("Operands:\nrn=0x%X, contains 0x%lX\nfirst operand contains 0x%lX\nsecond operand contains 0x%lX\nrd=0x%X, contains 0x%lX\n", rn, RN2.entire, OP1.entire, OP2.entire, rd, RD2.entire);
  
  RD2.entire = (long)((OP1.entire * OP2.entire) + RN2.entire);

  RB.write(rd, RD2.entire);

  dprintf(" *  R%d <= 0x%08X (%d)\n", rd, RD2.entire, RD2.entire); 

  // SET Q FLAG
}

//------------------------------------------------------
inline void DSMUL(int rd,
           ac_regbank<31, armv5e_parms::ac_word, armv5e_parms::ac_Dword>& RB,
           ac_reg<unsigned>& ac_pc) {

  reg_t RD2;

  dprintf("Instruction: SMUL<y><x>\n");
  dprintf("Operands:\nfirst operand contains 0x%lX\nsecond operand contains 0x%lX\nrd=0x%X, contains 0x%lX\n", OP1.entire, OP2.entire, rd, RD2.entire);
  
  RD2.entire = (long)(OP1.entire * OP2.entire);

  RB.write(rd, RD2.entire);

  dprintf(" *  R%d <= 0x%08X (%d)\n", rd, RD2.entire, RD2.entire); 

  // SET Q FLAG
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
  fprintf(stderr,"Warning: BLX instruction is not implemented in this model. PC=%X\n", ac_pc.read());
}

//!Instruction bx behavior method.
void ac_behavior( bx ){ BX(rm, RB, ac_pc); }

//!Instruction blx2 behavior method.
void ac_behavior( blx2 ){
  fprintf(stderr,"Warning: BLX instruction is not implemented in this model. PC=%X\n", ac_pc.read());
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
  fprintf(stderr,"Warning: BKPT instruction is not implemented in this model. PC=%X\n", ac_pc.read());
}

//!Instruction swi behavior method.
void ac_behavior( swi ){
  if (syscall.process_syscall(swinumber) == -1) {
    fprintf(stderr, "Warning: A syscall not implemented in this model was called.\n\tCaller address: 0x%X\n\tSWI number: 0x%X\t%d\n", (unsigned int)ac_pc, swinumber, swinumber);
  }
}

//!Instruction clz behavior method.
void ac_behavior( clz ){ CLZ(rd, rm, RB, ac_pc);}

//!Instruction mrs behavior method.
void ac_behavior( mrs ){ MRS(rd,r,zero3,subop2,func2,subop1,rm,fieldmask, RB, ac_pc);}

//!Instruction msr1 behavior method.
void ac_behavior( msr1 ){
  fprintf(stderr,"Warning: MSR instruction is not implemented in this model. PC=%X\n", ac_pc.read());
}

//!Instruction msr2 behavior method.
void ac_behavior( msr2 ){
  fprintf(stderr,"Warning: MSR instruction is not implemented in this model. PC=%X\n", ac_pc.read());
}

//!Instruction ldrd2 behavior method.
void ac_behavior( ldrd ){ LDRD(rd, rn, RB, ac_pc, MEM); }

//!Instruction ldrd2 behavior method.
void ac_behavior( strd ){ STRD(rd, rn, RB, ac_pc, MEM); }

//!Instruction dsmla behavior method.
void ac_behavior( dsmla ){ DSMLA(drd, drn, RB, ac_pc); }

//!Instruction dsmlal behavior method.
void ac_behavior( dsmlal ){
  fprintf(stderr,"Warning: SMLAL<y><x> instruction is not implemented in this model. PC=%X\n", ac_pc.read());
}

//!Instruction dsmul behavior method.
void ac_behavior( dsmul ){ DSMUL(drd, RB, ac_pc); }

//!Instruction dsmlaw behavior method.
void ac_behavior( dsmlaw ){
  fprintf(stderr,"Warning: SMLAW<y><x> instruction is not implemented in this model. PC=%X\n", ac_pc.read());
}

//!Instruction dsmulw behavior method.
void ac_behavior( dsmulw ){
  fprintf(stderr,"Warning: SMULW<y><x> instruction is not implemented in this model. PC=%X\n", ac_pc.read());
}

///Behaviors begin and end
void ac_behavior( begin ) { }
void ac_behavior( end ) { }
