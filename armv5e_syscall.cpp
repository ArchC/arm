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

#include "armv5e_syscall.H"

using namespace armv5e_parms;

void armv5e_syscall::get_buffer(int argn, unsigned char* buf, unsigned int size) {
  unsigned int addr = RB.read(argn);

  for (unsigned int i = 0; i<size; i++, addr++) {
    buf[i] = MEM.read_byte(addr);
  }
}

void armv5e_syscall::set_buffer(int argn, unsigned char* buf, unsigned int size) {
  unsigned int addr = RB.read(argn);

  for (unsigned int i = 0; i<size; i++, addr++) {
    MEM.write_byte(addr, buf[i]);
  }
}

void armv5e_syscall::set_buffer_noinvert(int argn, unsigned char* buf, unsigned int size) {
  unsigned int addr = RB.read(argn);

  for (unsigned int i = 0; i<size; i+=4, addr+=4) {
    MEM.write(addr, *(unsigned int *) &buf[i]);
  }
}

int armv5e_syscall::get_int(int argn) {
  return RB.read(argn);
}

void armv5e_syscall::set_int(int argn, int val) {
  RB.write(argn, val);
}

void armv5e_syscall::return_from_syscall() {
  ac_pc = RB.read(14);
}

void armv5e_syscall::set_prog_args(int argc, char **argv) {
  int i, j, base;

  unsigned int ac_argv[30];
  char ac_argstr[512];

  base = AC_RAM_END - 512;
  for (i=0, j=0; i<argc; i++) {
    int len = strlen(argv[i]) + 1;
    ac_argv[i] = base + j;
    memcpy(&ac_argstr[j], argv[i], len);
    j += len;
  }

  //Ajust %sp and write argument string
  RB.write(13, AC_RAM_END-512);
  set_buffer(13, (unsigned char*) ac_argstr, 512);

  //Ajust %sp and write string pointers
  RB.write(13, AC_RAM_END-512-120);
  set_buffer_noinvert(13, (unsigned char*) ac_argv, 120);

  //Set %o0 to the argument count
  RB.write(0, argc);

  //Set %o1 to the string pointers
  RB.write(1, AC_RAM_END-512-120);
}

