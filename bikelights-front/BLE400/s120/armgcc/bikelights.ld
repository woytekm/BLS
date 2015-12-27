/* Linker script to configure memory regions. */
SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
/*   FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x40000  */
/*   RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x4000  */
  FLASH (rx) : ORIGIN = 0x0001d000, LENGTH = 0x23000
/*  RAM (rwx)  : ORIGIN = 0x20002000, LENGTH = 0x2000 */
  RAM (rwx)  : ORIGIN = 0x20002800, LENGTH = 0x1800
 

}

INCLUDE "gcc_nrf51_common.ld"


