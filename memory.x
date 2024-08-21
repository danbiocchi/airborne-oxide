MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM : ORIGIN = 0x20000000, LENGTH = 128K
}

   SECTIONS
   {
     .heap (NOLOAD) :
     {
       . = ALIGN(8);
       _sheap = .;
       KEEP(*(.heap))
       . = ALIGN(8);
       _eheap = .;
     } > RAM
   }