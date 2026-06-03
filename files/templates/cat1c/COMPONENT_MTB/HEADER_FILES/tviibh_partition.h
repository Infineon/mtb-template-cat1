#if !defined(TVIIBH_PARTITION_H)
#define TVIIBH_PARTITION_H

#define STACK_SIZE                      0x1000
#define RAMVECTORS_ALIGNMENT            128

#define SRAM_START_RESERVE              0
#define SRAM_PRIVATE_FOR_SROM           0x800 /* 2K Private SRAM for SROM (e.g. API processing). Reserved at the beginning */

/* DEVICE RAM */
#define SRAM_BASE_ADDRESS               CY_SRAM_BASE  /* SRAM START */
#define TOTAL_RAM                       CY_SRAM_SIZE

/* DEVICE FLASH */
#define CODE_FLASH_BASE_ADDRESS         CY_FLASH_LG_SBM_BASE  /* FLASH START */
#define TOTAL_FLASH                     (CY_FLASH_LG_SBM_SIZE + CY_FLASH_SM_SBM_SIZE)
#define CODE_FLASH1_BASE_ADDRESS        CY_FLASH1_LG_SBM_BASE  /* FLASH START */
#define TOTAL_FLASH1                    (CY_FLASH1_LG_SBM_SIZE + CY_FLASH1_SM_SBM_SIZE)

/* CM0P/CM7 FLASH/RAM */
#define CM0PLUS_CODE_FLASH_RESERVE      0x00100000      /* 1024K CM0P flash size */
#define CM7_0_CODE_FLASH_RESERVE        0x00398000      /* 3680K CM7_0 flash size */
#define CM7_1_CODE_FLASH_RESERVE        0x00398000      /* 3680K CM7_1 flash size */
#define CM7_2_CODE_FLASH_RESERVE        0x00418000      /* 4192K CM7_2 flash size */
#define CM7_3_CODE_FLASH_RESERVE        0x00418000      /* 4192K CM7_3 flash size */
#define CM0PLUS_SRAM_RESERVE            0x00020000      /* 128K cm0plus */
#define CM7_0_SRAM_RESERVE              0x00080000      /* 512KB CM7_0 SRAM size, if present */
#define CM7_1_SRAM_RESERVE              0x00080000      /* 512KB CM7_1 SRAM size, if present */
#define CM7_2_SRAM_RESERVE              0x00080000      /* 512KB CM7_2 SRAM size, if present */
#define CM7_3_SRAM_RESERVE              0x00040000      /* 256KB CM7_3 SRAM size, if present */
#define NON_CACHE_SRAM_RESERVE          0x00020000      /* 128K non cached (shared memory) */

/* Validate Flash size */
#if (TOTAL_FLASH != 0x00830000) /** 16M device */
    #error "Invalid device"
#endif


/* SRAM reservations */
#define BASE_SRAM_CM0P                  (SRAM_BASE_ADDRESS + SRAM_START_RESERVE + SRAM_PRIVATE_FOR_SROM)
#define SIZE_SRAM_CM0P                  (CM0PLUS_SRAM_RESERVE - SRAM_START_RESERVE - SRAM_PRIVATE_FOR_SROM)
#define BASE_SRAM_CM7_0                 BASE_SRAM_CM0P + SIZE_SRAM_CM0P
#define SIZE_SRAM_CM7_0                 CM7_0_SRAM_RESERVE
#define BASE_SRAM_CM7_1                 (BASE_SRAM_CM7_0 + SIZE_SRAM_CM7_0)
#define SIZE_SRAM_CM7_1                 CM7_1_SRAM_RESERVE
#define BASE_SRAM_CM7_2                 (BASE_SRAM_CM7_1 + SIZE_SRAM_CM7_1)
#define SIZE_SRAM_CM7_2                 CM7_2_SRAM_RESERVE
#define BASE_SRAM_CM7_3                 (BASE_SRAM_CM7_2 + SIZE_SRAM_CM7_2)
#define SIZE_SRAM_CM7_3                 CM7_3_SRAM_RESERVE
#define BASE_SRAM_NON_CACHE             (SRAM_BASE_ADDRESS + TOTAL_RAM - NON_CACHE_SRAM_RESERVE)
#if ((SIZE_SRAM_CM0P + SRAM_START_RESERVE + SRAM_PRIVATE_FOR_SROM + + NON_CACHE_SRAM_RESERVE \
        + SIZE_SRAM_CM7_0 + SIZE_SRAM_CM7_1 + SIZE_SRAM_CM7_2 + SIZE_SRAM_CM7_3) > TOTAL_RAM)
    #error "Invalid RAM partition"
#endif

/* Code flash0 reservations */
#define BASE_CODE_FLASH_CM0P            CODE_FLASH_BASE_ADDRESS
#define SIZE_CODE_FLASH_CM0P            CM0PLUS_CODE_FLASH_RESERVE
#define BASE_CODE_FLASH_CM7_0           (BASE_CODE_FLASH_CM0P + SIZE_CODE_FLASH_CM0P)
#define SIZE_CODE_FLASH_CM7_0           CM7_0_CODE_FLASH_RESERVE
#define BASE_CODE_FLASH_CM7_1           (BASE_CODE_FLASH_CM7_0 + SIZE_CODE_FLASH_CM7_0)
#define SIZE_CODE_FLASH_CM7_1           CM7_1_CODE_FLASH_RESERVE
#if ((SIZE_CODE_FLASH_CM0P + SIZE_CODE_FLASH_CM7_0 + SIZE_CODE_FLASH_CM7_1) > TOTAL_FLASH)
    #error "Invalid Flash0 partition"
#endif
/* Code flash1 reservations */
#define BASE_CODE_FLASH_CM7_2           CODE_FLASH1_BASE_ADDRESS
#define SIZE_CODE_FLASH_CM7_2           CM7_2_CODE_FLASH_RESERVE
#define BASE_CODE_FLASH_CM7_3           (BASE_CODE_FLASH_CM7_2 + SIZE_CODE_FLASH_CM7_2)
#define SIZE_CODE_FLASH_CM7_3           CM7_3_CODE_FLASH_RESERVE
#if ((SIZE_CODE_FLASH_CM7_2 + SIZE_CODE_FLASH_CM7_3) > TOTAL_FLASH1)
    #error "Invalid Flash1 partition"
#endif

/* Partitioning validation */ 
/* Flash sector size */
#define TVIIBH_FLASHC_SECTOR_SIZE         0x8000  /* Large sector size is 32KB */
#if (CM0PLUS_CODE_FLASH_RESERVE % TVIIBH_FLASHC_SECTOR_SIZE != 0) 
#warning "The tviibh_partition.h defines CM0PLUS_CODE_FLASH_RESERVE to a value that does not land on a flash boundary.  This can potentially cause partial erasure of the beginning of CM7_0 application anytime the CM0+ application is separately flashed."
#endif
#if (CM7_0_CODE_FLASH_RESERVE % TVIIBH_FLASHC_SECTOR_SIZE != 0)
#warning "The tviibh_partition.h defines CM7_0_CODE_FLASH_RESERVE to a value that does not land on a flash boundary.  This can potentially cause partial erasure of the beginning of CM7_1 application anytime the CM7_0 application is flashed."
#endif


#endif /* TVIIBH_PARTITION_H */


/* [] END OF FILE */
