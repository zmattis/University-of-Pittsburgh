
;******************** (C) Yifeng ZHU ******************************************************************
; @file    core_cm4_constant.s
; @author  Yifeng Zhu @ UMaine
; @version V1.0.0
; @date    May-8-2015
; @note    Modifed from core_cm4.h (C) 2010 STMicroelectronics
; @brief   Assembly version of Cortex M3 core
; @note
;          This code is for the book "Embedded Systems with ARM Cortex-M3 
;          Microcontrollers in Assembly Language and C, Yifeng Zhu, 
;          ISBN-10: 0982692625.
; @attension
;          This code is provided for education purpose. The author shall not be 
;          held liable for any direct, indirect or consequential damages, for any 
;          reason whatever. More information can be found from book website: 
;          http://www.eece.maine.edu/~zhu/book
;******************************************************************************************************

;******************************************************************************************************
;               Register Abstraction
; Core Register contain:
; - Core Register
; - Core NVIC Register
; - Core SCB Register
; - Core SysTick Register
; - Core Debug Register
; - Core MPU Register
; - Core FPU Register
;******************************************************************************************************


; CMSIS_NVIC  Nested Vectored Interrupt Controller (NVIC)
 
; Structure type to access the Nested Vectored Interrupt Controller (NVIC).
 
NVIC_ISER0         EQU  0x000  ; Offset: 0x000 (R/W)  Interrupt Set Enable Register           
NVIC_ICER0         EQU  0x080  ; Offset: 0x080 (R/W)  Interrupt Clear Enable Register         
NVIC_ISPR0         EQU  0x100  ; Offset: 0x100 (R/W)  Interrupt Set Pending Register          
NVIC_ICPR0         EQU  0x180  ; Offset: 0x180 (R/W)  Interrupt Clear Pending Register        
NVIC_IABR0         EQU  0x200  ; Offset: 0x200 (R/W)  Interrupt Active bit Register           
NVIC_IP0           EQU  0x300  ; Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) 
NVIC_STIR0         EQU  0xE00  ; Offset: 0xE00 ( /W)  Software Trigger Interrupt Register     

NVIC_ISER          EQU  0x000  ; Offset: 0x000 (R/W)  Interrupt Set Enable Register           
NVIC_ICER          EQU  0x080  ; Offset: 0x080 (R/W)  Interrupt Clear Enable Register         
NVIC_ISPR          EQU  0x100  ; Offset: 0x100 (R/W)  Interrupt Set Pending Register          
NVIC_ICPR          EQU  0x180  ; Offset: 0x180 (R/W)  Interrupt Clear Pending Register        
NVIC_IABR          EQU  0x200  ; Offset: 0x200 (R/W)  Interrupt Active bit Register           
NVIC_IP            EQU  0x300  ; Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) 
NVIC_STIR          EQU  0xE00  ; Offset: 0xE00 ( /W)  Software Trigger Interrupt Register 
	
; Software Triggered Interrupt Register Definitions 
NVIC_STIR_INTID_Pos     EQU  0                                         ; STIR: INTLINESNUM Position 
NVIC_STIR_INTID_Msk     EQU  (0x1FF << NVIC_STIR_INTID_Pos)            ; STIR: INTLINESNUM Mask 


; CMSIS_core_register
; CMSIS_SCB     System Control Block (SCB)

; Structure type to access the System Control Block (SCB).

SCB_CPUID         EQU   0x000; Offset: 0x000 (R/ )  CPUID Base Register                                   
SCB_ICSR          EQU   0x004; Offset: 0x004 (R/W)  Interrupt Control and State Register                  
SCB_VTOR          EQU   0x008; Offset: 0x008 (R/W)  Vector Table Offset Register                          
SCB_AIRCR         EQU   0x00C; Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register      
SCB_SCR           EQU   0x010; Offset: 0x010 (R/W)  System Control Register                               
SCB_CCR           EQU   0x014; Offset: 0x014 (R/W)  Configuration Control Register                        
SCB_SHP           EQU   0x018; Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) 
SCB_SHCSR         EQU   0x024; Offset: 0x024 (R/W)  System Handler Control and State Register             
SCB_CFSR          EQU   0x028; Offset: 0x028 (R/W)  Configurable Fault Status Register                    
SCB_HFSR          EQU   0x02C; Offset: 0x02C (R/W)  HardFault Status Register                             
SCB_DFSR          EQU   0x030; Offset: 0x030 (R/W)  Debug Fault Status Register                           
SCB_MMFAR         EQU   0x034; Offset: 0x034 (R/W)  MemManage Fault Address Register                      
SCB_BFAR          EQU   0x038; Offset: 0x038 (R/W)  BusFault Address Register                             
SCB_AFSR          EQU   0x03C; Offset: 0x03C (R/W)  Auxiliary Fault Status Register                       
SCB_PFR           EQU   0x040; Offset: 0x040 (R/ )  Processor Feature Register                            
SCB_DFR           EQU   0x048; Offset: 0x048 (R/ )  Debug Feature Register                                
SCB_ADR           EQU   0x04C; Offset: 0x04C (R/ )  Auxiliary Feature Register                            
SCB_MMFR          EQU   0x050; Offset: 0x050 (R/ )  Memory Model Feature Register                         
SCB_ISAR          EQU   0x060; Offset: 0x060 (R/ )  Instruction Set Attributes Register                   
SCB_CPACR         EQU   0x088; Offset: 0x088 (R/W)  Coprocessor Access Control Register                   

; SCB CPUID Register Definitions 
SCB_CPUID_IMPLEMENTER_Pos          EQU 24                                             ; SCB CPUID: IMPLEMENTER Position 
SCB_CPUID_IMPLEMENTER_Msk          EQU (0xFF << SCB_CPUID_IMPLEMENTER_Pos)          ; SCB CPUID: IMPLEMENTER Mask 

SCB_CPUID_VARIANT_Pos              EQU 20                                             ; SCB CPUID: VARIANT Position 
SCB_CPUID_VARIANT_Msk              EQU (0xF << SCB_CPUID_VARIANT_Pos)               ; SCB CPUID: VARIANT Mask 

SCB_CPUID_ARCHITECTURE_Pos         EQU 16                                             ; SCB CPUID: ARCHITECTURE Position 
SCB_CPUID_ARCHITECTURE_Msk         EQU (0xF << SCB_CPUID_ARCHITECTURE_Pos)          ; SCB CPUID: ARCHITECTURE Mask 

SCB_CPUID_PARTNO_Pos               EQU  4                                             ; SCB CPUID: PARTNO Position 
SCB_CPUID_PARTNO_Msk               EQU (0xFFF << SCB_CPUID_PARTNO_Pos)              ; SCB CPUID: PARTNO Mask 

SCB_CPUID_REVISION_Pos             EQU  0                                             ; SCB CPUID: REVISION Position 
SCB_CPUID_REVISION_Msk             EQU (0xF << SCB_CPUID_REVISION_Pos)              ; SCB CPUID: REVISION Mask 

; SCB Interrupt Control State Register Definitions 
SCB_ICSR_NMIPENDSET_Pos            EQU 31                                             ; SCB ICSR: NMIPENDSET Position 
SCB_ICSR_NMIPENDSET_Msk            EQU (0x1 << SCB_ICSR_NMIPENDSET_Pos)               ; SCB ICSR: NMIPENDSET Mask 

SCB_ICSR_PENDSVSET_Pos             EQU 28                                             ; SCB ICSR: PENDSVSET Position 
SCB_ICSR_PENDSVSET_Msk             EQU (0x1 << SCB_ICSR_PENDSVSET_Pos)                ; SCB ICSR: PENDSVSET Mask 

SCB_ICSR_PENDSVCLR_Pos             EQU 27                                             ; SCB ICSR: PENDSVCLR Position 
SCB_ICSR_PENDSVCLR_Msk             EQU (0x1 << SCB_ICSR_PENDSVCLR_Pos)                ; SCB ICSR: PENDSVCLR Mask 

SCB_ICSR_PENDSTSET_Pos             EQU 26                                             ; SCB ICSR: PENDSTSET Position 
SCB_ICSR_PENDSTSET_Msk             EQU (0x1 << SCB_ICSR_PENDSTSET_Pos)                ; SCB ICSR: PENDSTSET Mask 

SCB_ICSR_PENDSTCLR_Pos             EQU 25                                             ; SCB ICSR: PENDSTCLR Position 
SCB_ICSR_PENDSTCLR_Msk             EQU (0x1 << SCB_ICSR_PENDSTCLR_Pos)                ; SCB ICSR: PENDSTCLR Mask 

SCB_ICSR_ISRPREEMPT_Pos            EQU 23                                             ; SCB ICSR: ISRPREEMPT Position 
SCB_ICSR_ISRPREEMPT_Msk            EQU (0x1 << SCB_ICSR_ISRPREEMPT_Pos)               ; SCB ICSR: ISRPREEMPT Mask 

SCB_ICSR_ISRPENDING_Pos            EQU 22                                             ; SCB ICSR: ISRPENDING Position 
SCB_ICSR_ISRPENDING_Msk            EQU (0x1 << SCB_ICSR_ISRPENDING_Pos)               ; SCB ICSR: ISRPENDING Mask 

SCB_ICSR_VECTPENDING_Pos           EQU 12                                             ; SCB ICSR: VECTPENDING Position 
SCB_ICSR_VECTPENDING_Msk           EQU (0x1FF << SCB_ICSR_VECTPENDING_Pos)          ; SCB ICSR: VECTPENDING Mask 

SCB_ICSR_RETTOBASE_Pos             EQU 11                                             ; SCB ICSR: RETTOBASE Position 
SCB_ICSR_RETTOBASE_Msk             EQU (0x1 << SCB_ICSR_RETTOBASE_Pos)                ; SCB ICSR: RETTOBASE Mask 

SCB_ICSR_VECTACTIVE_Pos            EQU  0                                             ; SCB ICSR: VECTACTIVE Position 
SCB_ICSR_VECTACTIVE_Msk            EQU (0x1FF << SCB_ICSR_VECTACTIVE_Pos)           ; SCB ICSR: VECTACTIVE Mask 

; SCB Vector Table Offset Register Definitions 
SCB_VTOR_TBLOFF_Pos                EQU  7                                             ; SCB VTOR: TBLOFF Position 
SCB_VTOR_TBLOFF_Msk                EQU (0x1FFFFFF << SCB_VTOR_TBLOFF_Pos)           ; SCB VTOR: TBLOFF Mask 

; SCB Application Interrupt and Reset Control Register Definitions 
SCB_AIRCR_VECTKEY_Pos              EQU 16                                             ; SCB AIRCR: VECTKEY Position 
SCB_AIRCR_VECTKEY_Msk              EQU (0xFFFF << SCB_AIRCR_VECTKEY_Pos)            ; SCB AIRCR: VECTKEY Mask 

SCB_AIRCR_VECTKEYSTAT_Pos          EQU 16                                             ; SCB AIRCR: VECTKEYSTAT Position 
SCB_AIRCR_VECTKEYSTAT_Msk          EQU (0xFFFF << SCB_AIRCR_VECTKEYSTAT_Pos)        ; SCB AIRCR: VECTKEYSTAT Mask 

SCB_AIRCR_ENDIANESS_Pos            EQU 15                                             ; SCB AIRCR: ENDIANESS Position 
SCB_AIRCR_ENDIANESS_Msk            EQU (0x1 << SCB_AIRCR_ENDIANESS_Pos)               ; SCB AIRCR: ENDIANESS Mask 

SCB_AIRCR_PRIGROUP_Pos             EQU  8                                             ; SCB AIRCR: PRIGROUP Position 
SCB_AIRCR_PRIGROUP_Msk             EQU (7 << SCB_AIRCR_PRIGROUP_Pos)                ; SCB AIRCR: PRIGROUP Mask 

SCB_AIRCR_SYSRESETREQ_Pos          EQU  2                                             ; SCB AIRCR: SYSRESETREQ Position 
SCB_AIRCR_SYSRESETREQ_Msk          EQU (0x1 << SCB_AIRCR_SYSRESETREQ_Pos)             ; SCB AIRCR: SYSRESETREQ Mask 

SCB_AIRCR_VECTCLRACTIVE_Pos        EQU  1                                             ; SCB AIRCR: VECTCLRACTIVE Position 
SCB_AIRCR_VECTCLRACTIVE_Msk        EQU (0x1 << SCB_AIRCR_VECTCLRACTIVE_Pos)           ; SCB AIRCR: VECTCLRACTIVE Mask 

SCB_AIRCR_VECTRESET_Pos            EQU  0                                             ; SCB AIRCR: VECTRESET Position 
SCB_AIRCR_VECTRESET_Msk            EQU (0x1 << SCB_AIRCR_VECTRESET_Pos)               ; SCB AIRCR: VECTRESET Mask 

; SCB System Control Register Definitions 
SCB_SCR_SEVONPEND_Pos              EQU  4                                             ; SCB SCR: SEVONPEND Position 
SCB_SCR_SEVONPEND_Msk              EQU (0x1 << SCB_SCR_SEVONPEND_Pos)                 ; SCB SCR: SEVONPEND Mask 

SCB_SCR_SLEEPDEEP_Pos              EQU  2                                             ; SCB SCR: SLEEPDEEP Position 
SCB_SCR_SLEEPDEEP_Msk              EQU (0x1 << SCB_SCR_SLEEPDEEP_Pos)                 ; SCB SCR: SLEEPDEEP Mask 

SCB_SCR_SLEEPONEXIT_Pos            EQU  1                                             ; SCB SCR: SLEEPONEXIT Position 
SCB_SCR_SLEEPONEXIT_Msk            EQU (0x1 << SCB_SCR_SLEEPONEXIT_Pos)               ; SCB SCR: SLEEPONEXIT Mask 

; SCB Configuration Control Register Definitions 
SCB_CCR_STKALIGN_Pos               EQU  9                                             ; SCB CCR: STKALIGN Position 
SCB_CCR_STKALIGN_Msk               EQU (0x1 << SCB_CCR_STKALIGN_Pos)                  ; SCB CCR: STKALIGN Mask 

SCB_CCR_BFHFNMIGN_Pos              EQU  8                                             ; SCB CCR: BFHFNMIGN Position 
SCB_CCR_BFHFNMIGN_Msk              EQU (0x1 << SCB_CCR_BFHFNMIGN_Pos)                 ; SCB CCR: BFHFNMIGN Mask 

SCB_CCR_DIV_0_TRP_Pos              EQU  4                                             ; SCB CCR: DIV_0_TRP Position 
SCB_CCR_DIV_0_TRP_Msk              EQU (0x1 << SCB_CCR_DIV_0_TRP_Pos)                 ; SCB CCR: DIV_0_TRP Mask 

SCB_CCR_UNALIGN_TRP_Pos            EQU  3                                             ; SCB CCR: UNALIGN_TRP Position 
SCB_CCR_UNALIGN_TRP_Msk            EQU (0x1 << SCB_CCR_UNALIGN_TRP_Pos)               ; SCB CCR: UNALIGN_TRP Mask 

SCB_CCR_USERSETMPEND_Pos           EQU  1                                             ; SCB CCR: USERSETMPEND Position 
SCB_CCR_USERSETMPEND_Msk           EQU (0x1 << SCB_CCR_USERSETMPEND_Pos)              ; SCB CCR: USERSETMPEND Mask 

SCB_CCR_NONBASETHRDENA_Pos         EQU  0                                             ; SCB CCR: NONBASETHRDENA Position 
SCB_CCR_NONBASETHRDENA_Msk         EQU (0x1 << SCB_CCR_NONBASETHRDENA_Pos)            ; SCB CCR: NONBASETHRDENA Mask 

; SCB System Handler Control and State Register Definitions 
SCB_SHCSR_USGFAULTENA_Pos          EQU 18                                             ; SCB SHCSR: USGFAULTENA Position 
SCB_SHCSR_USGFAULTENA_Msk          EQU (0x1 << SCB_SHCSR_USGFAULTENA_Pos)             ; SCB SHCSR: USGFAULTENA Mask 

SCB_SHCSR_BUSFAULTENA_Pos          EQU 17                                             ; SCB SHCSR: BUSFAULTENA Position 
SCB_SHCSR_BUSFAULTENA_Msk          EQU (0x1 << SCB_SHCSR_BUSFAULTENA_Pos)             ; SCB SHCSR: BUSFAULTENA Mask 

SCB_SHCSR_MEMFAULTENA_Pos          EQU 16                                             ; SCB SHCSR: MEMFAULTENA Position 
SCB_SHCSR_MEMFAULTENA_Msk          EQU (0x1 << SCB_SHCSR_MEMFAULTENA_Pos)             ; SCB SHCSR: MEMFAULTENA Mask 

SCB_SHCSR_SVCALLPENDED_Pos         EQU 15                                             ; SCB SHCSR: SVCALLPENDED Position 
SCB_SHCSR_SVCALLPENDED_Msk         EQU (0x1 << SCB_SHCSR_SVCALLPENDED_Pos)            ; SCB SHCSR: SVCALLPENDED Mask 

SCB_SHCSR_BUSFAULTPENDED_Pos       EQU 14                                             ; SCB SHCSR: BUSFAULTPENDED Position 
SCB_SHCSR_BUSFAULTPENDED_Msk       EQU (0x1 << SCB_SHCSR_BUSFAULTPENDED_Pos)          ; SCB SHCSR: BUSFAULTPENDED Mask 

SCB_SHCSR_MEMFAULTPENDED_Pos       EQU 13                                             ; SCB SHCSR: MEMFAULTPENDED Position 
SCB_SHCSR_MEMFAULTPENDED_Msk       EQU (0x1 << SCB_SHCSR_MEMFAULTPENDED_Pos)          ; SCB SHCSR: MEMFAULTPENDED Mask 

SCB_SHCSR_USGFAULTPENDED_Pos       EQU 12                                             ; SCB SHCSR: USGFAULTPENDED Position 
SCB_SHCSR_USGFAULTPENDED_Msk       EQU (0x1 << SCB_SHCSR_USGFAULTPENDED_Pos)          ; SCB SHCSR: USGFAULTPENDED Mask 

SCB_SHCSR_SYSTICKACT_Pos           EQU 11                                             ; SCB SHCSR: SYSTICKACT Position 
SCB_SHCSR_SYSTICKACT_Msk           EQU (0x1 << SCB_SHCSR_SYSTICKACT_Pos)              ; SCB SHCSR: SYSTICKACT Mask 

SCB_SHCSR_PENDSVACT_Pos            EQU 10                                             ; SCB SHCSR: PENDSVACT Position 
SCB_SHCSR_PENDSVACT_Msk            EQU (0x1 << SCB_SHCSR_PENDSVACT_Pos)               ; SCB SHCSR: PENDSVACT Mask 

SCB_SHCSR_MONITORACT_Pos           EQU  8                                             ; SCB SHCSR: MONITORACT Position 
SCB_SHCSR_MONITORACT_Msk           EQU (0x1 << SCB_SHCSR_MONITORACT_Pos)              ; SCB SHCSR: MONITORACT Mask 

SCB_SHCSR_SVCALLACT_Pos            EQU  7                                             ; SCB SHCSR: SVCALLACT Position 
SCB_SHCSR_SVCALLACT_Msk            EQU (0x1 << SCB_SHCSR_SVCALLACT_Pos)               ; SCB SHCSR: SVCALLACT Mask 

SCB_SHCSR_USGFAULTACT_Pos          EQU  3                                             ; SCB SHCSR: USGFAULTACT Position 
SCB_SHCSR_USGFAULTACT_Msk          EQU (0x1 << SCB_SHCSR_USGFAULTACT_Pos)             ; SCB SHCSR: USGFAULTACT Mask 

SCB_SHCSR_BUSFAULTACT_Pos          EQU  1                                             ; SCB SHCSR: BUSFAULTACT Position 
SCB_SHCSR_BUSFAULTACT_Msk          EQU (0x1 << SCB_SHCSR_BUSFAULTACT_Pos)             ; SCB SHCSR: BUSFAULTACT Mask 

SCB_SHCSR_MEMFAULTACT_Pos          EQU  0                                             ; SCB SHCSR: MEMFAULTACT Position 
SCB_SHCSR_MEMFAULTACT_Msk          EQU (0x1 << SCB_SHCSR_MEMFAULTACT_Pos)             ; SCB SHCSR: MEMFAULTACT Mask 

; SCB Configurable Fault Status Registers Definitions 
SCB_CFSR_USGFAULTSR_Pos            EQU 16                                             ; SCB CFSR: Usage Fault Status Register Position 
SCB_CFSR_USGFAULTSR_Msk            EQU (0xFFFF << SCB_CFSR_USGFAULTSR_Pos)          ; SCB CFSR: Usage Fault Status Register Mask 

SCB_CFSR_BUSFAULTSR_Pos            EQU  8                                             ; SCB CFSR: Bus Fault Status Register Position 
SCB_CFSR_BUSFAULTSR_Msk            EQU (0xFF << SCB_CFSR_BUSFAULTSR_Pos)            ; SCB CFSR: Bus Fault Status Register Mask 

SCB_CFSR_MEMFAULTSR_Pos            EQU  0                                             ; SCB CFSR: Memory Manage Fault Status Register Position 
SCB_CFSR_MEMFAULTSR_Msk            EQU (0xFF << SCB_CFSR_MEMFAULTSR_Pos)            ; SCB CFSR: Memory Manage Fault Status Register Mask 

; SCB Hard Fault Status Registers Definitions 
SCB_HFSR_DEBUGEVT_Pos              EQU 31                                             ; SCB HFSR: DEBUGEVT Position 
SCB_HFSR_DEBUGEVT_Msk              EQU (0x1 << SCB_HFSR_DEBUGEVT_Pos)                 ; SCB HFSR: DEBUGEVT Mask 

SCB_HFSR_FORCED_Pos                EQU 30                                             ; SCB HFSR: FORCED Position 
SCB_HFSR_FORCED_Msk                EQU (0x1 << SCB_HFSR_FORCED_Pos)                   ; SCB HFSR: FORCED Mask 

SCB_HFSR_VECTTBL_Pos               EQU  1                                             ; SCB HFSR: VECTTBL Position 
SCB_HFSR_VECTTBL_Msk               EQU (0x1 << SCB_HFSR_VECTTBL_Pos)                  ; SCB HFSR: VECTTBL Mask 

; SCB Debug Fault Status Register Definitions 
SCB_DFSR_EXTERNAL_Pos              EQU  4                                             ; SCB DFSR: EXTERNAL Position 
SCB_DFSR_EXTERNAL_Msk              EQU (0x1 << SCB_DFSR_EXTERNAL_Pos)                 ; SCB DFSR: EXTERNAL Mask 

SCB_DFSR_VCATCH_Pos                EQU  3                                             ; SCB DFSR: VCATCH Position 
SCB_DFSR_VCATCH_Msk                EQU (0x1 << SCB_DFSR_VCATCH_Pos)                   ; SCB DFSR: VCATCH Mask 

SCB_DFSR_DWTTRAP_Pos               EQU  2                                             ; SCB DFSR: DWTTRAP Position 
SCB_DFSR_DWTTRAP_Msk               EQU (0x1 << SCB_DFSR_DWTTRAP_Pos)                  ; SCB DFSR: DWTTRAP Mask 

SCB_DFSR_BKPT_Pos                  EQU  1                                             ; SCB DFSR: BKPT Position 
SCB_DFSR_BKPT_Msk                  EQU (0x1 << SCB_DFSR_BKPT_Pos)                     ; SCB DFSR: BKPT Mask 

SCB_DFSR_HALTED_Pos                EQU  0                                             ; SCB DFSR: HALTED Position 
SCB_DFSR_HALTED_Msk                EQU (0x1 << SCB_DFSR_HALTED_Pos)                   ; SCB DFSR: HALTED Mask 




; CMSIS_core_register
; CMSIS_SCnSCB System Controls not in SCB (SCnSCB)
; Type definitions for the System Control and ID Register not in the SCB
; Structure type to access the System Control and ID Register not in the SCB.

SCnSCB_ICTR    EQU 0x004                   ; Offset: 0x004 (R/ )  Interrupt Controller Type Register      
SCnSCB_ACTLR   EQU 0x008                   ; Offset: 0x008 (R/W)  Auxiliary Control Register              

; Interrupt Controller Type Register Definitions 
SCnSCB_ICTR_INTLINESNUM_Pos        EQU  0                                        ; ICTR: INTLINESNUM Position 
SCnSCB_ICTR_INTLINESNUM_Msk        EQU (0xF << SCnSCB_ICTR_INTLINESNUM_Pos)      ; ICTR: INTLINESNUM Mask 

; Auxiliary Control Register Definitions 
SCnSCB_ACTLR_DISOOFP_Pos           EQU  9                                          ; ACTLR: DISOOFP Position 
SCnSCB_ACTLR_DISOOFP_Msk           EQU (0x1 << SCnSCB_ACTLR_DISOOFP_Pos)           ; ACTLR: DISOOFP Mask 

SCnSCB_ACTLR_DISFPCA_Pos           EQU  8                                          ; ACTLR: DISFPCA Position 
SCnSCB_ACTLR_DISFPCA_Msk           EQU (0x1 << SCnSCB_ACTLR_DISFPCA_Pos)           ; ACTLR: DISFPCA Mask 

SCnSCB_ACTLR_DISFOLD_Pos           EQU  2                                          ; ACTLR: DISFOLD Position 
SCnSCB_ACTLR_DISFOLD_Msk           EQU (0x1 << SCnSCB_ACTLR_DISFOLD_Pos)           ; ACTLR: DISFOLD Mask 

SCnSCB_ACTLR_DISDEFWBUF_Pos        EQU  1                                          ; ACTLR: DISDEFWBUF Position 
SCnSCB_ACTLR_DISDEFWBUF_Msk        EQU (0x1 << SCnSCB_ACTLR_DISDEFWBUF_Pos)        ; ACTLR: DISDEFWBUF Mask 

SCnSCB_ACTLR_DISMCYCINT_Pos        EQU  0                                          ; ACTLR: DISMCYCINT Position 
SCnSCB_ACTLR_DISMCYCINT_Msk        EQU (0x1 << SCnSCB_ACTLR_DISMCYCINT_Pos)        ; ACTLR: DISMCYCINT Mask 


;CMSIS_core_register
; CMSIS_SysTick     System Tick Timer (SysTick)
; Type definitions for the System Timer Registers.

; Structure type to access the System Timer (SysTick).
 

SysTick_CTRL      EQU 0x000              ; Offset: 0x000 (R/W)  SysTick Control and Status Register 
SysTick_LOAD      EQU 0x004              ; Offset: 0x004 (R/W)  SysTick Reload Value Register       
SysTick_VAL       EQU 0x008              ; Offset: 0x008 (R/W)  SysTick Current Value Register      
SysTick_CALIB     EQU 0x00C              ; Offset: 0x00C (R/ )  SysTick Calibration Register        

; SysTick Control / Status Register Definitions 
SysTick_CTRL_COUNTFLAG_Pos         EQU 16                                             ; SysTick CTRL: COUNTFLAG Position 
SysTick_CTRL_COUNTFLAG_Msk         EQU (0x1 << SysTick_CTRL_COUNTFLAG_Pos)            ; SysTick CTRL: COUNTFLAG Mask 

SysTick_CTRL_CLKSOURCE_Pos         EQU  2                                             ; SysTick CTRL: CLKSOURCE Position 
SysTick_CTRL_CLKSOURCE_Msk         EQU (0x1 << SysTick_CTRL_CLKSOURCE_Pos)            ; SysTick CTRL: CLKSOURCE Mask 

SysTick_CTRL_TICKINT_Pos           EQU  1                                             ; SysTick CTRL: TICKINT Position 
SysTick_CTRL_TICKINT_Msk           EQU (0x1 << SysTick_CTRL_TICKINT_Pos)              ; SysTick CTRL: TICKINT Mask 

SysTick_CTRL_ENABLE_Pos            EQU  0                                             ; SysTick CTRL: ENABLE Position 
SysTick_CTRL_ENABLE_Msk            EQU (0x1 << SysTick_CTRL_ENABLE_Pos)               ; SysTick CTRL: ENABLE Mask 

; SysTick Reload Register Definitions 
SysTick_LOAD_RELOAD_Pos            EQU  0                                             ; SysTick LOAD: RELOAD Position 
SysTick_LOAD_RELOAD_Msk            EQU (0xFFFFFF << SysTick_LOAD_RELOAD_Pos)        ; SysTick LOAD: RELOAD Mask 

; SysTick Current Register Definitions 
SysTick_VAL_CURRENT_Pos            EQU  0                                             ; SysTick VAL: CURRENT Position 
SysTick_VAL_CURRENT_Msk            EQU (0xFFFFFF << SysTick_VAL_CURRENT_Pos)        ; SysTick VAL: CURRENT Mask 

; SysTick Calibration Register Definitions 
SysTick_CALIB_NOREF_Pos            EQU 31                                             ; SysTick CALIB: NOREF Position 
SysTick_CALIB_NOREF_Msk            EQU (0x1 << SysTick_CALIB_NOREF_Pos)               ; SysTick CALIB: NOREF Mask 

SysTick_CALIB_SKEW_Pos             EQU 30                                             ; SysTick CALIB: SKEW Position 
SysTick_CALIB_SKEW_Msk             EQU (0x1 << SysTick_CALIB_SKEW_Pos)                ; SysTick CALIB: SKEW Mask 

SysTick_CALIB_TENMS_Pos            EQU  0                                             ; SysTick CALIB: TENMS Position 
SysTick_CALIB_TENMS_Msk            EQU (0xFFFFFF << SysTick_CALIB_TENMS_Pos)        ; SysTick CALIB: TENMS Mask 


;CMSIS_core_register
; CMSIS_ITM     Instrumentation Trace Macrocell (ITM)
; Type definitions for the Instrumentation Trace Macrocell (ITM)

; Structure type to access the Instrumentation Trace Macrocell Register (ITM).
 
ITM_PORT     EQU 0x000      ; Offset: 0x000 ( /W)  ITM Stimulus Port Registers               
ITM_TER      EQU 0xE00      ; Offset: 0xE00 (R/W)  ITM Trace Enable Register                 
ITM_TPR      EQU 0xE40      ; Offset: 0xE40 (R/W)  ITM Trace Privilege Register              
ITM_TCR      EQU 0xE80      ; Offset: 0xE80 (R/W)  ITM Trace Control Register                
ITM_IWR      EQU 0xEF8      ; Offset: 0xEF8 ( /W)  ITM Integration Write Register            
ITM_IRR      EQU 0xEFC      ; Offset: 0xEFC (R/ )  ITM Integration Read Register             
ITM_IMCR     EQU 0xF00      ; Offset: 0xF00 (R/W)  ITM Integration Mode Control Register     
ITM_LAR      EQU 0xFB0      ; Offset: 0xFB0 ( /W)  ITM Lock Access Register                  
ITM_LSR      EQU 0xFB4      ; Offset: 0xFB4 (R/ )  ITM Lock Status Register                  
ITM_PID4     EQU 0xFD0      ; Offset: 0xFD0 (R/ )  ITM Peripheral Identification Register #4 
ITM_PID5     EQU 0xFD4      ; Offset: 0xFD4 (R/ )  ITM Peripheral Identification Register #5 
ITM_PID6     EQU 0xFD5      ; Offset: 0xFD8 (R/ )  ITM Peripheral Identification Register #6 
ITM_PID7     EQU 0xFDC      ; Offset: 0xFDC (R/ )  ITM Peripheral Identification Register #7 
ITM_PID0     EQU 0xFE0      ; Offset: 0xFE0 (R/ )  ITM Peripheral Identification Register #0 
ITM_PID1     EQU 0xFE4      ; Offset: 0xFE4 (R/ )  ITM Peripheral Identification Register #1 
ITM_PID2     EQU 0xFE8      ; Offset: 0xFE8 (R/ )  ITM Peripheral Identification Register #2 
ITM_PID3     EQU 0xFEC      ; Offset: 0xFEC (R/ )  ITM Peripheral Identification Register #3 
ITM_CID0     EQU 0xFF0      ; Offset: 0xFF0 (R/ )  ITM Component  Identification Register #0 
ITM_CID1     EQU 0xFF4      ; Offset: 0xFF4 (R/ )  ITM Component  Identification Register #1 
ITM_CID2     EQU 0xFF8      ; Offset: 0xFF8 (R/ )  ITM Component  Identification Register #2 
ITM_CID3     EQU 0xFFC      ; Offset: 0xFFC (R/ )  ITM Component  Identification Register #3 

; ITM Trace Privilege Register Definitions 
ITM_TPR_PRIVMASK_Pos               EQU  0                                           ; ITM TPR: PRIVMASK Position 
ITM_TPR_PRIVMASK_Msk               EQU (0xF << ITM_TPR_PRIVMASK_Pos)                ; ITM TPR: PRIVMASK Mask 

; ITM Trace Control Register Definitions 
ITM_TCR_BUSY_Pos                   EQU 23                                           ; ITM TCR: BUSY Position 
ITM_TCR_BUSY_Msk                   EQU (0x1 << ITM_TCR_BUSY_Pos)                    ; ITM TCR: BUSY Mask 

ITM_TCR_TraceBusID_Pos             EQU 16                                           ; ITM TCR: ATBID Position 
ITM_TCR_TraceBusID_Msk             EQU (0x7F << ITM_TCR_TraceBusID_Pos)             ; ITM TCR: ATBID Mask 

ITM_TCR_GTSFREQ_Pos                EQU 10                                           ; ITM TCR: Global timestamp frequency Position 
ITM_TCR_GTSFREQ_Msk                EQU (3 << ITM_TCR_GTSFREQ_Pos)                   ; ITM TCR: Global timestamp frequency Mask 

ITM_TCR_TSPrescale_Pos             EQU  8                                           ; ITM TCR: TSPrescale Position 
ITM_TCR_TSPrescale_Msk             EQU (3 << ITM_TCR_TSPrescale_Pos)                ; ITM TCR: TSPrescale Mask 

ITM_TCR_SWOENA_Pos                 EQU  4                                           ; ITM TCR: SWOENA Position 
ITM_TCR_SWOENA_Msk                 EQU (0x1 << ITM_TCR_SWOENA_Pos)                  ; ITM TCR: SWOENA Mask 

ITM_TCR_DWTENA_Pos                 EQU  3                                           ; ITM TCR: DWTENA Position 
ITM_TCR_DWTENA_Msk                 EQU (0x1 << ITM_TCR_DWTENA_Pos)                  ; ITM TCR: DWTENA Mask 

ITM_TCR_SYNCENA_Pos                EQU  2                                           ; ITM TCR: SYNCENA Position 
ITM_TCR_SYNCENA_Msk                EQU (0x1 << ITM_TCR_SYNCENA_Pos)                 ; ITM TCR: SYNCENA Mask 

ITM_TCR_TSENA_Pos                  EQU  1                                           ; ITM TCR: TSENA Position 
ITM_TCR_TSENA_Msk                  EQU (0x1 << ITM_TCR_TSENA_Pos)                   ; ITM TCR: TSENA Mask 

ITM_TCR_ITMENA_Pos                 EQU  0                                           ; ITM TCR: ITM Enable bit Position 
ITM_TCR_ITMENA_Msk                 EQU (0x1 << ITM_TCR_ITMENA_Pos)                  ; ITM TCR: ITM Enable bit Mask 

; ITM Integration Write Register Definitions 
ITM_IWR_ATVALIDM_Pos               EQU  0                                           ; ITM IWR: ATVALIDM Position 
ITM_IWR_ATVALIDM_Msk               EQU (0x1 << ITM_IWR_ATVALIDM_Pos)                ; ITM IWR: ATVALIDM Mask 

; ITM Integration Read Register Definitions 
ITM_IRR_ATREADYM_Pos               EQU  0                                           ; ITM IRR: ATREADYM Position 
ITM_IRR_ATREADYM_Msk               EQU (0x1 << ITM_IRR_ATREADYM_Pos)                ; ITM IRR: ATREADYM Mask 

; ITM Integration Mode Control Register Definitions 
ITM_IMCR_INTEGRATION_Pos           EQU  0                                           ; ITM IMCR: INTEGRATION Position 
ITM_IMCR_INTEGRATION_Msk           EQU (0x1 << ITM_IMCR_INTEGRATION_Pos)            ; ITM IMCR: INTEGRATION Mask 

; ITM Lock Status Register Definitions 
ITM_LSR_ByteAcc_Pos                EQU  2                                           ; ITM LSR: ByteAcc Position 
ITM_LSR_ByteAcc_Msk                EQU (0x1 << ITM_LSR_ByteAcc_Pos)                 ; ITM LSR: ByteAcc Mask 

ITM_LSR_Access_Pos                 EQU  1                                           ; ITM LSR: Access Position 
ITM_LSR_Access_Msk                 EQU (0x1 << ITM_LSR_Access_Pos)                  ; ITM LSR: Access Mask 

ITM_LSR_Present_Pos                EQU  0                                           ; ITM LSR: Present Position 
ITM_LSR_Present_Msk                EQU (0x1 << ITM_LSR_Present_Pos)                 ; ITM LSR: Present Mask 

;CMSIS_core_register
; CMSIS_DWT     Data Watchpoint and Trace (DWT)
; Type definitions for the Data Watchpoint and Trace (DWT)
; Structure type to access the Data Watchpoint and Trace Register (DWT).

DWT_CTRL      EQU 0x000   ; Offset: 0x000 (R/W)  Control Register                          
DWT_CYCCNT    EQU 0x004   ; Offset: 0x004 (R/W)  Cycle Count Register                      
DWT_CPICNT    EQU 0x008   ; Offset: 0x008 (R/W)  CPI Count Register                        
DWT_EXCCNT    EQU 0x00C   ; Offset: 0x00C (R/W)  Exception Overhead Count Register         
DWT_SLEEPCNT  EQU 0x010   ; Offset: 0x010 (R/W)  Sleep Count Register                      
DWT_LSUCNT    EQU 0x014   ; Offset: 0x014 (R/W)  LSU Count Register                        
DWT_FOLDCNT   EQU 0x018   ; Offset: 0x018 (R/W)  Folded-instruction Count Register         
DWT_PCSR      EQU 0x01C   ; Offset: 0x01C (R/ )  Program Counter Sample Register           
DWT_COMP0     EQU 0x020   ; Offset: 0x020 (R/W)  Comparator Register 0                     
DWT_MASK0     EQU 0x024   ; Offset: 0x024 (R/W)  Mask Register 0                           
DWT_FUNCTION0 EQU 0x028   ; Offset: 0x028 (R/W)  Function Register 0                       
DWT_COMP1     EQU 0x030   ; Offset: 0x030 (R/W)  Comparator Register 1                     
DWT_MASK1     EQU 0x034   ; Offset: 0x034 (R/W)  Mask Register 1                           
DWT_FUNCTION1 EQU 0x038   ; Offset: 0x038 (R/W)  Function Register 1                       
DWT_COMP2     EQU 0x040   ; Offset: 0x040 (R/W)  Comparator Register 2                     
DWT_MASK2     EQU 0x044   ; Offset: 0x044 (R/W)  Mask Register 2                           
DWT_FUNCTION2 EQU 0x048   ; Offset: 0x048 (R/W)  Function Register 2                       
DWT_COMP3     EQU 0x050   ; Offset: 0x050 (R/W)  Comparator Register 3                     
DWT_MASK3     EQU 0x054   ; Offset: 0x054 (R/W)  Mask Register 3                           
DWT_FUNCTION3 EQU 0x058   ; Offset: 0x058 (R/W)  Function Register 3                       

; DWT Control Register Definitions 
DWT_CTRL_NUMCOMP_Pos               EQU 28                                          ; DWT CTRL: NUMCOMP Position 
DWT_CTRL_NUMCOMP_Msk               EQU (0xF << DWT_CTRL_NUMCOMP_Pos)             ; DWT CTRL: NUMCOMP Mask 

DWT_CTRL_NOTRCPKT_Pos              EQU 27                                          ; DWT CTRL: NOTRCPKT Position 
DWT_CTRL_NOTRCPKT_Msk              EQU (0x1 << DWT_CTRL_NOTRCPKT_Pos)            ; DWT CTRL: NOTRCPKT Mask 

DWT_CTRL_NOEXTTRIG_Pos             EQU 26                                          ; DWT CTRL: NOEXTTRIG Position 
DWT_CTRL_NOEXTTRIG_Msk             EQU (0x1 << DWT_CTRL_NOEXTTRIG_Pos)           ; DWT CTRL: NOEXTTRIG Mask 

DWT_CTRL_NOCYCCNT_Pos              EQU 25                                          ; DWT CTRL: NOCYCCNT Position 
DWT_CTRL_NOCYCCNT_Msk              EQU (0x1 << DWT_CTRL_NOCYCCNT_Pos)            ; DWT CTRL: NOCYCCNT Mask 

DWT_CTRL_NOPRFCNT_Pos              EQU 24                                          ; DWT CTRL: NOPRFCNT Position 
DWT_CTRL_NOPRFCNT_Msk              EQU (0x1 << DWT_CTRL_NOPRFCNT_Pos)            ; DWT CTRL: NOPRFCNT Mask 

DWT_CTRL_CYCEVTENA_Pos             EQU 22                                          ; DWT CTRL: CYCEVTENA Position 
DWT_CTRL_CYCEVTENA_Msk             EQU (0x1 << DWT_CTRL_CYCEVTENA_Pos)           ; DWT CTRL: CYCEVTENA Mask 

DWT_CTRL_FOLDEVTENA_Pos            EQU 21                                          ; DWT CTRL: FOLDEVTENA Position 
DWT_CTRL_FOLDEVTENA_Msk            EQU (0x1 << DWT_CTRL_FOLDEVTENA_Pos)          ; DWT CTRL: FOLDEVTENA Mask 

DWT_CTRL_LSUEVTENA_Pos             EQU 20                                          ; DWT CTRL: LSUEVTENA Position 
DWT_CTRL_LSUEVTENA_Msk             EQU (0x1 << DWT_CTRL_LSUEVTENA_Pos)           ; DWT CTRL: LSUEVTENA Mask 

DWT_CTRL_SLEEPEVTENA_Pos           EQU 19                                          ; DWT CTRL: SLEEPEVTENA Position 
DWT_CTRL_SLEEPEVTENA_Msk           EQU (0x1 << DWT_CTRL_SLEEPEVTENA_Pos)         ; DWT CTRL: SLEEPEVTENA Mask 

DWT_CTRL_EXCEVTENA_Pos             EQU 18                                          ; DWT CTRL: EXCEVTENA Position 
DWT_CTRL_EXCEVTENA_Msk             EQU (0x1 << DWT_CTRL_EXCEVTENA_Pos)           ; DWT CTRL: EXCEVTENA Mask 

DWT_CTRL_CPIEVTENA_Pos             EQU 17                                          ; DWT CTRL: CPIEVTENA Position 
DWT_CTRL_CPIEVTENA_Msk             EQU (0x1 << DWT_CTRL_CPIEVTENA_Pos)           ; DWT CTRL: CPIEVTENA Mask 

DWT_CTRL_EXCTRCENA_Pos             EQU 16                                          ; DWT CTRL: EXCTRCENA Position 
DWT_CTRL_EXCTRCENA_Msk             EQU (0x1 << DWT_CTRL_EXCTRCENA_Pos)           ; DWT CTRL: EXCTRCENA Mask 

DWT_CTRL_PCSAMPLENA_Pos            EQU 12                                          ; DWT CTRL: PCSAMPLENA Position 
DWT_CTRL_PCSAMPLENA_Msk            EQU (0x1 << DWT_CTRL_PCSAMPLENA_Pos)          ; DWT CTRL: PCSAMPLENA Mask 

DWT_CTRL_SYNCTAP_Pos               EQU 10                                          ; DWT CTRL: SYNCTAP Position 
DWT_CTRL_SYNCTAP_Msk               EQU (0x3 << DWT_CTRL_SYNCTAP_Pos)             ; DWT CTRL: SYNCTAP Mask 

DWT_CTRL_CYCTAP_Pos                EQU  9                                          ; DWT CTRL: CYCTAP Position 
DWT_CTRL_CYCTAP_Msk                EQU (0x1 << DWT_CTRL_CYCTAP_Pos)              ; DWT CTRL: CYCTAP Mask 

DWT_CTRL_POSTINIT_Pos              EQU  5                                          ; DWT CTRL: POSTINIT Position 
DWT_CTRL_POSTINIT_Msk              EQU (0xF << DWT_CTRL_POSTINIT_Pos)            ; DWT CTRL: POSTINIT Mask 

DWT_CTRL_POSTPRESET_Pos            EQU  1                                          ; DWT CTRL: POSTPRESET Position 
DWT_CTRL_POSTPRESET_Msk            EQU (0xF << DWT_CTRL_POSTPRESET_Pos)          ; DWT CTRL: POSTPRESET Mask 

DWT_CTRL_CYCCNTENA_Pos             EQU  0                                          ; DWT CTRL: CYCCNTENA Position 
DWT_CTRL_CYCCNTENA_Msk             EQU (0x1 << DWT_CTRL_CYCCNTENA_Pos)           ; DWT CTRL: CYCCNTENA Mask 

; DWT CPI Count Register Definitions 
DWT_CPICNT_CPICNT_Pos              EQU  0                                          ; DWT CPICNT: CPICNT Position 
DWT_CPICNT_CPICNT_Msk              EQU (0xFF << DWT_CPICNT_CPICNT_Pos)           ; DWT CPICNT: CPICNT Mask 

; DWT Exception Overhead Count Register Definitions 
DWT_EXCCNT_EXCCNT_Pos              EQU  0                                          ; DWT EXCCNT: EXCCNT Position 
DWT_EXCCNT_EXCCNT_Msk              EQU (0xFF << DWT_EXCCNT_EXCCNT_Pos)           ; DWT EXCCNT: EXCCNT Mask 

; DWT Sleep Count Register Definitions 
DWT_SLEEPCNT_SLEEPCNT_Pos          EQU  0                                          ; DWT SLEEPCNT: SLEEPCNT Position 
DWT_SLEEPCNT_SLEEPCNT_Msk          EQU (0xFF << DWT_SLEEPCNT_SLEEPCNT_Pos)       ; DWT SLEEPCNT: SLEEPCNT Mask 

; DWT LSU Count Register Definitions 
DWT_LSUCNT_LSUCNT_Pos              EQU  0                                          ; DWT LSUCNT: LSUCNT Position 
DWT_LSUCNT_LSUCNT_Msk              EQU (0xFF << DWT_LSUCNT_LSUCNT_Pos)           ; DWT LSUCNT: LSUCNT Mask 

; DWT Folded-instruction Count Register Definitions 
DWT_FOLDCNT_FOLDCNT_Pos            EQU  0                                          ; DWT FOLDCNT: FOLDCNT Position 
DWT_FOLDCNT_FOLDCNT_Msk            EQU (0xFF << DWT_FOLDCNT_FOLDCNT_Pos)         ; DWT FOLDCNT: FOLDCNT Mask 

; DWT Comparator Mask Register Definitions 
DWT_MASK_MASK_Pos                  EQU  0                                          ; DWT MASK: MASK Position 
DWT_MASK_MASK_Msk                  EQU (0x1F << DWT_MASK_MASK_Pos)               ; DWT MASK: MASK Mask 

; DWT Comparator Function Register Definitions 
DWT_FUNCTION_MATCHED_Pos           EQU 24                                          ; DWT FUNCTION: MATCHED Position 
DWT_FUNCTION_MATCHED_Msk           EQU (0x1 << DWT_FUNCTION_MATCHED_Pos)         ; DWT FUNCTION: MATCHED Mask 

DWT_FUNCTION_DATAVADDR1_Pos        EQU 16                                          ; DWT FUNCTION: DATAVADDR1 Position 
DWT_FUNCTION_DATAVADDR1_Msk        EQU (0xF << DWT_FUNCTION_DATAVADDR1_Pos)      ; DWT FUNCTION: DATAVADDR1 Mask 

DWT_FUNCTION_DATAVADDR0_Pos        EQU 12                                          ; DWT FUNCTION: DATAVADDR0 Position 
DWT_FUNCTION_DATAVADDR0_Msk        EQU (0xF << DWT_FUNCTION_DATAVADDR0_Pos)      ; DWT FUNCTION: DATAVADDR0 Mask 

DWT_FUNCTION_DATAVSIZE_Pos         EQU 10                                          ; DWT FUNCTION: DATAVSIZE Position 
DWT_FUNCTION_DATAVSIZE_Msk         EQU (0x3 << DWT_FUNCTION_DATAVSIZE_Pos)       ; DWT FUNCTION: DATAVSIZE Mask 

DWT_FUNCTION_LNK1ENA_Pos           EQU  9                                          ; DWT FUNCTION: LNK1ENA Position 
DWT_FUNCTION_LNK1ENA_Msk           EQU (0x1 << DWT_FUNCTION_LNK1ENA_Pos)         ; DWT FUNCTION: LNK1ENA Mask 

DWT_FUNCTION_DATAVMATCH_Pos        EQU  8                                          ; DWT FUNCTION: DATAVMATCH Position 
DWT_FUNCTION_DATAVMATCH_Msk        EQU (0x1 << DWT_FUNCTION_DATAVMATCH_Pos)      ; DWT FUNCTION: DATAVMATCH Mask 

DWT_FUNCTION_CYCMATCH_Pos          EQU  7                                          ; DWT FUNCTION: CYCMATCH Position 
DWT_FUNCTION_CYCMATCH_Msk          EQU (0x1 << DWT_FUNCTION_CYCMATCH_Pos)        ; DWT FUNCTION: CYCMATCH Mask 

DWT_FUNCTION_EMITRANGE_Pos         EQU  5                                          ; DWT FUNCTION: EMITRANGE Position 
DWT_FUNCTION_EMITRANGE_Msk         EQU (0x1 << DWT_FUNCTION_EMITRANGE_Pos)       ; DWT FUNCTION: EMITRANGE Mask 

DWT_FUNCTION_FUNCTION_Pos          EQU  0                                          ; DWT FUNCTION: FUNCTION Position 
DWT_FUNCTION_FUNCTION_Msk          EQU (0xF << DWT_FUNCTION_FUNCTION_Pos)        ; DWT FUNCTION: FUNCTION Mask 



;CMSIS_core_register
; CMSIS_TPI     Trace Port Interface (TPI)
; Type definitions for the Trace Port Interface (TPI)

; Structure type to access the Trace Port Interface Register (TPI).

TPI_SSPSR     EQU 0x000    ; Offset: 0x000 (R/ )  Supported Parallel Port Size Register     
TPI_CSPSR     EQU 0x004    ; Offset: 0x004 (R/W)  Current Parallel Port Size Register 
TPI_ACPR      EQU 0x010    ; Offset: 0x010 (R/W)  Asynchronous Clock Prescaler Register 
TPI_SPPR      EQU 0x0F0    ; Offset: 0x0F0 (R/W)  Selected Pin Protocol Register 
TPI_FFSR      EQU 0x300    ; Offset: 0x300 (R/ )  Formatter and Flush Status Register 
TPI_FFCR      EQU 0x304    ; Offset: 0x304 (R/W)  Formatter and Flush Control Register 
TPI_FSCR      EQU 0x308    ; Offset: 0x308 (R/ )  Formatter Synchronization Counter Register 
TPI_TRIGGER   EQU 0xEE8    ; Offset: 0xEE8 (R/ )  TRIGGER 
TPI_FIFO0     EQU 0xEEC    ; Offset: 0xEEC (R/ )  Integration ETM Data 
TPI_ITATBCTR2 EQU 0xEF0    ; Offset: 0xEF0 (R/ )  ITATBCTR2 
TPI_ITATBCTR0 EQU 0xEF8    ; Offset: 0xEF8 (R/ )  ITATBCTR0 
TPI_FIFO1     EQU 0xEFC    ; Offset: 0xEFC (R/ )  Integration ITM Data 
TPI_ITCTRL    EQU 0xF00    ; Offset: 0xF00 (R/W)  Integration Mode Control 
TPI_CLAIMSET  EQU 0xFA0    ; Offset: 0xFA0 (R/W)  Claim tag set 
TPI_CLAIMCLR  EQU 0xFA4    ; Offset: 0xFA4 (R/W)  Claim tag clear 
TPI_DEVID     EQU 0xFC8    ; Offset: 0xFC8 (R/ )  TPIU_DEVID 
TPI_DEVTYPE   EQU 0xFCC    ; Offset: 0xFCC (R/ )  TPIU_DEVTYPE 

; TPI Asynchronous Clock Prescaler Register Definitions 
TPI_ACPR_PRESCALER_Pos             EQU  0                                          ; TPI ACPR: PRESCALER Position 
TPI_ACPR_PRESCALER_Msk             EQU (0x1FFF << TPI_ACPR_PRESCALER_Pos)        ; TPI ACPR: PRESCALER Mask 

; TPI Selected Pin Protocol Register Definitions 
TPI_SPPR_TXMODE_Pos                EQU  0                                          ; TPI SPPR: TXMODE Position 
TPI_SPPR_TXMODE_Msk                EQU (0x3 << TPI_SPPR_TXMODE_Pos)              ; TPI SPPR: TXMODE Mask 

; TPI Formatter and Flush Status Register Definitions 
TPI_FFSR_FtNonStop_Pos             EQU  3                                          ; TPI FFSR: FtNonStop Position 
TPI_FFSR_FtNonStop_Msk             EQU (0x1 << TPI_FFSR_FtNonStop_Pos)           ; TPI FFSR: FtNonStop Mask 

TPI_FFSR_TCPresent_Pos             EQU  2                                          ; TPI FFSR: TCPresent Position 
TPI_FFSR_TCPresent_Msk             EQU (0x1 << TPI_FFSR_TCPresent_Pos)           ; TPI FFSR: TCPresent Mask 

TPI_FFSR_FtStopped_Pos             EQU  1                                          ; TPI FFSR: FtStopped Position 
TPI_FFSR_FtStopped_Msk             EQU (0x1 << TPI_FFSR_FtStopped_Pos)           ; TPI FFSR: FtStopped Mask 

TPI_FFSR_FlInProg_Pos              EQU  0                                          ; TPI FFSR: FlInProg Position 
TPI_FFSR_FlInProg_Msk              EQU (0x1 << TPI_FFSR_FlInProg_Pos)            ; TPI FFSR: FlInProg Mask 

; TPI Formatter and Flush Control Register Definitions 
TPI_FFCR_TrigIn_Pos                EQU  8                                          ; TPI FFCR: TrigIn Position 
TPI_FFCR_TrigIn_Msk                EQU (0x1 << TPI_FFCR_TrigIn_Pos)              ; TPI FFCR: TrigIn Mask 

TPI_FFCR_EnFCont_Pos               EQU  1                                          ; TPI FFCR: EnFCont Position 
TPI_FFCR_EnFCont_Msk               EQU (0x1 << TPI_FFCR_EnFCont_Pos)             ; TPI FFCR: EnFCont Mask 

; TPI TRIGGER Register Definitions 
TPI_TRIGGER_TRIGGER_Pos            EQU  0                                          ; TPI TRIGGER: TRIGGER Position 
TPI_TRIGGER_TRIGGER_Msk            EQU (0x1 << TPI_TRIGGER_TRIGGER_Pos)          ; TPI TRIGGER: TRIGGER Mask 

; TPI Integration ETM Data Register Definitions (FIFO0) 
TPI_FIFO0_ITM_ATVALID_Pos          EQU 29                                          ; TPI FIFO0: ITM_ATVALID Position 
TPI_FIFO0_ITM_ATVALID_Msk          EQU (0x3 << TPI_FIFO0_ITM_ATVALID_Pos)        ; TPI FIFO0: ITM_ATVALID Mask 

TPI_FIFO0_ITM_bytecount_Pos        EQU 27                                          ; TPI FIFO0: ITM_bytecount Position 
TPI_FIFO0_ITM_bytecount_Msk        EQU (0x3 << TPI_FIFO0_ITM_bytecount_Pos)      ; TPI FIFO0: ITM_bytecount Mask 

TPI_FIFO0_ETM_ATVALID_Pos          EQU 26                                          ; TPI FIFO0: ETM_ATVALID Position 
TPI_FIFO0_ETM_ATVALID_Msk          EQU (0x3 << TPI_FIFO0_ETM_ATVALID_Pos)        ; TPI FIFO0: ETM_ATVALID Mask 

TPI_FIFO0_ETM_bytecount_Pos        EQU 24                                          ; TPI FIFO0: ETM_bytecount Position 
TPI_FIFO0_ETM_bytecount_Msk        EQU (0x3 << TPI_FIFO0_ETM_bytecount_Pos)      ; TPI FIFO0: ETM_bytecount Mask 

TPI_FIFO0_ETM2_Pos                 EQU 16                                          ; TPI FIFO0: ETM2 Position 
TPI_FIFO0_ETM2_Msk                 EQU (0xFF << TPI_FIFO0_ETM2_Pos)              ; TPI FIFO0: ETM2 Mask 

TPI_FIFO0_ETM1_Pos                 EQU  8                                          ; TPI FIFO0: ETM1 Position 
TPI_FIFO0_ETM1_Msk                 EQU (0xFF << TPI_FIFO0_ETM1_Pos)              ; TPI FIFO0: ETM1 Mask 

TPI_FIFO0_ETM0_Pos                 EQU  0                                          ; TPI FIFO0: ETM0 Position 
TPI_FIFO0_ETM0_Msk                 EQU (0xFF << TPI_FIFO0_ETM0_Pos)              ; TPI FIFO0: ETM0 Mask 

; TPI ITATBCTR2 Register Definitions 
TPI_ITATBCTR2_ATREADY_Pos          EQU  0                                          ; TPI ITATBCTR2: ATREADY Position 
TPI_ITATBCTR2_ATREADY_Msk          EQU (0x1 << TPI_ITATBCTR2_ATREADY_Pos)        ; TPI ITATBCTR2: ATREADY Mask 

; TPI Integration ITM Data Register Definitions (FIFO1) 
TPI_FIFO1_ITM_ATVALID_Pos          EQU 29                                          ; TPI FIFO1: ITM_ATVALID Position 
TPI_FIFO1_ITM_ATVALID_Msk          EQU (0x3 << TPI_FIFO1_ITM_ATVALID_Pos)        ; TPI FIFO1: ITM_ATVALID Mask 

TPI_FIFO1_ITM_bytecount_Pos        EQU 27                                          ; TPI FIFO1: ITM_bytecount Position 
TPI_FIFO1_ITM_bytecount_Msk        EQU (0x3 << TPI_FIFO1_ITM_bytecount_Pos)      ; TPI FIFO1: ITM_bytecount Mask 

TPI_FIFO1_ETM_ATVALID_Pos          EQU 26                                          ; TPI FIFO1: ETM_ATVALID Position 
TPI_FIFO1_ETM_ATVALID_Msk          EQU (0x3 << TPI_FIFO1_ETM_ATVALID_Pos)        ; TPI FIFO1: ETM_ATVALID Mask 

TPI_FIFO1_ETM_bytecount_Pos        EQU 24                                          ; TPI FIFO1: ETM_bytecount Position 
TPI_FIFO1_ETM_bytecount_Msk        EQU (0x3 << TPI_FIFO1_ETM_bytecount_Pos)      ; TPI FIFO1: ETM_bytecount Mask 

TPI_FIFO1_ITM2_Pos                 EQU 16                                          ; TPI FIFO1: ITM2 Position 
TPI_FIFO1_ITM2_Msk                 EQU (0xFF << TPI_FIFO1_ITM2_Pos)              ; TPI FIFO1: ITM2 Mask 

TPI_FIFO1_ITM1_Pos                 EQU  8                                          ; TPI FIFO1: ITM1 Position 
TPI_FIFO1_ITM1_Msk                 EQU (0xFF << TPI_FIFO1_ITM1_Pos)              ; TPI FIFO1: ITM1 Mask 

TPI_FIFO1_ITM0_Pos                 EQU  0                                          ; TPI FIFO1: ITM0 Position 
TPI_FIFO1_ITM0_Msk                 EQU (0xFF << TPI_FIFO1_ITM0_Pos)              ; TPI FIFO1: ITM0 Mask 

; TPI ITATBCTR0 Register Definitions 
TPI_ITATBCTR0_ATREADY_Pos          EQU  0                                          ; TPI ITATBCTR0: ATREADY Position 
TPI_ITATBCTR0_ATREADY_Msk          EQU (0x1 << TPI_ITATBCTR0_ATREADY_Pos)        ; TPI ITATBCTR0: ATREADY Mask 

; TPI Integration Mode Control Register Definitions 
TPI_ITCTRL_Mode_Pos                EQU  0                                          ; TPI ITCTRL: Mode Position 
TPI_ITCTRL_Mode_Msk                EQU (0x1 << TPI_ITCTRL_Mode_Pos)              ; TPI ITCTRL: Mode Mask 

; TPI DEVID Register Definitions 
TPI_DEVID_NRZVALID_Pos             EQU 11                                          ; TPI DEVID: NRZVALID Position 
TPI_DEVID_NRZVALID_Msk             EQU (0x1 << TPI_DEVID_NRZVALID_Pos)           ; TPI DEVID: NRZVALID Mask 

TPI_DEVID_MANCVALID_Pos            EQU 10                                          ; TPI DEVID: MANCVALID Position 
TPI_DEVID_MANCVALID_Msk            EQU (0x1 << TPI_DEVID_MANCVALID_Pos)          ; TPI DEVID: MANCVALID Mask 

TPI_DEVID_PTINVALID_Pos            EQU  9                                          ; TPI DEVID: PTINVALID Position 
TPI_DEVID_PTINVALID_Msk            EQU (0x1 << TPI_DEVID_PTINVALID_Pos)          ; TPI DEVID: PTINVALID Mask 

TPI_DEVID_MinBufSz_Pos             EQU  6                                          ; TPI DEVID: MinBufSz Position 
TPI_DEVID_MinBufSz_Msk             EQU (0x7 << TPI_DEVID_MinBufSz_Pos)           ; TPI DEVID: MinBufSz Mask 

TPI_DEVID_AsynClkIn_Pos            EQU  5                                          ; TPI DEVID: AsynClkIn Position 
TPI_DEVID_AsynClkIn_Msk            EQU (0x1 << TPI_DEVID_AsynClkIn_Pos)          ; TPI DEVID: AsynClkIn Mask 

TPI_DEVID_NrTraceInput_Pos         EQU  0                                          ; TPI DEVID: NrTraceInput Position 
TPI_DEVID_NrTraceInput_Msk         EQU (0x1F << TPI_DEVID_NrTraceInput_Pos)      ; TPI DEVID: NrTraceInput Mask 

; TPI DEVTYPE Register Definitions 
TPI_DEVTYPE_SubType_Pos            EQU  0                                          ; TPI DEVTYPE: SubType Position 
TPI_DEVTYPE_SubType_Msk            EQU (0xF << TPI_DEVTYPE_SubType_Pos)          ; TPI DEVTYPE: SubType Mask 

TPI_DEVTYPE_MajorType_Pos          EQU  4                                          ; TPI DEVTYPE: MajorType Position 
TPI_DEVTYPE_MajorType_Msk          EQU (0xF << TPI_DEVTYPE_MajorType_Pos)        ; TPI DEVTYPE: MajorType Mask 


;CMSIS_core_register
; CMSIS_MPU     Memory Protection Unit (MPU)
; Type definitions for the Memory Protection Unit (MPU)
; Structure type to access the Memory Protection Unit (MPU).

MPU_TYPE     EQU 0x000      ; Offset: 0x000 (R/ )  MPU Type Register                              
MPU_CTRL     EQU 0x004      ; Offset: 0x004 (R/W)  MPU Control Register                           
MPU_RNR      EQU 0x008      ; Offset: 0x008 (R/W)  MPU Region RNRber Register                     
MPU_RBAR     EQU 0x00C      ; Offset: 0x00C (R/W)  MPU Region Base Address Register               
MPU_RASR     EQU 0x010      ; Offset: 0x010 (R/W)  MPU Region Attribute and Size Register         
MPU_RBAR_A1  EQU 0x014      ; Offset: 0x014 (R/W)  MPU Alias 1 Region Base Address Register       
MPU_RASR_A1  EQU 0x018      ; Offset: 0x018 (R/W)  MPU Alias 1 Region Attribute and Size Register 
MPU_RBAR_A2  EQU 0x01C      ; Offset: 0x01C (R/W)  MPU Alias 2 Region Base Address Register       
MPU_RASR_A2  EQU 0x020      ; Offset: 0x020 (R/W)  MPU Alias 2 Region Attribute and Size Register 
MPU_RBAR_A3  EQU 0x024      ; Offset: 0x024 (R/W)  MPU Alias 3 Region Base Address Register       
MPU_RASR_A3  EQU 0x028      ; Offset: 0x028 (R/W)  MPU Alias 3 Region Attribute and Size Register 

; MPU Type Register 
MPU_TYPE_IREGION_Pos               EQU 16                                             ; MPU TYPE: IREGION Position 
MPU_TYPE_IREGION_Msk               EQU (0xFF << MPU_TYPE_IREGION_Pos)               ; MPU TYPE: IREGION Mask 

MPU_TYPE_DREGION_Pos               EQU  8                                             ; MPU TYPE: DREGION Position 
MPU_TYPE_DREGION_Msk               EQU (0xFF << MPU_TYPE_DREGION_Pos)               ; MPU TYPE: DREGION Mask 

MPU_TYPE_SEPARATE_Pos              EQU  0                                             ; MPU TYPE: SEPARATE Position 
MPU_TYPE_SEPARATE_Msk              EQU (0x1 << MPU_TYPE_SEPARATE_Pos)                 ; MPU TYPE: SEPARATE Mask 

; MPU Control Register 
MPU_CTRL_PRIVDEFENA_Pos            EQU  2                                             ; MPU CTRL: PRIVDEFENA Position 
MPU_CTRL_PRIVDEFENA_Msk            EQU (0x1 << MPU_CTRL_PRIVDEFENA_Pos)               ; MPU CTRL: PRIVDEFENA Mask 

MPU_CTRL_HFNMIENA_Pos              EQU  1                                             ; MPU CTRL: HFNMIENA Position 
MPU_CTRL_HFNMIENA_Msk              EQU (0x1 << MPU_CTRL_HFNMIENA_Pos)                 ; MPU CTRL: HFNMIENA Mask 

MPU_CTRL_ENABLE_Pos                EQU  0                                             ; MPU CTRL: ENABLE Position 
MPU_CTRL_ENABLE_Msk                EQU (0x1 << MPU_CTRL_ENABLE_Pos)                   ; MPU CTRL: ENABLE Mask 

; MPU Region Number Register 
MPU_RNR_REGION_Pos                 EQU  0                                             ; MPU RNR: REGION Position 
MPU_RNR_REGION_Msk                 EQU (0xFF << MPU_RNR_REGION_Pos)                 ; MPU RNR: REGION Mask 

; MPU Region Base Address Register 
MPU_RBAR_ADDR_Pos                  EQU  5                                             ; MPU RBAR: ADDR Position 
MPU_RBAR_ADDR_Msk                  EQU (0x7FFFFFF << MPU_RBAR_ADDR_Pos)             ; MPU RBAR: ADDR Mask 

MPU_RBAR_VALID_Pos                 EQU  4                                             ; MPU RBAR: VALID Position 
MPU_RBAR_VALID_Msk                 EQU (0x1 << MPU_RBAR_VALID_Pos)                    ; MPU RBAR: VALID Mask 

MPU_RBAR_REGION_Pos                EQU  0                                             ; MPU RBAR: REGION Position 
MPU_RBAR_REGION_Msk                EQU (0xF << MPU_RBAR_REGION_Pos)                 ; MPU RBAR: REGION Mask 

; MPU Region Attribute and Size Register 
MPU_RASR_ATTRS_Pos                 EQU 16                                             ; MPU RASR: MPU Region Attribute field Position 
MPU_RASR_ATTRS_Msk                 EQU (0xFFFF << MPU_RASR_ATTRS_Pos)               ; MPU RASR: MPU Region Attribute field Mask 

MPU_RASR_XN_Pos                    EQU 28                                             ; MPU RASR: ATTRS.XN Position 
MPU_RASR_XN_Msk                    EQU (0x1 << MPU_RASR_XN_Pos)                       ; MPU RASR: ATTRS.XN Mask 

MPU_RASR_AP_Pos                    EQU 24                                             ; MPU RASR: ATTRS.AP Position 
MPU_RASR_AP_Msk                    EQU (0x7 << MPU_RASR_AP_Pos)                     ; MPU RASR: ATTRS.AP Mask 

MPU_RASR_TEX_Pos                   EQU 19                                             ; MPU RASR: ATTRS.TEX Position 
MPU_RASR_TEX_Msk                   EQU (0x7 << MPU_RASR_TEX_Pos)                    ; MPU RASR: ATTRS.TEX Mask 

MPU_RASR_S_Pos                     EQU 18                                             ; MPU RASR: ATTRS.S Position 
MPU_RASR_S_Msk                     EQU (0x1 << MPU_RASR_S_Pos)                        ; MPU RASR: ATTRS.S Mask 

MPU_RASR_C_Pos                     EQU 17                                             ; MPU RASR: ATTRS.C Position 
MPU_RASR_C_Msk                     EQU (0x1 << MPU_RASR_C_Pos)                        ; MPU RASR: ATTRS.C Mask 

MPU_RASR_B_Pos                     EQU 16                                             ; MPU RASR: ATTRS.B Position 
MPU_RASR_B_Msk                     EQU (0x1 << MPU_RASR_B_Pos)                        ; MPU RASR: ATTRS.B Mask 

MPU_RASR_SRD_Pos                   EQU  8                                             ; MPU RASR: Sub-Region Disable Position 
MPU_RASR_SRD_Msk                   EQU (0xFF << MPU_RASR_SRD_Pos)                   ; MPU RASR: Sub-Region Disable Mask 

MPU_RASR_SIZE_Pos                  EQU  1                                             ; MPU RASR: Region Size Field Position 
MPU_RASR_SIZE_Msk                  EQU (0x1F << MPU_RASR_SIZE_Pos)                  ; MPU RASR: Region Size Field Mask 

MPU_RASR_ENABLE_Pos                EQU  0                                             ; MPU RASR: Region enable bit Position 
MPU_RASR_ENABLE_Msk                EQU (0x1 << MPU_RASR_ENABLE_Pos)                   ; MPU RASR: Region enable bit Disable Mask 


;CMSIS_core_register
; CMSIS_FPU     Floating Point Unit (FPU)
; Type definitions for the Floating Point Unit (FPU)
; Structure type to access the Floating Point Unit (FPU).

FPU_FPCCR  EQU 0x004                   ; Offset: 0x004 (R/W)  Floating-Point Context Control Register               
FPU_FPCAR  EQU 0x008                   ; Offset: 0x008 (R/W)  Floating-Point Context Address Register               
FPU_FPDSCR EQU 0x00C                   ; Offset: 0x00C (R/W)  Floating-Point Default Status Control Register        
FPU_MVFR0  EQU 0x010                   ; Offset: 0x010 (R/ )  Media and FP Feature Register 0                       
FPU_MVFR1  EQU 0x014                   ; Offset: 0x014 (R/ )  Media and FP Feature Register 1                       

; Floating-Point Context Control Register 
FPU_FPCCR_ASPEN_Pos                EQU 31                                             ; FPCCR: ASPEN bit Position 
FPU_FPCCR_ASPEN_Msk                EQU (0x1 << FPU_FPCCR_ASPEN_Pos)                   ; FPCCR: ASPEN bit Mask 

FPU_FPCCR_LSPEN_Pos                EQU 30                                             ; FPCCR: LSPEN Position 
FPU_FPCCR_LSPEN_Msk                EQU (0x1 << FPU_FPCCR_LSPEN_Pos)                   ; FPCCR: LSPEN bit Mask 

FPU_FPCCR_MONRDY_Pos               EQU  8                                             ; FPCCR: MONRDY Position 
FPU_FPCCR_MONRDY_Msk               EQU (0x1 << FPU_FPCCR_MONRDY_Pos)                  ; FPCCR: MONRDY bit Mask 

FPU_FPCCR_BFRDY_Pos                EQU  6                                             ; FPCCR: BFRDY Position 
FPU_FPCCR_BFRDY_Msk                EQU (0x1 << FPU_FPCCR_BFRDY_Pos)                   ; FPCCR: BFRDY bit Mask 

FPU_FPCCR_MMRDY_Pos                EQU  5                                             ; FPCCR: MMRDY Position 
FPU_FPCCR_MMRDY_Msk                EQU (0x1 << FPU_FPCCR_MMRDY_Pos)                   ; FPCCR: MMRDY bit Mask 

FPU_FPCCR_HFRDY_Pos                EQU  4                                             ; FPCCR: HFRDY Position 
FPU_FPCCR_HFRDY_Msk                EQU (0x1 << FPU_FPCCR_HFRDY_Pos)                   ; FPCCR: HFRDY bit Mask 

FPU_FPCCR_THREAD_Pos               EQU  3                                             ; FPCCR: processor mode bit Position 
FPU_FPCCR_THREAD_Msk               EQU (0x1 << FPU_FPCCR_THREAD_Pos)                  ; FPCCR: processor mode active bit Mask 

FPU_FPCCR_USER_Pos                 EQU  1                                             ; FPCCR: privilege level bit Position 
FPU_FPCCR_USER_Msk                 EQU (0x1 << FPU_FPCCR_USER_Pos)                    ; FPCCR: privilege level bit Mask 

FPU_FPCCR_LSPACT_Pos               EQU  0                                             ; FPCCR: Lazy state preservation active bit Position 
FPU_FPCCR_LSPACT_Msk               EQU (0x1 << FPU_FPCCR_LSPACT_Pos)                  ; FPCCR: Lazy state preservation active bit Mask 

; Floating-Point Context Address Register 
FPU_FPCAR_ADDRESS_Pos              EQU  3                                             ; FPCAR: ADDRESS bit Position 
FPU_FPCAR_ADDRESS_Msk              EQU (0x1FFFFFFF << FPU_FPCAR_ADDRESS_Pos)        ; FPCAR: ADDRESS bit Mask 

; Floating-Point Default Status Control Register 
FPU_FPDSCR_AHP_Pos                 EQU 26                                             ; FPDSCR: AHP bit Position 
FPU_FPDSCR_AHP_Msk                 EQU (0x1 << FPU_FPDSCR_AHP_Pos)                    ; FPDSCR: AHP bit Mask 

FPU_FPDSCR_DN_Pos                  EQU 25                                             ; FPDSCR: DN bit Position 
FPU_FPDSCR_DN_Msk                  EQU (0x1 << FPU_FPDSCR_DN_Pos)                     ; FPDSCR: DN bit Mask 

FPU_FPDSCR_FZ_Pos                  EQU 24                                             ; FPDSCR: FZ bit Position 
FPU_FPDSCR_FZ_Msk                  EQU (0x1 << FPU_FPDSCR_FZ_Pos)                     ; FPDSCR: FZ bit Mask 

FPU_FPDSCR_RMode_Pos               EQU 22                                             ; FPDSCR: RMode bit Position 
FPU_FPDSCR_RMode_Msk               EQU (3 << FPU_FPDSCR_RMode_Pos)                  ; FPDSCR: RMode bit Mask 

; Media and FP Feature Register 0 
FPU_MVFR0_FP_rounding_modes_Pos    EQU 28                                             ; MVFR0: FP rounding modes bits Position 
FPU_MVFR0_FP_rounding_modes_Msk    EQU (0xF << FPU_MVFR0_FP_rounding_modes_Pos)     ; MVFR0: FP rounding modes bits Mask 

FPU_MVFR0_Short_vectors_Pos        EQU 24                                             ; MVFR0: Short vectors bits Position 
FPU_MVFR0_Short_vectors_Msk        EQU (0xF << FPU_MVFR0_Short_vectors_Pos)         ; MVFR0: Short vectors bits Mask 

FPU_MVFR0_Square_root_Pos          EQU 20                                             ; MVFR0: Square root bits Position 
FPU_MVFR0_Square_root_Msk          EQU (0xF << FPU_MVFR0_Square_root_Pos)           ; MVFR0: Square root bits Mask 

FPU_MVFR0_Divide_Pos               EQU 16                                             ; MVFR0: Divide bits Position 
FPU_MVFR0_Divide_Msk               EQU (0xF << FPU_MVFR0_Divide_Pos)                ; MVFR0: Divide bits Mask 

FPU_MVFR0_FP_excep_trapping_Pos    EQU 12                                             ; MVFR0: FP exception trapping bits Position 
FPU_MVFR0_FP_excep_trapping_Msk    EQU (0xF << FPU_MVFR0_FP_excep_trapping_Pos)     ; MVFR0: FP exception trapping bits Mask 

FPU_MVFR0_Double_precision_Pos     EQU  8                                             ; MVFR0: Double-precision bits Position 
FPU_MVFR0_Double_precision_Msk     EQU (0xF << FPU_MVFR0_Double_precision_Pos)      ; MVFR0: Double-precision bits Mask 

FPU_MVFR0_Single_precision_Pos     EQU  4                                             ; MVFR0: Single-precision bits Position 
FPU_MVFR0_Single_precision_Msk     EQU (0xF << FPU_MVFR0_Single_precision_Pos)      ; MVFR0: Single-precision bits Mask 

FPU_MVFR0_A_SIMD_registers_Pos     EQU  0                                             ; MVFR0: A_SIMD registers bits Position 
FPU_MVFR0_A_SIMD_registers_Msk     EQU (0xF << FPU_MVFR0_A_SIMD_registers_Pos)      ; MVFR0: A_SIMD registers bits Mask 

; Media and FP Feature Register 1 
FPU_MVFR1_FP_fused_MAC_Pos         EQU 28                                             ; MVFR1: FP fused MAC bits Position 
FPU_MVFR1_FP_fused_MAC_Msk         EQU (0xF << FPU_MVFR1_FP_fused_MAC_Pos)          ; MVFR1: FP fused MAC bits Mask 

FPU_MVFR1_FP_HPFP_Pos              EQU 24                                             ; MVFR1: FP HPFP bits Position 
FPU_MVFR1_FP_HPFP_Msk              EQU (0xF << FPU_MVFR1_FP_HPFP_Pos)               ; MVFR1: FP HPFP bits Mask 

FPU_MVFR1_D_NaN_mode_Pos           EQU  4                                             ; MVFR1: D_NaN mode bits Position 
FPU_MVFR1_D_NaN_mode_Msk           EQU (0xF << FPU_MVFR1_D_NaN_mode_Pos)            ; MVFR1: D_NaN mode bits Mask 

FPU_MVFR1_FtZ_mode_Pos             EQU  0                                             ; MVFR1: FtZ mode bits Position 
FPU_MVFR1_FtZ_mode_Msk             EQU (0xF << FPU_MVFR1_FtZ_mode_Pos)              ; MVFR1: FtZ mode bits Mask 

; CMSIS_core_register
; CMSIS_CoreDebug       Core Debug Registers (CoreDebug)
; Type definitions for the Core Debug Registers
; Structure type to access the Core Debug Register (CoreDebug).

CoreDebug_DHCSR  EQU 0x000           ; Offset: 0x000 (R/W)  Debug Halting Control and Status Register    
CoreDebug_DCRSR  EQU 0x004           ; Offset: 0x004 ( /W)  Debug Core Register Selector Register        
CoreDebug_DCRDR  EQU 0x008           ; Offset: 0x008 (R/W)  Debug Core Register Data Register            
CoreDebug_DEMCR  EQU 0x00C           ; Offset: 0x00C (R/W)  Debug Exception and Monitor Control Register 

; Debug Halting Control and Status Register 
CoreDebug_DHCSR_DBGKEY_Pos         EQU 16                                             ; CoreDebug DHCSR: DBGKEY Position 
CoreDebug_DHCSR_DBGKEY_Msk         EQU (0xFFFF << CoreDebug_DHCSR_DBGKEY_Pos)       ; CoreDebug DHCSR: DBGKEY Mask 

CoreDebug_DHCSR_S_RESET_ST_Pos     EQU 25                                             ; CoreDebug DHCSR: S_RESET_ST Position 
CoreDebug_DHCSR_S_RESET_ST_Msk     EQU (0x1 << CoreDebug_DHCSR_S_RESET_ST_Pos)        ; CoreDebug DHCSR: S_RESET_ST Mask 

CoreDebug_DHCSR_S_RETIRE_ST_Pos    EQU 24                                             ; CoreDebug DHCSR: S_RETIRE_ST Position 
CoreDebug_DHCSR_S_RETIRE_ST_Msk    EQU (0x1 << CoreDebug_DHCSR_S_RETIRE_ST_Pos)       ; CoreDebug DHCSR: S_RETIRE_ST Mask 

CoreDebug_DHCSR_S_LOCKUP_Pos       EQU 19                                             ; CoreDebug DHCSR: S_LOCKUP Position 
CoreDebug_DHCSR_S_LOCKUP_Msk       EQU (0x1 << CoreDebug_DHCSR_S_LOCKUP_Pos)          ; CoreDebug DHCSR: S_LOCKUP Mask 

CoreDebug_DHCSR_S_SLEEP_Pos        EQU 18                                             ; CoreDebug DHCSR: S_SLEEP Position 
CoreDebug_DHCSR_S_SLEEP_Msk        EQU (0x1 << CoreDebug_DHCSR_S_SLEEP_Pos)           ; CoreDebug DHCSR: S_SLEEP Mask 

CoreDebug_DHCSR_S_HALT_Pos         EQU 17                                             ; CoreDebug DHCSR: S_HALT Position 
CoreDebug_DHCSR_S_HALT_Msk         EQU (0x1 << CoreDebug_DHCSR_S_HALT_Pos)            ; CoreDebug DHCSR: S_HALT Mask 

CoreDebug_DHCSR_S_REGRDY_Pos       EQU 16                                             ; CoreDebug DHCSR: S_REGRDY Position 
CoreDebug_DHCSR_S_REGRDY_Msk       EQU (0x1 << CoreDebug_DHCSR_S_REGRDY_Pos)          ; CoreDebug DHCSR: S_REGRDY Mask 

CoreDebug_DHCSR_C_SNAPSTALL_Pos    EQU  5                                             ; CoreDebug DHCSR: C_SNAPSTALL Position 
CoreDebug_DHCSR_C_SNAPSTALL_Msk    EQU (0x1 << CoreDebug_DHCSR_C_SNAPSTALL_Pos)       ; CoreDebug DHCSR: C_SNAPSTALL Mask 

CoreDebug_DHCSR_C_MASKINTS_Pos     EQU  3                                             ; CoreDebug DHCSR: C_MASKINTS Position 
CoreDebug_DHCSR_C_MASKINTS_Msk     EQU (0x1 << CoreDebug_DHCSR_C_MASKINTS_Pos)        ; CoreDebug DHCSR: C_MASKINTS Mask 

CoreDebug_DHCSR_C_STEP_Pos         EQU  2                                             ; CoreDebug DHCSR: C_STEP Position 
CoreDebug_DHCSR_C_STEP_Msk         EQU (0x1 << CoreDebug_DHCSR_C_STEP_Pos)            ; CoreDebug DHCSR: C_STEP Mask 

CoreDebug_DHCSR_C_HALT_Pos         EQU  1                                             ; CoreDebug DHCSR: C_HALT Position 
CoreDebug_DHCSR_C_HALT_Msk         EQU (0x1 << CoreDebug_DHCSR_C_HALT_Pos)            ; CoreDebug DHCSR: C_HALT Mask 

CoreDebug_DHCSR_C_DEBUGEN_Pos      EQU  0                                             ; CoreDebug DHCSR: C_DEBUGEN Position 
CoreDebug_DHCSR_C_DEBUGEN_Msk      EQU (0x1 << CoreDebug_DHCSR_C_DEBUGEN_Pos)         ; CoreDebug DHCSR: C_DEBUGEN Mask 

; Debug Core Register Selector Register 
CoreDebug_DCRSR_REGWnR_Pos         EQU 16                                             ; CoreDebug DCRSR: REGWnR Position 
CoreDebug_DCRSR_REGWnR_Msk         EQU (0x1 << CoreDebug_DCRSR_REGWnR_Pos)            ; CoreDebug DCRSR: REGWnR Mask 

CoreDebug_DCRSR_REGSEL_Pos         EQU  0                                             ; CoreDebug DCRSR: REGSEL Position 
CoreDebug_DCRSR_REGSEL_Msk         EQU (0x1F << CoreDebug_DCRSR_REGSEL_Pos)         ; CoreDebug DCRSR: REGSEL Mask 

; Debug Exception and Monitor Control Register 
CoreDebug_DEMCR_TRCENA_Pos         EQU 24                                             ; CoreDebug DEMCR: TRCENA Position 
CoreDebug_DEMCR_TRCENA_Msk         EQU (0x1 << CoreDebug_DEMCR_TRCENA_Pos)            ; CoreDebug DEMCR: TRCENA Mask 

CoreDebug_DEMCR_MON_REQ_Pos        EQU 19                                             ; CoreDebug DEMCR: MON_REQ Position 
CoreDebug_DEMCR_MON_REQ_Msk        EQU (0x1 << CoreDebug_DEMCR_MON_REQ_Pos)           ; CoreDebug DEMCR: MON_REQ Mask 

CoreDebug_DEMCR_MON_STEP_Pos       EQU 18                                             ; CoreDebug DEMCR: MON_STEP Position 
CoreDebug_DEMCR_MON_STEP_Msk       EQU (0x1 << CoreDebug_DEMCR_MON_STEP_Pos)          ; CoreDebug DEMCR: MON_STEP Mask 

CoreDebug_DEMCR_MON_PEND_Pos       EQU 17                                             ; CoreDebug DEMCR: MON_PEND Position 
CoreDebug_DEMCR_MON_PEND_Msk       EQU (0x1 << CoreDebug_DEMCR_MON_PEND_Pos)          ; CoreDebug DEMCR: MON_PEND Mask 

CoreDebug_DEMCR_MON_EN_Pos         EQU 16                                             ; CoreDebug DEMCR: MON_EN Position 
CoreDebug_DEMCR_MON_EN_Msk         EQU (0x1 << CoreDebug_DEMCR_MON_EN_Pos)            ; CoreDebug DEMCR: MON_EN Mask 

CoreDebug_DEMCR_VC_HARDERR_Pos     EQU 10                                             ; CoreDebug DEMCR: VC_HARDERR Position 
CoreDebug_DEMCR_VC_HARDERR_Msk     EQU (0x1 << CoreDebug_DEMCR_VC_HARDERR_Pos)        ; CoreDebug DEMCR: VC_HARDERR Mask 

CoreDebug_DEMCR_VC_INTERR_Pos      EQU  9                                             ; CoreDebug DEMCR: VC_INTERR Position 
CoreDebug_DEMCR_VC_INTERR_Msk      EQU (0x1 << CoreDebug_DEMCR_VC_INTERR_Pos)         ; CoreDebug DEMCR: VC_INTERR Mask 

CoreDebug_DEMCR_VC_BUSERR_Pos      EQU  8                                             ; CoreDebug DEMCR: VC_BUSERR Position 
CoreDebug_DEMCR_VC_BUSERR_Msk      EQU (0x1 << CoreDebug_DEMCR_VC_BUSERR_Pos)         ; CoreDebug DEMCR: VC_BUSERR Mask 

CoreDebug_DEMCR_VC_STATERR_Pos     EQU  7                                             ; CoreDebug DEMCR: VC_STATERR Position 
CoreDebug_DEMCR_VC_STATERR_Msk     EQU (0x1 << CoreDebug_DEMCR_VC_STATERR_Pos)        ; CoreDebug DEMCR: VC_STATERR Mask 

CoreDebug_DEMCR_VC_CHKERR_Pos      EQU  6                                             ; CoreDebug DEMCR: VC_CHKERR Position 
CoreDebug_DEMCR_VC_CHKERR_Msk      EQU (0x1 << CoreDebug_DEMCR_VC_CHKERR_Pos)         ; CoreDebug DEMCR: VC_CHKERR Mask 

CoreDebug_DEMCR_VC_NOCPERR_Pos     EQU  5                                             ; CoreDebug DEMCR: VC_NOCPERR Position 
CoreDebug_DEMCR_VC_NOCPERR_Msk     EQU (0x1 << CoreDebug_DEMCR_VC_NOCPERR_Pos)        ; CoreDebug DEMCR: VC_NOCPERR Mask 

CoreDebug_DEMCR_VC_MMERR_Pos       EQU  4                                             ; CoreDebug DEMCR: VC_MMERR Position 
CoreDebug_DEMCR_VC_MMERR_Msk       EQU (0x1 << CoreDebug_DEMCR_VC_MMERR_Pos)          ; CoreDebug DEMCR: VC_MMERR Mask 

CoreDebug_DEMCR_VC_CORERESET_Pos   EQU  0                                             ; CoreDebug DEMCR: VC_CORERESET Position 
CoreDebug_DEMCR_VC_CORERESET_Msk   EQU (0x1 << CoreDebug_DEMCR_VC_CORERESET_Pos)      ; CoreDebug DEMCR: VC_CORERESET Mask 

; CMSIS_core_register
; CMSIS_core_base     Core Definitions
; Definitions for base addresses, unions, and structures.

; Memory mapping of Cortex-M4 Hardware 
SCS_BASE            EQU (0xE000E000)                            ; System Control Space Base Address  
ITM_BASE            EQU (0xE0000000)                            ; ITM Base Address                   
DWT_BASE            EQU (0xE0001000)                            ; DWT Base Address                   
TPI_BASE            EQU (0xE0040000)                            ; TPI Base Address                   
CoreDebug_BASE      EQU (0xE000EDF0)                            ; Core Debug Base Address            
SysTick_BASE        EQU (SCS_BASE +  0x0010)                    ; SysTick Base Address               
NVIC_BASE           EQU (SCS_BASE +  0x0100)                    ; NVIC Base Address                  
SCB_BASE            EQU (SCS_BASE +  0x0D00)                    ; System Control Block Base Address  

SCnSCB              EQU (     SCS_BASE      )   ; System control Register not in SCB 
SCB                 EQU (     SCB_BASE      )   ; SCB configuration struct           
SysTick             equ (     SysTick_BASE  )   ; SysTick configuration struct       
NVIC                EQU (     NVIC_BASE     )   ; NVIC configuration struct          
ITM                 EQU (     ITM_BASE      )   ; ITM configuration struct           
DWT                 EQU (     DWT_BASE      )   ; DWT configuration struct           
TPI                 EQU (     TPI_BASE      )   ; TPI configuration struct           
CoreDebug           EQU (     CoreDebug_BASE)   ; Core Debug configuration struct    

MPU_BASE            EQU (SCS_BASE +  0x0D90)                    ; Memory Protection Unit             
MPU                 EQU (      MPU_BASE      )   ; Memory Protection Unit             

FPU_BASE            EQU (SCS_BASE +  0x0F30)                    ; Floating Point Unit                
FPU                 EQU (      FPU_BASE      )   ; Floating Point Unit                

; This following is added to remove the compiler warning.
    AREA    __DEFINES_STM32F4_CORE4_xx_DUMMY, CODE, READONLY
    END

