/* File: startup_ARMCM0.S
 * Purpose: startup file for Cortex-M0 devices. Should use with 
 *   GCC for ARM Embedded Processors
 * Version: V1.2
 * Date: 15 Nov 2011
 * 
 * Copyright (c) 2011, ARM Limited
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the ARM Limited nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ARM LIMITED BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
    .syntax unified
    .arch armv6-m

    .section .stack
    .align 3
#ifdef __STACK_SIZE
    .equ    Stack_Size, __STACK_SIZE
#else
    .equ    Stack_Size, 0xc00
#endif
    .globl    __StackTop
    .globl    __StackLimit
__StackLimit:
    .space    Stack_Size
    .size __StackLimit, . - __StackLimit
__StackTop:
    .size __StackTop, . - __StackTop

    .section .heap
    .align 3
#ifdef __HEAP_SIZE
    .equ    Heap_Size, __HEAP_SIZE
#else
    .equ    Heap_Size, 0x100
#endif
    .globl    __HeapBase
    .globl    __HeapLimit
__HeapBase:
    .space    Heap_Size
    .size __HeapBase, . - __HeapBase
__HeapLimit:
    .size __HeapLimit, . - __HeapLimit
    
    .macro vector_definition peripheral
\peripheral\()Vector :
    .long   \peripheral\()Wrapper
    .endm

    .section .isr_vector
    .align 2
    .globl __isr_vector
__isr_vector:
    .long    __StackTop            /* Top of Stack */
    .long    Reset_Handler         /* Reset Handler */
    .long    NMI_Handler           /* NMI Handler */
    vector_definition    HardFault_/* Hard Fault Handler */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    vector_definition    SVC_           /* SVCall Handler */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    vector_definition    PendSV_        /* PendSV Handler */
    vector_definition    SysTick_       /* SysTick Handler */

    /* External interrupts */
    vector_definition 	POWER_CLOCK_IRQ		 /*POWER_CLOCK */
    vector_definition 	RADIO_IRQ		 /*RADIO */
    vector_definition 	UART0_IRQ		 /*UART0 */
    vector_definition 	SPI0_TWI0_IRQ		 /*SPI0_TWI0 */
    vector_definition 	SPI1_TWI1_IRQ		 /*SPI1_TWI1 */
    .long 	0		 /*Reserved */
    vector_definition 	GPIOTE_IRQ		 /*GPIOTE */
    vector_definition 	ADC_IRQ		 /*ADC */
    vector_definition 	TIMER0_IRQ		 /*TIMER0 */
    vector_definition 	TIMER1_IRQ		 /*TIMER1 */
    vector_definition 	TIMER2_IRQ		 /*TIMER2 */
    vector_definition 	RTC0_IRQ		 /*RTC0 */
    vector_definition 	TEMP_IRQ		 /*TEMP */
    vector_definition 	RNG_IRQ		 /*RNG */
    vector_definition 	ECB_IRQ		 /*ECB */
    vector_definition 	CCM_AAR_IRQ		 /*CCM_AAR */
    vector_definition 	WDT_IRQ		 /*WDT */
    vector_definition 	RTC1_IRQ		 /*RTC1 */
    vector_definition 	QDEC_IRQ		 /*QDEC */
    .long 	0		 /*Reserved */
    vector_definition 	SWI0_IRQ		 /*SWI0 */
    vector_definition 	SWI1_IRQ		 /*SWI1 */
    vector_definition 	SWI2_IRQ		 /*SWI2 */
    vector_definition 	SWI3_IRQ		 /*SWI3 */
    vector_definition 	SWI4_IRQ		 /*SWI4 */
    vector_definition 	SWI5_IRQ		 /*SWI5 */
    .long 	0		 /*Reserved */
    .long 	0		 /*Reserved */
    .long 	0		 /*Reserved */
    .long 	0		 /*Reserved */
    .long 	0		 /*Reserved */
    .long 	0		 /*Reserved */
 
    .size    __isr_vector, . - __isr_vector

    .text
    .thumb
    .thumb_func
    .align 2
    .globl    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
    .equ   NRF_POWER_RAMON_ADDRESS,            0x40000524
    .equ   NRF_POWER_RAMON_RAM1ON_ONMODE_Msk,  0x3  
    ldr    r0, =NRF_POWER_RAMON_ADDRESS
    ldr    r2, [r0]
    movs   r1, #NRF_POWER_RAMON_RAM1ON_ONMODE_Msk
    orrs   r2, r1
    str    r2, [r0]

/*     Loop to copy data from read only memory to RAM. The ranges
 *      of copy from/to are specified by following symbols evaluated in 
 *      linker script.
 *      __etext: End of code section, i.e., begin of data sections to copy from.
 *      __data_start__/__data_end__: RAM address range that data should be
 *      copied to. Both must be aligned to 4 bytes boundary.  */
    ldr    r1, =__etext
    ldr    r2, =__data_start__
    ldr    r3, =__data_end__

    subs    r3, r2
    ble    .flash_to_ram_loop_end

    movs    r4, 0
.flash_to_ram_loop:
    ldr    r0, [r1,r4]
    str    r0, [r2,r4]
    adds    r4, 4
    cmp    r4, r3
    blt    .flash_to_ram_loop
.flash_to_ram_loop_end:
    ldr    r0, =SystemInit
    blx    r0
    ldr    r0, =_start
    bx     r0
    .pool
    .size Reset_Handler, . - Reset_Handler
    
    .macro def_wrapper peripheral
    .align 2
    .thumb_func
    .type    \peripheral\()Wrapper, %function
    .global is_bootloader_running
\peripheral\()Wrapper:
    ldr r1, =is_bootloader_running
    ldr r0, [r1]
    cmp r0, #0
    beq .\peripheral\()_app_handler
	ldr r1, =\peripheral\()Handler
	ldr r0, [r1]
    bx r0
.\peripheral\()_app_handler :
    ldr r1, =BOOTLOADER_SIZE+\peripheral\()Vector
    ldr r0, [r1]
    bx r0
    .size    \peripheral\()Wrapper, . - \peripheral\()Wrapper
    .endm

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro    def_default_handler    handler_name
    .align 1
    .thumb_func
    .weak    \handler_name
    .type    \handler_name, %function
\handler_name :
    b    .
    .size    \handler_name, . - \handler_name
    .endm
    
    .equ BOOTLOADER_SIZE, 0x20000
    def_default_handler    NMI_Handler
    def_wrapper HardFault_
    def_default_handler    HardFault_Handler
    def_wrapper SVC_
    def_default_handler    SVC_Handler
    def_wrapper PendSV_
    def_default_handler    PendSV_Handler
    def_wrapper SysTick_
    def_default_handler    SysTick_Handler
    def_default_handler    Default_Handler
    def_wrapper POWER_CLOCK_IRQ
    def_default_handler    POWER_CLOCK_IRQHandler		 
    def_wrapper RADIO_IRQ
    def_default_handler    RADIO_IRQHandler		 
    def_wrapper    UART0_IRQ
    def_default_handler    UART0_IRQHandler		 
    def_wrapper SPI0_TWI0_IRQ
    def_default_handler    SPI0_TWI0_IRQHandler		 
    def_wrapper SPI1_TWI1_IRQ
    def_default_handler    SPI1_TWI1_IRQHandler		 
    def_wrapper GPIOTE_IRQ
    def_default_handler    GPIOTE_IRQHandler		 
    def_wrapper ADC_IRQ
    def_default_handler    ADC_IRQHandler		 
    def_wrapper TIMER0_IRQ
    def_default_handler    TIMER0_IRQHandler		 
    def_wrapper TIMER1_IRQ
    def_default_handler    TIMER1_IRQHandler		 
    def_wrapper TIMER2_IRQ
    def_default_handler    TIMER2_IRQHandler		 
    def_wrapper RTC0_IRQ
    def_default_handler    RTC0_IRQHandler		 
    def_wrapper TEMP_IRQ
    def_default_handler    TEMP_IRQHandler		 
    def_wrapper RNG_IRQ
    def_default_handler    RNG_IRQHandler		 
    def_wrapper    ECB_IRQ		 
    def_default_handler    ECB_IRQHandler		 
    def_wrapper    CCM_AAR_IRQ		 
    def_default_handler    CCM_AAR_IRQHandler		 
    def_wrapper    WDT_IRQ		 
    def_default_handler    WDT_IRQHandler		 
    def_wrapper    RTC1_IRQ		 
    def_default_handler    RTC1_IRQHandler		 
    def_wrapper    QDEC_IRQ		 
    def_default_handler    QDEC_IRQHandler		 
    def_wrapper    SWI0_IRQ		 
    def_default_handler    SWI0_IRQHandler		 
    def_wrapper    SWI1_IRQ		 
    def_default_handler    SWI1_IRQHandler		 
    def_wrapper    SWI2_IRQ		 
    def_default_handler    SWI2_IRQHandler		 
    def_wrapper    SWI3_IRQ		 
    def_default_handler    SWI3_IRQHandler		 
    def_wrapper    SWI4_IRQ		 
    def_default_handler    SWI4_IRQHandler		 
    def_wrapper    SWI5_IRQ		 
    def_default_handler    SWI5_IRQHandler		 

    .weak    DEF_IRQHandler
    .set    DEF_IRQHandler, Default_Handler

    .end
