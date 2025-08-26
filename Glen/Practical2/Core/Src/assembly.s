/*
 * assembly.s
 *
 */
 
 @ DO NOT EDIT
	.syntax unified
    .text
    .global ASM_Main
    .thumb_func

@ DO NOT EDIT
vectors:
	.word 0x20002000
	.word ASM_Main + 1

@ DO NOT EDIT label ASM_Main
ASM_Main:

	@ Some code is given below for you to start with
	@ ENABLING HARDWARE CLOCKS
	LDR R0, RCC_BASE  		@ Enable clock for GPIOA and B by setting bit 17 and 18 in RCC_AHBENR
	LDR R1, [R0, #0x14]     @ Reads value from R0 with offset and stores in R1
	LDR R2, AHBENR_GPIOAB	@ AHBENR_GPIOAB is defined under LITERALS at the end of the code. Loads bin for Port A , B On.
	ORRS R1, R1, R2			@ R1 = R1 | R2 (combining binarys
	STR R1, [R0, #0x14]     @ Stores new R1 value in the RCC_AHBENR location

	LDR R0, GPIOA_BASE		@ Enable pull-up resistors for pushbuttons
	MOVS R1, #0b01010101    @ For PUPDR
	STR R1, [R0, #0x0C]     @ Store R1 at R0 plus offset
	LDR R1, GPIOB_BASE  	@ Set pins connected to LEDs to outputs
	LDR R2, MODER_OUTPUT
	STR R2, [R1, #0]		@ Atore with ofset of 1. Now configured as outputs
	MOVS R2, #0         	@ NOTE: R2 will be dedicated to holding the value on the LEDs

@ TODO: Add code, labels and logic for button checks and LED patterns

@ Note: We know registers R0-R10 are unreserved and we can use them
@ Available Flage: GT, LT, EQ/Zero, Carry
@ Check what the state of the buttons is. Assign R3 button state register address
@ Can only use registers r0 - r7

	LDR R3, GPIOA_BASE @ Store button address in R3
	MOVS R4, #0 @Set the R4 register value to 0

main_loop: @ Begin main loop


LDR R7, SHORT_DELAY_CNT  @ Sets defualt delay

@Difference between TST(For bitwise comparison) and CMP(For integer comparison)

@ Store Button Registers in R5, R3 is just the start of the port address
LDR R5, [R3, #0x10] @ You need to add an address offset because button registers aren't striaght at the beginning of port

@test if SW3 has been pressed
test_SW3:
	MOVS R6, #8 @ Value 8 into R6 and set status flag - bitmask for SW3 pushed
	TST R5,  @ Test R5 (Bitwise AND with value in R6) ,
	BEQ write_leds @ Branch if Equal -> jumps to write_leds section of code


@ test if SW2 has been pressed
test_SW2:
	MOVS R6, #4
	TST R5, R6
	BEQ write_leds @ move to write_leds label in code if pressed

test_SW0:
	@ Test if SW0 has been pressed, pulled-up so pressed = zero
	MOVS R6, #1
	TST R5, R6 @ If button was pressed --> Sets Z flag to 1 --> EQ condition checks the Z flag
	BEQ increment_by_2

@ Increment count to LEDs by 1
ADDS r2, #1
B test_SW1

@ Increment LEDs by 2
increment_by_2:
	ADDS r2, #2

test_SW1:
	@ R6 used to store ref test value.
	MOVS R6, #2
	TST R5, R6  @Test if SW1 has been pressed. Since buttons are puleld up, Test if Bits are equal to 0.
	BEQ change_delay
	B delay

change_delay:
	LDR R7, LONG_DELAY_CNT

delay:
	SUBS R7, #1
	BNE delay  @ Branch if result(R7) is not equal to zero

@delay_long:
@	LDR R7, LONG_DELAY_CNT
@	SUBS R7, #1
@	BNE delay_long

write_leds:

	@ Test if SW3 has been pressed
	MOVS R6, #8
	TST R5, R6
	BEQ loop @ move to the loop label if the button has been pressed

	@ test if SW2 has been pressed
	MOVS R6, #4
	TST R5, R6
	BNE continue_patterns @ continue normally if SW_2 has not been pressed

	MOVS R6, #0xAA
	STR R6, [R1, #0x14]
	B loop

	continue_patterns:
		STR R2, [R1, #0x14]  @ Stores R2 into the register with address R1 + offset

	loop:
		B main_loop

@ LITERALS; DO NOT EDIT
	.align
RCC_BASE: 			.word 0x40021000
AHBENR_GPIOAB: 		.word 0b1100000000000000000
GPIOA_BASE:  		.word 0x48000000 @ Stores the buttons
GPIOB_BASE:  		.word 0x48000400 @ Stores the LEDs
MODER_OUTPUT: 		.word 0x5555

@ TODO: Add your own values for these delays
LONG_DELAY_CNT: 	.word 1900000    @ ≈0.7 s (tune in lab)
SHORT_DELAY_CNT: 	.word 800000	 @ ≈0.3 s (tune in lab)
