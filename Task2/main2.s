	.syntax unified
	.cpu cortex-m4
	.fpu softvfp
	.syntax unified
	.thumb

.global main

	   .data
 message:  .asciz "HOW ARE YOU?"  // Saving message on memory address
	   .text


main:
	ldr r0,=message      //Getting the message's memory address
	mov r1,#0			//Cleaning register R1
	mov r2,#0			//additional register which will count the number of passes through the loop
loop:
	ldrb r1,[r0], #1    //Getting char value (ascii value)
						//Shifting
	cmp r1,#0x00		//Comparing char ascii value with 0 (ascii value for null char is 0)
	beq endloop			//If r1==0x00 -> break the loop 
	add r2,r2,#1		//counter++ (additional register)
	b loop				//Branch
endloop:
	mov r1,#0           // Test line 
