

	.syntax unified
	.cpu cortex-m4
	.fpu softvfp
	.syntax unified
	.thumb

.global main

.section .text.main


main:
	mov r0,#0x2         //writing in register R0:  R0=0x2
	mov r1,r0,LSL #28   //First address on SRAM, R1=0x20000000
	mov r4,r1           //Saving address in R4,  R4=R1
	add r2,r1,#0xff     //Second address on SRAM, R2=0x200000ff
	mov r5,r2           //Saving address in R5, R5=R2

// Values of R4 and R5 registers are the same as values of R1 and R2 registers
// Reason behind that copying is that we will lose addresses in registers R1 and R2 while we fill arrays

	mov r10,#20         //counter -> for loop, R10=20
	mov r3,#1			//R3=1, additional register for filling arrays

loop1:                  //Filling first array
	str r3,[r1], #4     //On address in register R1 we put the value of register R3, then shifting for 4 bytes (in R1 we have new address)
	add r3,r3,#1		//increment for R3 register
	subs r10,r10,#1		//Reducing counter value, setting flag
	bne loop1           //branch (with condition)

// First array:
// 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20

	mov r10,#20
	mov r3,#1

loop2:                  // Filling second array on the same way
	str r3,[r2], #4
	add r3,r3,#1
	subs r10,r10,#1
	bne loop2;

// Second array:
// 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20


// Main task

	mov r10,#20         //Counter -> for loop, R10=20
	mov r3,#0           //Sum, R3=0

loop3:					//for loop
	ldr r1,[r4],#4 		// R1 = firstArray[counter], shifting for 4 bytes
	ldr r2,[r5],#4		// R2 = secondArray[cunter], shifting for 4 bytes
	mla r3,r1,r2,r3		// sum+=R1*R2;
	subs r10,r10,#1		// Reducing counter value, setting flag
	bne loop3

// Sum, R3=2870
