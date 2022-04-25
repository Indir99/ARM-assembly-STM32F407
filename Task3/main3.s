	.syntax unified
	.cpu cortex-m4
	.fpu softvfp
	.syntax unified
	.thumb

.global main

	   .data
	   .text

main:

//First case X > 10 - in R2 reg we expect value R2=11
	mov r1,#15       // R1=15
	mov r2,#0        // R2=0
	CMP r1,#10		 // Comparing R1 value with 10 (changing flags)

	ITE GT           // IT 
	movgt r2,#11     // if(R1>10), execute
	movle r2,#9		 // if(R1<=10), execute
	IT EQ
	moveq r2,#10 	 // if(R1=10), execute
// Result: R2=11;


// Second case X<10 - in R2 reg we expect value R2=9
	mov r1,#7		// R1=7
	mov r2,#0		// R2=0
	CMP r1,#10		// Comparing R1 value with 10

	ITE GT			// IT 
	movgt r2,#11	// if(R1>10), execute
	movle r2,#9		// if(R1<=10), execute
	IT EQ
	moveq r2,#10	//if(R1=10), execute
// Result R2=9;

// Third case X=10 - in R2 reg we expect value R2=10
	mov r1,#10		// R1=10
	mov r2,#0		// R2=0
	CMP r1,#10		// Comparing R1 value with 10

	ITE GT			// IT 
	movgt r2,#11	// if(R1>10), execute
	movle r2,#9		// if(R1<=10), execute
	IT EQ
	moveq r2,#10	//if(R1=10), execute
// Result: R2=10

