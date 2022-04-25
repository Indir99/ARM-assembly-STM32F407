	.syntax unified
	.cpu cortex-m4
	.fpu softvfp
	.syntax unified
	.thumb

.global main

	   .data
	   .text

main:

//Prvi slucaj X>10 - ocekujemo u registru R2 vrijednost R2=11
	mov r1,#15       // R1=15
	mov r2,#0        // R2=0
	CMP r1,#10		 // Poredimo vrijednost registra R1 sa 10 (mijenjamo flag-ove)

	ITE GT           // IT blok
	movgt r2,#11     // Ako je R1>10 linija koda ce biti izvrsena
	movle r2,#9		 // Ako je R1<=10 linija koda ce biti izvrsena
	IT EQ
	moveq r2,#10 	 // Ako je R1=10 linija koda ce biti izvrsena
// U registru R2 imamo vrijednost R2=11;


// Drugi slucaj X<10 - ocekujemo u registru R2 vrijednost R2=9
	mov r1,#7		// R1=7
	mov r2,#0		// R2=0
	CMP r1,#10		// Poredimo vrijednost registra R1 sa 10

	ITE GT			// IT blok
	movgt r2,#11	// Ako je R1>10 linija koda ce biti izvrsena
	movle r2,#9		// Ako je R1<=10 linija koda ce biti izvrsena
	IT EQ
	moveq r2,#10	//Ako je R1=10 linija koda ce biti izvrsena
// U registru R2 imamo vrijednost R2=9;

// Treci slucaj X=10 - ocekujemo u registru R2 vrijednost R2=10
	mov r1,#10		// R1=7
	mov r2,#0		// R2=0
	CMP r1,#10		// Poredimo vrijednost registra R1 sa 10

	ITE GT			// IT blok
	movgt r2,#11	// Ako je R1>10 linija koda ce biti izvrsena
	movle r2,#9		// Ako je R1<=10 linija koda ce biti izvrsena
	IT EQ
	moveq r2,#10	//Ako je R1=10 linija koda ce biti izvrsena
// U registru R2 imamo vrijednost R2=10

