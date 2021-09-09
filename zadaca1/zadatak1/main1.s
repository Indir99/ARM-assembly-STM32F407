

	.syntax unified
	.cpu cortex-m4
	.fpu softvfp
	.syntax unified
	.thumb

.global main

.section .text.main


main:
	mov r0,#0x2         //U registar R0 upisujemo R0=0x2
	mov r1,r0,LSL #28   //Dolazak do prve adrese na SRAM, R1=0x20000000
	mov r4,r1           //Cuvamo polaznu adresu,  R4=R1
	add r2,r1,#0xff     //Dolazak do druge adrese na SRAM, R2=0x200000ff
	mov r5,r2           //Cuvamo drugu polaznu adresu, R5=R2

// Registri R4 i R5 u sebi sadrze adrese iste kao i registri R1 i R2
// Razlog koristenja navedenih registara je to sto cemo nakon punjenja nizova izgubiti polazne adrese
// koje su smjestene u registre R1 i R2


	mov r10,#20         //Brojac za for petlju, R10=20
	mov r3,#1			//R3=1, pomocni registar za punjenje prvog niza

loop1:                  //Punimo prvi niz
	str r3,[r1], #4     //Na adresu u registru R1 smijestamo vr. registra R3 i pomjeramo se za 4 bajta
	add r3,r3,#1		//Inkrementiramo vrijednost registra R3
	subs r10,r10,#1		//Smanjujemo vrijednost brojaca i setujemo flag
	bne loop1           //branch uz uslov

// Prvi niz nakon punjenja je:
// 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20

	mov r10,#20
	mov r3,#1

loop2:                  //Punimo drugi niz na isti nacin
	str r3,[r2], #4
	add r3,r3,#1
	subs r10,r10,#1
	bne loop2;

// Drugi niz nakon punjenja je:
// 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20


// Glavni dio zadatka

	mov r10,#20         //Brojac za for petlju, R10=20
	mov r3,#0           //Suma, R3=0

loop3:					//for petlja iz zadatka
	ldr r1,[r4],#4 		//Ucitavamo u registar r1 prvu varijablu prvog niza i nakon toga vrsimo pomjeranje
	ldr r2,[r5],#4		//Ucitavamo u registar r2 prvu varijablu drugog niza i nakon toga vrsimo pomjeranje
	mla r3,r1,r2,r3		//Mnozenje i-tih varijabli i sumiranje sa prethodnom vrijednosti sume
	subs r10,r10,#1		//Smanjujemo vrijednost brojaca i setujemo flag
	bne loop3

// Nakon izvrsavanja prethodne petlje, vrijednost sume (registra R3) je R3=2870
// Isto rjesenje smo dobili i pokretanjem C koda
