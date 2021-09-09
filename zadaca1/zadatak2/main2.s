	.syntax unified
	.cpu cortex-m4
	.fpu softvfp
	.syntax unified
	.thumb

.global main

	   .data
 poruka:  .asciz "KAKO STE DANAS"  //Odg. tekst smjestimo u memoriju na neku adresu
	   .text


main:
	ldr r0,=poruka      //Dohvatamo adresu teksta i smjestimo je u registar R0
	mov r1,#0			//Cistimo registar R1
	mov r2,#0			//Pomocni registar koji broji koliko smo puta bili u petlji
loop:
	ldrb r1,[r0], #1    //Dohvatamo prvi karakter i smijestamo ga u registar R1, tj njegovu ascii vrijednost
						//Nakon toga se vrsi shiftanje
	cmp r1,#0x00		//Poredimo da li je nas karakter jednak null karakteru (ascii vr. za null karakter je 0)
	beq endloop			//Ako je nas karakter jednak null karakteru, prekini petlju
	add r2,r2,#1		//Inkrementiraj pomocni brojac
	b loop				//Branch (skok) u petlju
endloop:
	mov r1,#0           // test linija za provjeru izlaska iz petlje
//Vrijednost u registru R2 nakon izvrsavanja petlje je 14 jer odgovarajuci tekst ima tacno toliko karaktera
// do null karaktera
