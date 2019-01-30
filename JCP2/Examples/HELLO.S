; Simple hello world app for Skunk Console - run with jcp -c hello.cof

	.include "jaguar.inc"
	
	; just copied the needed externs from skunk.s
	; you could also just keep them all, no harm
	; Otherwise you get 'undefined expression'
	.extern skunkRESET
	.extern skunkNOP
	.extern skunkCONSOLEWRITE
	.extern skunkCONSOLECLOSE
		
start:
	; prepare the library
	jsr skunkRESET
	
	; Write the string to the console (load address of string to a0)
	move.l	#helloworld,a0
	jsr skunkCONSOLEWRITE

	; close the console (no arguments)
	jsr skunkCONSOLECLOSE
	
	; wait to be reset
	move.w #$12dd,BG
forevr:
	nop
	jmp forevr
	
	.long	
helloworld:
	.dc.b	'Hello world!',13,10,0

	.end
