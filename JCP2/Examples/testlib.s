; quickie tool to run a few skunklib functions
; intended for use from JCP
; mac -fb testlib.s
; aln -w -e -v -rd -a 5000 x 4000 -o testlib.cof testlib.o

;; Like the ROM Dump, this code is really, really dumb, expecting 
;; to run from the skunkboard startup. It won't even init hardware
;; or set up the OPL, since that's all technically done.
;; All it's going to do is run through the utilities 

	.include "jaguar.inc"
	
start:
	; prepare the library
	jsr skunkRESET
;	jsr skunkRESET6MB
	
	jsr skunkNOP
	
	move.l	#file,a0
	jsr skunkCONSOLEWRITE

	clr	d3
	move.l	#out,a1
i_lp:
	move.l	#buf,a0
	; note that buf is 20 blanks and a NUL /after/ the 20, so it's always terminated
	move.l	#20,d0
	jsr skunkCONSOLEREAD
	
	; save off the first 4 bytes
	move.l	buf,(a1)+
	addq	#1,d3
	cmp.l	#10,d3
	bne		i_lp
	
	; just to prove it worked, write the data back out
	clr	d3
	move.l	#out,a1
i_lp2:
	move.l	(a1)+,buf
	; add EOL
	move.b	#13,buf+4
	move.b	#10,buf+5
	move.b	#0, buf+6
	
	move.l	#buf,a0
	jsr skunkCONSOLEWRITE
	
	addq	#1,d3
	cmp.l	#10,d3
	bne		i_lp2
	
	; now test the file system
	jsr skunkNOP
	
	move.l	#file,a0
	moveq	#0,d0
	jsr skunkFILEOPEN
	
	move.l	#hello,a0
	moveq	#5,d0
	jsr	skunkFILEWRITE
	
	move.l	#world,a0
	moveq	#5,d0
	jsr	skunkFILEWRITE
	
	jsr skunkFILECLOSE
	jsr skunkCONSOLECLOSE

	; wait to be reset
	move.w #$12dd,BG
forevr:
	nop
	jmp forevr
	
	.long	
buf:
	.dc.b	'                    ',0
	
	.long
file:
	.dc.b	'dummy',0

	.long
out:
	.dc.l	0,0,0,0,0,0,0,0,0,0
	
	.long
hello:
	.dc.b	'Hello'
	
	.long
world:
	.dc.b	'World'
	
	.long
	.include "skunk.s"

	.end
