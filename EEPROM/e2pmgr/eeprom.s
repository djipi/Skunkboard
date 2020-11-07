;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Jaguar Development System Source Code					   
; Copyright (c) 1994, 1995 Atari Corp.
; ALL RIGHTS RESERVED
;
; Copyright 2020, James Jones
; Portions copyright 2001, Carl Forhan, Used and relicensed with permission.
; SPDX-License-Identifier: CC0-1.0
;
; Project: eeprom.s - E2PROM Read & Write Functions (Non-Handshaking Version)
;  Module: eeprom.s - Low & High Level Read/Write Functions
;
; Revision History:
;	24-Sep-93 -  DS: Created
;     	29-Nov-94 - SDS: Modified to use delay rather than busy poll
;       	         for wait after write.
;       15-Dec-94 - SDS: Added Eeprom series of high level calls.
;	18-Jan-95 - SDS: Renamed calls from Eeprom... to ee...
;                        for more signifigant letters in Alcyon
;                        compilation.
;       14-Mar-95 - SDS: Fixed two routines to not save D0 so an
;                        error code is actually returned.
;       22-Sep-95 -  MF: Added Library identification header string
;       21-Sep-20 -  JJ: Added raw bank read/write routines
;       20-Oct-20 -  JJ: Added 93C86 support based on code by Carl Forhan
;                        Added eeInit128/eeInit2048 to choose EEPROM type
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

		.include "jaguar.inc"	

;;;;;;;;;;;;;;;;;;;
;;; Global Symbols

		.globl eeDetect
		.globl eeInit128
		.globl eeInit2048
		.globl eeWriteWord
		.globl eeWriteBank
		.globl eeRawWriteBank
		.globl eeReadWord
		.globl eeReadBank
		.globl eeRawReadBank
		.globl eeUpdateChecksum
		.globl eeValidateChecksum

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;	Hi-Score on-board-cartridge EEPROM primitives 
;;;	for use by Jaguar game cartridge developers.
;;;
;;;	128 bytes (accessible as 64 words) of non-volatile
;;;	memory are available on Jaguar game cartridges to
;;;	preserve Hi-scores or other game status The last
;;;     word (word #63) should be used for a checksum on
;;;	data validity.
;;;
;;;	Data is retained for up to 10 years, and a minimum
;;;     of 100,000 write cycles is assured, according to
;;;	product literature. 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

GPIO_0		.equ	$f14800		;General purpose I/O #0
GPIO_1		.equ	$f15000		;General purpose I/O #1

;   Equates derived from the above
;   to allow indirect with 16-bit displacement addressing

GPIO_0of	.equ	GPIO_0-JOYSTICK	;offset to GPIO_0 (when addr reg Ax -> JOY1) 
GPIO_1of	.equ	GPIO_1-JOYSTICK	;offset to GPIO_1 (when addr reg Ax -> JOY1) 

;   9-bit Commands tested on:
;	National Semiconductor NM93C14
;       Excel (equiv)
;	Atmel (equiv)
;	ISSI (equiv)
;
;  9-bit (93C46) commands..
;		 	 876543210

eREAD		.equ	%110000000		;read from EEPROM
eEWEN		.equ	%100110000		;Erase/write Enable
eWRITE		.equ	%101000000		;Write selected register
eEWDS		.equ	%100000000		;Erase/Write disable (default)

;  13-bit Commands tested on:
;       Microchip Technology 93LC86C-E/SN
;
;  13-bit (93C86) commands..
;		 	 2109876543210
e2048READ	equ	%1100000000000		;read from EEPROM
e2048EWEN	equ	%1001100000000		;Erase/write Enable
e2048WRITE	equ	%1010000000000		;Write selected register
e2048EWDS	equ	%1000000000000		;Erase/Write disable (default)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;  DO (data out)	- is read on bit0 of JOY1
;  DI (data in) 	- is written on bit0 of GPIO_0
;  CS (chip select)	- is pulsed low by any access to GPIO_1
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; EEPROM Library Header

		dc.b	"EEPROM Library",0,0		; 16 bytes long
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Procedure: eeDetect
;;;            Detect the size/type of the EEPROM, if any.
;;;
;;;  Inputs: None
;;; Returns: d0 = 0x0 - 93C46/128B EEPROM was found
;;;          d0 = 0x1 - 93C86/2048B EEPROM was found
;;;          d0 = 0xFFFFFFFF - No EEPROM found or unknown EEPROM size/type
;;;          All other registers are preserved.

eeDetect:
		movem.l	d1-d3, -(sp)

		move.l	#0, d1		; Read the first word assuming 93C46
		jsr	eeread128
		move.w	d0, d2		; Save the original value
		eor.w	#$ffff, d0	; invert all the bits
		move.w	d0, d3		; save the inverted value
		jsr	eewrite128	; Write it back
		jsr	eeread128	; Then read back what we wrote
		cmp.w	d0, d3		; Did the write succeed?
		bne	.try2048	; No.
		move.w	d2, d0		; Yes, restore the EEPROM content
		jsr	eewrite128
		moveq	#0, d0
		bra	.done

.try2048:
		jsr	eeread2048	; Read the first word assuming 93C86
		move.w	d0, d2		; Save the original value
		eor.w	#$ffff, d0	; invert all the bits
		move.w	d0, d3		; save the inverted value
		jsr	eewrite2048	; Write it back
		jsr	eeread2048	; Then read back what we wrote
		cmp.w	d0, d3		; Did the write succeed?
		bne	.notfound	; No.
		move.w	d2, d0		; Yes, restore the EEPROM content
		jsr	eewrite2048
		moveq	#1, d0
		bra	.done

.notfound:
		move.l	#$ffffffff, d0
.done:
		movem.l	(sp)+, d1-d3
		rts

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Procedure: eeInit128
;;;            Set up the library to use the 93C46/128 byte EEPROM routines
;;;
;;;  Inputs: None
;;; Returns: None (All registers are preserved)

eeInit128:
		move.w	#64, eewords
		move.l	#eeread128, eereadfunc
		move.l	#eewrite128, eewritefunc
		rts

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Procedure: eeInit2048
;;;            Set up the library to use the 93C86/2048 byte EEPROM routines
;;;
;;;  Inputs: None
;;; Returns: None (All registers are preserved)

eeInit2048:
		move.w	#1024, eewords
		move.l	#eeread2048, eereadfunc
		move.l	#eewrite2048, eewritefunc
		rts

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Procedure: eeWriteWord
;;;            Write a word to EEPROM and ensure it was written.
;;;
;;;  Inputs: d0.w = data to be written
;;;	     d1.w = least signifigant bits specify write address
;;;
;;; Returns: d0.w = Non-zero indicates an error occurred

eeWriteWord:
		move.l	d2,-(sp)
		move.w	d0,d2		; Save value

		bsr	eewrite		; Write value
		bsr	eeread		; Read value back

		cmp.w	d0,d2		; Are they the same?
		bne	.badwrite

		move.w	#$0,d0		; Success
		bra	.ewwout
.badwrite:
		move.w	#$1,d0
.ewwout:
		move.l	(sp)+,d2
		rts

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Procedure: eeReadWord
;;;            Read a word from the EEPROM.
;;;
;;;  Inputs: d1.w = Least signifigant bits specify write address
;;;
;;; Returns: d0.w = Word read

eeReadWord:
		bsr	eeread
		rts

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Procedure: eeWriteBank
;;;            Write <eepromsize> - 1 words to EEPROM plus one checksum word.
;;;
;;;  Inputs: a0.l = Pointer to data buffer containing words to write
;;;
;;; Returns: d0.w = Non-zero indicates an error occurred

eeWriteBank:
		movem.l a0/d1-d4,-(sp)

		move.w	eewords,d4
		sub.w	#1,d4
		clr.w	d1		; Address counter
		clr.w	d2		; Checksum Accumulator
.loopwrite:
		move.w	(a0)+,d0
		add.w	d0,d2		; Add value to checksum

		move.w	d0,d3		; Copy it.

		bsr	eewrite		; Write the word
		bsr	eeread		; Read the word back

		cmp.w	d0,d3		; Are they the same?
		beq	.nextword

		bra	.errwrite
.nextword:
		addq.w	#1,d1
		cmp.w	d4,d1		; Write all but last word
		blt	.loopwrite
		
		eor.w	#$FFFF,d2	; IMPORTANT!!!
		move.w	d2,d0
		
		bsr	eewrite
		bsr	eeread
		
		cmp.w	d0,d2
		bne	.errwrite
		
		move.w	#$0,d0
		bra	.ewbout
.errwrite:
		move.w	#$1,d0	
.ewbout:
		movem.l	(sp)+,a0/d1-d4
		rts

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Procedure: eeRawWriteBank
;;;            Write <eepromsize> words to EEPROM
;;;
;;;  Inputs: a0.l = Pointer to data buffer containing words to write
;;;
;;; Returns: d0.w = Non-zero indicates an error occurred

eeRawWriteBank:
		movem.l a0/d1-d3,-(sp)

		move.w	eewords,d3
		clr.w	d1		; Address counter
.loopwrite:
		move.w	(a0)+,d0

		move.w	d0,d2		; Copy it.

		bsr	eewrite		; Write the word
		bsr	eeread		; Read the word back

		cmp.w	d0,d2		; Are they the same?
		beq	.nextword

		bra	.errwrite
.nextword:
		addq.w	#1,d1
		cmp.w	d3,d1	; Write all words
		blt	.loopwrite
		
		move.w	#$0,d0
		bra	.ewbout
.errwrite:
		move.w	#$1,d0	
.ewbout:
		movem.l	(sp)+,a0/d1-d3
		rts

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Procedure: eeReadBank
;;;            Read <eepromsize> - 1 words (+ checksum) from the EEPROM.
;;;
;;;  Inputs: a0.l = Destination buffer of write data  
;;;
;;; Returns: d0.w = Non-zero indicates an error occurred

eeReadBank:
		movem.l	a0/d1-d3,-(sp)

		move.w	eewords,d3
		sub.w	#1,d3
		clr.w	d1		; Address counter
		clr.w	d2		; Checksum accumulator
.nextread:
		jsr	eeread		; Read data
		add.w	d0,d2		; Add to checksum
		move.w	d0,(a0)+	; Store data in buffer
		
		addq.w	#1,d1
		cmp.w	d3,d1
		blt	.nextread
		
		eor.w	#$FFFF,d2	; IMPORTANT!!!

		bsr	eeread
		cmp.w	d0,d2		; Do checksums match?
		bne	.bankerror
		
		move.w	#$0,d0
		bra	.eerbout
.bankerror:
		move.w	#$1,d0	
.eerbout:
		movem.l	(sp)+,a0/d1-d3
		rts

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Procedure: eeRawReadBank
;;;            Read <eepromsize> words from the EEPROM.
;;;
;;;  Inputs: a0.l = Destination buffer of write data  

eeRawReadBank:
		movem.l	a0/d0-d2,-(sp)

		move.w	eewords,d2
		clr.w	d1		; Address counter
.nextread:
		jsr	eeread		; Read data
		move.w	d0,(a0)+	; Store data in buffer
		
		addq.w	#1,d1
		cmp.w	d2,d1
		blt	.nextread
		
		movem.l	(sp)+,a0/d0-d2
		rts

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Procedure: eeUpdateChecksum
;;;            Read a bank of <eepromsize> - 1 words, calculate a new
;;;            checksum, and write it.
;;;
;;; Returns: d0.w = Non-zero indicates an error occurred

eeUpdateChecksum:
		movem.l	d1-d3,-(sp)

		move.w	eewords,d3
		sub.w	#1,d3
		clr.w	d1		; Address counter
		clr.w	d2		; Checksum accumulator
.nextread:
		bsr	eeread		; Read data
		add.w	d0,d2		; Add to checksum
		
		addq.w	#1,d1
		cmp.w	d3,d1
		blt	.nextread
		
		eor.w	#$FFFF,d2	; IMPORTANT!!!

		move.w	d2,d0
		jsr	eeWriteWord	; Will return error in D0

		movem.l	(sp)+,d1-d3
		rts

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Procedure: eeValidateChecksum
;;;            Read a bank of <eepromsize> - 1 words (+ checksum) and return
;;;	       an error code if checksum does not validate.
;;;
;;; Returns: d0.w = Non-zero indicates an error occurred

eeValidateChecksum:
		movem.l	d1-d3,-(sp)

		move.w	eewords,d3
		sub.w	#1,d3
		clr.w	d1		; Address counter
		clr.w	d2		; Checksum accumulator
.nextread:
		bsr	eeread		; Read data
		add.w	d0,d2		; Add to checksum
		
		addq.w	#1,d1
		cmp.w	d3,d1
		blt	.nextread
		
		eor.w	#$FFFF,d2	; IMPORTANT!!!

		bsr	eeread
		cmp.w	d0,d2		; Do checksums match?
		bne	.bankerror
		
		move.w	#$0,d0
		bra	.eerbout
.bankerror:
		move.w	#$1,d0	
.eerbout:
		movem.l	(sp)+,d1-d3
		rts

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; LOW-LEVEL PRIMITIVE (DO NOT CALL DIRECTLY...OR MODIFY FOR THAT MATTER)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Procedure: eewrite
;;;            Write a word to the EEPROM type selected by eeInit[128,2048]
;;;
;;;  Inputs: d0.w = data to be written
;;;          d1.w = least signifigant bits specify write address
;;;

eewrite:
	move.l	a0,-(sp)
	move.l	eewritefunc,a0
	jsr	(a0)
	move.l	(sp)+,a0
	rts

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; LOW-LEVEL PRIMITIVE (DO NOT CALL DIRECTLY...OR MODIFY FOR THAT MATTER)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Procedure: eewrite128
;;;            Write a word to a 93C46 (128 byte) EEPROM
;;;
;;;  Inputs: d0.w = data to be written
;;;	     d1.w = least signifigant 6 bits specify write address (0-63)
;;;

eewrite128:
		movem.l	a0/d0-d3,-(sp)
		lea	JOYSTICK,a0  	;set ptr to EEPROM i/o addresses

		tst.w	GPIO_1of(a0)	;strobe ChipSelect

		move.w	#eEWEN,d2	;erase/write enable command
		bsr	out9bits	;send it to EEPROM

		tst.w	GPIO_1of(a0)	;strobe ChipSelect

		andi.w	#$3f,d1		;force write addr to be legit (0-63)
		ori.w	#eWRITE,d1	;form WRITE command
		move.w	d1,d2
		bsr	out9bits	;send it to EEPROM

		move.w	d0,d2		;get 16-bit data word to send
		bsr	out16bit	;  & send it

		tst.w	GPIO_1of(a0)	;strobe ChipSelect

;;; Chip specs say to wait 1 msec for status valid...we wait an additional
;;; 10 to ensure write has completed because the chip status report can't
;;; be relied upon in our case.

		move.w	#5267,d0    	; Wait 11 msecs
.wrwait:
		nop			
		nop
		nop
		nop
		nop
		nop
		dbra	d0,.wrwait

		move.w	#eEWDS,d2	;get erase/write disable command
		bsr	out9bits	;send it

		tst.w	GPIO_1of(a0)	;strobe ChipSelect

		movem.l	(sp)+,a0/d0-d3
		rts			;we're done

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; LOW-LEVEL PRIMITIVE (DO NOT CALL DIRECTLY...OR MODIFY FOR THAT MATTER)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Procedure: eewrite2048
;;;            Write a word to a 93C86 (2048 byte) EEPROM
;;;
;;;  Inputs: d0.w = data to be written
;;;	     d1.w = least signifigant 10 bits specify write address (0-1023)
;;;

eewrite2048:
		movem.l	a0/d0-d3,-(sp)
		lea	JOYSTICK,a0  	;set ptr to EEPROM i/o addresses

		tst.w	GPIO_1of(a0)	;strobe ChipSelect

		move.w	#e2048EWEN,d2	;erase/write enable command
		bsr	out13bits	;send it to EEPROM

		tst.w	GPIO_1of(a0)	;strobe ChipSelect

		andi.w	#$3ff,d1	;force write addr to be legit (0-63)
		ori.w	#e2048WRITE,d1	;form WRITE command
		move.w	d1,d2
		bsr	out13bits	;send it to EEPROM

		move.w	d0,d2		;get 16-bit data word to send
		bsr	out16bit	;  & send it

		tst.w	GPIO_1of(a0)	;strobe ChipSelect

;;; Comment in eewrite128 implies chip status bit reads can't be relied upon,
;;; so just wait long enough that we can be sure the write has completed.
;;; Was $1493 from JagServer OS disassembly (likely copied from the value
;;; Atari used above in eewrite128), then $1100, now $c00 (Around 6.4ms)
		move.w	#$c00,d0    	; Wait 6.4 msecs
.wrwait:
		nop
		nop
		nop
		nop
		nop
		nop
		dbra	d0,.wrwait

		move.w	#e2048EWDS,d2	;get erase/write disable command
		bsr	out13bits	;send it

		tst.w	GPIO_1of(a0)	;strobe ChipSelect

		movem.l	(sp)+,a0/d0-d3
		rts			;we're done

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; LOW-LEVEL PRIMITIVE (DO NOT CALL DIRECTLY...OR MODIFY FOR THAT MATTER)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Procedure: eeread
;;;            Read a word from the EEPROM type selected by eeInit[128,2048]
;;;
;;;  Inputs: d0.w = data to be written
;;;          d1.w = least signifigant bits specify read address
;;;

eeread:
	move.l	a0,-(sp)
	move.l	eereadfunc,a0
	jsr	(a0)
	move.l	(sp)+,a0
	rts

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; LOW-LEVEL PRIMITIVE (DO NOT CALL DIRECTLY...OR MODIFY FOR THAT MATTER)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Procedure: eeread128
;;;            Read a word from a 93C46 (128 byte) EEPROM
;;;
;;;  Inputs: d1.w = least signifigant 6 bits specify read address (0-63)
;;;
;;; Returns: d0.w = data as read from EEPROM
;;;

eeread128:
		movem.l	a0/d1-d3,-(sp)
		lea	JOYSTICK,a0 	;set ptr to EEPROM i/o address

		tst.w	GPIO_1of(a0)	;strobe ChipSelect

		andi.w	#$3f,d1		;force legit read addr
		ori.w	#eREAD,d1
		move.w	d1,d2
		bsr	out9bits

		moveq	#0,d0
		moveq	#15,d2		;pick up 17 bits (1st is dummy)
.inlp:
		tst.w	GPIO_0of(a0)
		nop
		move.w	(a0),d1
		lsr.w	#1,d1
		addx.w	d0,d0
		nop
		nop
		nop
		nop
		nop
		nop
		dbra	d2,.inlp

		movem.l	(sp)+,a0/d1-d3
		rts

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; LOW-LEVEL PRIMITIVE (DO NOT CALL DIRECTLY...OR MODIFY FOR THAT MATTER)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Procedure: eeread2048
;;;            Read a word from a 93C86 (2048 byte) EEPROM
;;;
;;;  Inputs: d1.w = least signifigant 10 bits specify read address (0-1023)
;;;
;;; Returns: d0.w = data as read from EEPROM
;;;
eeread2048:
		movem.l	a0/d1-d3,-(sp)
		lea	JOYSTICK,a0 	;set ptr to EEPROM i/o address

		tst.w	GPIO_1of(a0)	;strobe ChipSelect

		andi.w	#$3ff,d1	;force legit read addr
		ori.w	#e2048READ,d1
		move.w	d1,d2
		bsr	out13bits

		moveq	#0,d0
		moveq	#15,d2		;pick up 17 bits (1st is dummy)
.inlp:
		tst.w	GPIO_0of(a0)
		nop
		move.w	(a0),d1
		lsr.w	#1,d1
		addx.w	d0,d0
		nop
		nop
		nop
		nop
		nop
		nop
		dbra	d2,.inlp

		movem.l	(sp)+,a0/d1-d3
		rts


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Procedures: out16bit/out13bits/out9bits
;;;             Output 'x' bits to the eeprom.
;;;	      Serial data sent to device is written to DI, bit0 of GPIO_0
;;;
;;; Inputs: a0.l = JOYSTICK
;;;  	    d2.w = 9/13/16-bit data word to write
;;;
;;; Register Usage: d2.w, d3.l are destroyed
;;;

out16bit:
		rol.w	#1,d2		;align 1st serial data bit (bit15) to bit0
		moveq	#15,d3		;send 16 bits
		bra.s	out9lp
out13bits:
		rol.w	#4, d2		;align 1st serial data bit (bit12) to bit0
		moveq	#12, d3		;send 13 bits
		bra.s	out9lp
out9bits:
		rol.w	#8,d2		;align 1st serial data bit (bit8) to bit0
		moveq	#8,d3		;send 9
out9lp:
		move.w	d2,GPIO_0of(a0)	;write next bit
		nop
		nop
		nop			;delay next write
		nop
		nop
		nop
		rol.w	#1,d2		;adjust bit0 for next datum
		dbra	d3,out9lp	;go for all 9 or all 16
		rts

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Data
.data
		.long
eewritefunc:
		dc.l	eewrite128
eereadfunc:
		dc.l	eeread128
eewords:
		dc.w	64

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

		.end

