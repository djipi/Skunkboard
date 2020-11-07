; Copyright 2020, James Jones
; SPDX-License-Identifier: CC0-1.0

		.include "jaguar.inc"
		.include "skunk.inc"

; From eeprom.s
		.extern eeRawReadBank
		.extern eeInit128
		.extern eeInit2048
		.extern eeDetect

; Begin startup code. Don't use startup.s, don't clobber the stack, and don't
; even set up Tom/Jerry or Video. This code is meant to run from the skunk
; boot screen only, and returns back to the skunk polling loop when done.

		.68000
		.text
start:
		movem.l a0/d0,-(sp)
		jsr	skunkRESET
		jsr	skunkNOP
		jsr	skunkNOP

		jsr	eeDetect
		cmp.b	#0, d0
		bne	.chk2048
		lea	smallfound, a0
		jsr	skunkCONSOLEWRITE
		move.l	#128, e2psize
		jsr	eeInit128
		bra	.doneinit
.chk2048:
		cmp.b	#1, d0
		bne	.notfound
		lea	bigfound, a0
		jsr	skunkCONSOLEWRITE
		move.l	#2048, e2psize
		jsr	eeInit2048
		bra	.doneinit

.notfound:
		lea	e2pnotfound,a0
		jsr	skunkCONSOLEWRITE
		bra	.done

.doneinit:
		lea	e2pscrch,a0		; Read e2p to scratch buffer
		jsr	eeRawReadBank

.if ^^defined FOR_JCP
		; jcp will have already opened skunk file
.else
		lea	filename,a0		; Open eeprom.e2p in write mode
		move.l	#0,d0
		jsr	skunkFILEOPEN
.endif
		lea	e2pscrch,a0		; Write e2p content to file
		move.l	e2psize,d0
		jsr	skunkFILEWRITE

.if !(^^defined FOR_JCP)
		lea	e2pgoodmsg,a0
		jsr	skunkCONSOLEWRITE
.endif

.done:
		jsr	skunkFILECLOSE
		jsr	skunkCONSOLECLOSE
		movem.l (sp)+,a0/d0
		rts

		.data
		.long
e2pnotfound:	dc.b	'No supported serial EEPROM found.',13,10,0
		.long
smallfound:	dc.b	'Detected 93C46 (128-byte) serial EEPROM. Receiving data...',13,10,0
		.long
bigfound:	dc.b	'Detected 93C86 (2048-byte) serial EEPROM. Receiving data...',13,10,0
.if !(^^defined FOR_JCP)
		.long
filename:	dc.b	'eeprom.e2p',0
		.long
e2pgoodmsg:	dc.b	'EEPROM content saved.',13,10,0
.endif

		.bss
		.long
e2psize:	.ds.l	1
e2pscrch:	.ds.w	2048>>1		; Working copy of eeprom content
