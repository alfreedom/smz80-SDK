;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.5.0 #9253 (Jun 20 2015) (Linux)
; This file was generated Fri Sep 22 15:19:39 2017
;--------------------------------------------------------
	.module main
	.optsdcc -mz80
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _main
	.globl _init_system
	.globl _isr_vector38
	.globl _isr_vector66
	.globl _uart_print
	.globl _uart_interrupt_isr
	.globl _isprint
	.globl _flag
	.globl _cont
	.globl ___ret_aux
	.globl _address_low
	.globl _address_hight
	.globl _data
	.globl _write_byte_EEPROM_ptr
	.globl _delay_1ms_ptr
	.globl _size
	.globl _dir_destination
	.globl _dir_origin
	.globl _delay_1ms
	.globl _delay_ms
	.globl _delay_10us
	.globl _delay_100us
	.globl _copeaBloque
	.globl _uart_init
	.globl _uart_set_baudrate
	.globl _uart_write
	.globl _uart_write_buffer
	.globl _uart_read
	.globl _uart_read_buffer
	.globl _uart_available
	.globl _uart_flush
	.globl _uart_read_line
	.globl _uart_disable_interrupts
	.globl _uart_enable_interrupts
	.globl _printBuffer
	.globl _eeprom_write
	.globl _eeprom_erase
	.globl _eeprom_write_buffer
	.globl _eeprom_read
	.globl _eeprom_read_buffer
	.globl _write_byte
	.globl _packet_fill
	.globl _packet_check
	.globl _packet_read
	.globl _packet_send
	.globl _bootloader_init
	.globl _bootloader_check_program_commnad
	.globl _bootloader_run
	.globl _bootloader_start_app
	.globl _io_write
	.globl _io_read
	.globl _io_write_buffer
	.globl _io_read_buffer
	.globl _ppi_init
	.globl _ppi_set_portc_bit
	.globl _ppi_clear_portc_bit
	.globl _test_program_command
;--------------------------------------------------------
; special function registers
;--------------------------------------------------------
_URRBR	=	0x0070
_URTHR	=	0x0070
_URIER	=	0x0071
_URIIR	=	0x0072
_URFCR	=	0x0072
_URLCR	=	0x0073
_URLSR	=	0x0075
_URMCR	=	0x0074
_URMSR	=	0x0076
_URDLL	=	0x0070
_URDLM	=	0x0071
_PPI_PORTA	=	0x0000
_PPI_PORTB	=	0x0001
_PPI_PORTC	=	0x0002
_PPI_CTRL	=	0x0003
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area _DATA
_dir_origin::
	.ds 2
_dir_destination::
	.ds 2
_size::
	.ds 2
_delay_1ms_ptr::
	.ds 2
__uart_in_buffer:
	.ds 1024
__in_buffer_index:
	.ds 2
__out_buffer_index:
	.ds 2
__is_interrupt_enable:
	.ds 1
_eeprom_ptr:
	.ds 2
_write_byte_EEPROM_ptr::
	.ds 2
_data::
	.ds 1
_address_hight::
	.ds 1
_address_low::
	.ds 1
_aux_address_l:
	.ds 1
_aux_address_h:
	.ds 1
_old_app_int_isr_addr:
	.ds 2
_old_app_int_isr_addr_l:
	.ds 1
_old_app_int_isr_addr_h:
	.ds 1
_old_app_nmi_isr_addr:
	.ds 2
_old_app_nmi_isr_addr_l:
	.ds 1
_old_app_nmi_isr_addr_h:
	.ds 1
_pkg_in:
	.ds 260
_pkg_out:
	.ds 260
___ret_aux::
	.ds 1
_cont::
	.ds 2
_flag::
	.ds 2
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area _INITIALIZED
_ptr_int_isr:
	.ds 2
_ptr_int_isr_l:
	.ds 2
_ptr_int_isr_h:
	.ds 2
_ptr_nmi_isr:
	.ds 2
_ptr_nmi_isr_l:
	.ds 2
_ptr_nmi_isr_h:
	.ds 2
_app_main_addr:
	.ds 2
;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area _DABS (ABS)
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area _HOME
	.area _GSINIT
	.area _GSFINAL
	.area _GSINIT
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area _HOME
	.area _HOME
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area _CODE
;./include/z80utils.h:86: void delay_1ms(){
;	---------------------------------
; Function delay_1ms
; ---------------------------------
_delay_1ms::
;./include/z80utils.h:93: __endasm;
	EXX
	EX AF,AF'
;./include/z80utils.h:96: for(j=0;j<0x04;j++)
	ld	hl,#0x0000
00106$:
;./include/z80utils.h:97: for(i=0;i<0x1FF;i++)
	ld	de,#0x01FF
00105$:
;./include/z80utils.h:98: __asm__("nop");
	nop
	ld	c,e
	ld	b,d
	dec	bc
	ld	e, c
;./include/z80utils.h:97: for(i=0;i<0x1FF;i++)
	ld	a,b
	ld	d,a
	or	a,c
	jr	NZ,00105$
;./include/z80utils.h:96: for(j=0;j<0x04;j++)
	inc	hl
	ld	a,l
	sub	a, #0x04
	ld	a,h
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00106$
;./include/z80utils.h:112: __endasm;
	EX AF,AF'
	EXX
	ret
;./include/z80utils.h:115: void delay_ms(int ms){
;	---------------------------------
; Function delay_ms
; ---------------------------------
_delay_ms::
	push	ix
	ld	ix,#0
	add	ix,sp
;./include/z80utils.h:121: __endasm;
	EXX
	EX AF,AF'
;./include/z80utils.h:125: while(ms--)
	ld	c,4 (ix)
	ld	b,5 (ix)
00102$:
	ld	e, c
	ld	d, b
	dec	bc
	ld	a,d
	or	a,e
	jr	Z,00104$
;./include/z80utils.h:126: for(i=0;i<0x10A;i++)
	ld	hl,#0x010A
00107$:
;./include/z80utils.h:127: __asm__("nop");
	nop
	ex	de,hl
	dec	de
	ld	l, e
;./include/z80utils.h:126: for(i=0;i<0x10A;i++)
	ld	a,d
	ld	h,a
	or	a,e
	jr	NZ,00107$
	jr	00102$
00104$:
;./include/z80utils.h:141: __endasm;
	EX AF,AF'
	EXX
	pop	ix
	ret
;./include/z80utils.h:144: void delay_10us(){
;	---------------------------------
; Function delay_10us
; ---------------------------------
_delay_10us::
;./include/z80utils.h:156: __endasm;
	EXX
	EX AF,AF'
	LD B,#0x2
	    LOOP_10:
	DJNZ LOOP_10
	EX AF,AF'
	EXX
	ret
;./include/z80utils.h:168: void delay_100us(){
;	---------------------------------
; Function delay_100us
; ---------------------------------
_delay_100us::
;./include/z80utils.h:181: __endasm;
	EXX
	EX AF,AF'
	LD B,#0x3A
	    LOOP_100:
	DJNZ LOOP_100
	EX AF,AF'
	EXX
	RET
	ret
;./include/z80utils.h:192: void copeaBloque(uint16_t origen,uint16_t destino, uint8_t tam)
;	---------------------------------
; Function copeaBloque
; ---------------------------------
_copeaBloque::
;./include/z80utils.h:194: dir_origin = origen;
	ld	iy,#2
	add	iy,sp
	ld	a,0 (iy)
	ld	iy,#_dir_origin
	ld	0 (iy),a
	ld	iy,#2
	add	iy,sp
	ld	a,1 (iy)
	ld	iy,#_dir_origin
	ld	1 (iy),a
;./include/z80utils.h:195: dir_destination=destino;
	ld	iy,#4
	add	iy,sp
	ld	a,0 (iy)
	ld	iy,#_dir_destination
	ld	0 (iy),a
	ld	iy,#4
	add	iy,sp
	ld	a,1 (iy)
	ld	iy,#_dir_destination
	ld	1 (iy),a
;./include/z80utils.h:196: size = tam;
	ld	iy,#6
	add	iy,sp
	ld	a,0 (iy)
	ld	iy,#_size
	ld	0 (iy),a
	ld	iy,#_size
	ld	1 (iy),#0x00
;./include/z80utils.h:202: __endasm;
	LD HL,(_dir_origin)
	LD DE,(_dir_destination)
	LD BC,(_size)
	LDIR
	ret
;./include/z80uart.h:273: void uart_init(const uart_cfg_t *uart_config){
;	---------------------------------
; Function uart_init
; ---------------------------------
_uart_init::
	push	ix
	ld	ix,#0
	add	ix,sp
;./include/z80uart.h:276: uart_set_baudrate(uart_config->baudrate);
	ld	e,4 (ix)
	ld	d,5 (ix)
	ld	a,(de)
	push	de
	push	af
	inc	sp
	call	_uart_set_baudrate
	inc	sp
	pop	de
;./include/z80uart.h:278: URIER = uart_config->interrupt;
	ld	hl,#0x0004
	add	hl,de
	ld	a,(hl)
	out	(_URIER),a
;./include/z80uart.h:279: _is_interrupt_enable = uart_config->interrupt;
	ld	(#__is_interrupt_enable + 0),a
;./include/z80uart.h:281: URLCR = (uart_config->stop_bits) | (uart_config->parity) | (uart_config->word_length);
	ld	l, e
	ld	h, d
	inc	hl
	ld	b,(hl)
	ld	l, e
	ld	h, d
	inc	hl
	inc	hl
	ld	a,(hl)
	or	a, b
	ld	h,d
	ld	l, e
	inc	hl
	inc	hl
	inc	hl
	ld	d,(hl)
	or	a, d
	out	(_URLCR),a
;./include/z80uart.h:282: _in_buffer_index = _out_buffer_index = 0;
	ld	hl,#0x0000
	ld	(__out_buffer_index),hl
	ld	l, #0x00
	ld	(__in_buffer_index),hl
	pop	ix
	ret
;./include/z80uart.h:285: void uart_set_baudrate(const uart_baudrate_t baudrate){
;	---------------------------------
; Function uart_set_baudrate
; ---------------------------------
_uart_set_baudrate::
;./include/z80uart.h:287: URLCR |= BV(UDLAB);
	in	a,(_URLCR)
	set	7, a
	out	(_URLCR),a
;./include/z80uart.h:289: URDLL = baudrate;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	out	(_URDLL),a
;./include/z80uart.h:291: URDLM = ((uint16_t)baudrate)>>8;
	ld	a, #0x00
	out	(_URDLM),a
;./include/z80uart.h:293: URLCR &= ~BV(UDLAB);
	in	a,(_URLCR)
	and	a, #0x7F
	out	(_URLCR),a
	ret
;./include/z80uart.h:296: void uart_write(uint8_t c){
;	---------------------------------
; Function uart_write
; ---------------------------------
_uart_write::
;./include/z80uart.h:298: while( !(URLSR & BV(UTHRE)))
00101$:
	in	a,(_URLSR)
	and	a, #0x20
	jr	NZ,00103$
;./include/z80uart.h:299: NOP();    
	NOP
	jr	00101$
00103$:
;./include/z80uart.h:301: URTHR = (char)c;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	out	(_URTHR),a
	ret
;./include/z80uart.h:305: void uart_write_buffer(uint8_t* buffer, int count){
;	---------------------------------
; Function uart_write_buffer
; ---------------------------------
_uart_write_buffer::
;./include/z80uart.h:307: for (i = 0; i < count; i++)
	ld	de,#0x0000
00103$:
	ld	hl,#4
	add	hl,sp
	ld	a,e
	sub	a, (hl)
	ld	a,d
	inc	hl
	sbc	a, (hl)
	jp	PO, 00116$
	xor	a, #0x80
00116$:
	ret	P
;./include/z80uart.h:308: uart_write(buffer[i]);    
	ld	hl, #2
	add	hl, sp
	ld	a, (hl)
	inc	hl
	ld	h, (hl)
	ld	l, a
	add	hl,de
	ld	h,(hl)
	push	de
	push	hl
	inc	sp
	call	_uart_write
	inc	sp
	pop	de
;./include/z80uart.h:307: for (i = 0; i < count; i++)
	inc	de
	jr	00103$
;./include/z80uart.h:311: uint8_t uart_read(){
;	---------------------------------
; Function uart_read
; ---------------------------------
_uart_read::
;./include/z80uart.h:316: while(uart_available()<=0)
00101$:
	call	_uart_available
	xor	a, a
	cp	a, l
	sbc	a, h
	jp	PO, 00120$
	xor	a, #0x80
00120$:
	jp	P,00101$
;./include/z80uart.h:321: incoming = _uart_in_buffer[_out_buffer_index++];
	ld	de,(__out_buffer_index)
	ld	hl, #__out_buffer_index+0
	inc	(hl)
	jr	NZ,00121$
	ld	hl, #__out_buffer_index+1
	inc	(hl)
00121$:
	ld	hl,#__uart_in_buffer
	add	hl,de
	ld	e,(hl)
	ld	d,#0x00
;./include/z80uart.h:322: if(_out_buffer_index == UART_BUFFER_SIZE)
	ld	a,(#__out_buffer_index + 0)
	or	a, a
	jr	NZ,00105$
	ld	a,(#__out_buffer_index + 1)
	sub	a, #0x04
	jr	NZ,00105$
;./include/z80uart.h:323: _out_buffer_index=0;
	ld	hl,#0x0000
	ld	(__out_buffer_index),hl
00105$:
;./include/z80uart.h:324: return incoming;
	ld	l,e
	ret
;./include/z80uart.h:355: int uart_read_buffer(uint8_t* buffer, int count){
;	---------------------------------
; Function uart_read_buffer
; ---------------------------------
_uart_read_buffer::
;./include/z80uart.h:358: if(uart_available() < count)
	call	_uart_available
	ld	d,l
	ld	e,h
	ld	hl,#4
	add	hl,sp
	ld	a,d
	sub	a, (hl)
	ld	a,e
	inc	hl
	sbc	a, (hl)
	jp	PO, 00122$
	xor	a, #0x80
00122$:
	jp	P,00111$
;./include/z80uart.h:359: return -1;
	ld	hl,#0xFFFF
	ret
;./include/z80uart.h:361: for (i = 0; i < count; i++)
00111$:
	ld	de,#0x0000
00105$:
	ld	hl,#4
	add	hl,sp
	ld	a,e
	sub	a, (hl)
	ld	a,d
	inc	hl
	sbc	a, (hl)
	jp	PO, 00123$
	xor	a, #0x80
00123$:
	jp	P,00103$
;./include/z80uart.h:362: buffer[i]=uart_read();
	ld	hl, #2
	add	hl, sp
	ld	a, (hl)
	inc	hl
	ld	h, (hl)
	ld	l, a
	add	hl,de
	push	hl
	push	de
	call	_uart_read
	ld	a,l
	pop	de
	pop	hl
	ld	(hl),a
;./include/z80uart.h:361: for (i = 0; i < count; i++)
	inc	de
	jr	00105$
00103$:
;./include/z80uart.h:364: return i;
	ex	de,hl
	ret
;./include/z80uart.h:367: int uart_available(){
;	---------------------------------
; Function uart_available
; ---------------------------------
_uart_available::
;./include/z80uart.h:368: int count=_in_buffer_index - _out_buffer_index;
	ld	hl,#__out_buffer_index
	ld	a,(#__in_buffer_index + 0)
	sub	a, (hl)
	ld	d,a
	ld	a,(#__in_buffer_index + 1)
	inc	hl
	sbc	a, (hl)
	ld	e,a
;./include/z80uart.h:370: return (count < 0) ? UART_BUFFER_SIZE - _out_buffer_index-1 : count ;
	bit	7, e
	jr	Z,00103$
	ld	hl,#__out_buffer_index
	ld	a,#0xFF
	sub	a, (hl)
	ld	d,a
	ld	a,#0x03
	inc	hl
	sbc	a, (hl)
	ld	e,a
00103$:
	ld	l, d
	ld	h, e
	ret
;./include/z80uart.h:374: void uart_flush(){
;	---------------------------------
; Function uart_flush
; ---------------------------------
_uart_flush::
;./include/z80uart.h:376: _in_buffer_index = _out_buffer_index = 0;
	ld	hl,#0x0000
	ld	(__out_buffer_index),hl
	ld	l, #0x00
	ld	(__in_buffer_index),hl
	ret
;./include/z80uart.h:382: void uart_interrupt_isr(){
;	---------------------------------
; Function uart_interrupt_isr
; ---------------------------------
_uart_interrupt_isr::
;./include/z80uart.h:390: __endasm;
	push af
	push bc
	push de
	push hl
	push iy
;./include/z80uart.h:393: _uart_in_buffer[_in_buffer_index++] = URRBR;
	ld	de,(__in_buffer_index)
	ld	hl, #__in_buffer_index+0
	inc	(hl)
	jr	NZ,00109$
	ld	hl, #__in_buffer_index+1
	inc	(hl)
00109$:
	ld	hl,#__uart_in_buffer
	add	hl,de
	in	a,(_URRBR)
	ld	(hl),a
;./include/z80uart.h:394: if(_in_buffer_index == UART_BUFFER_SIZE)
	ld	iy,#__in_buffer_index
	ld	a,0 (iy)
	or	a, a
	jr	NZ,00102$
	ld	iy,#__in_buffer_index
	ld	a,1 (iy)
	sub	a, #0x04
	jr	NZ,00102$
;./include/z80uart.h:395: _in_buffer_index=0;
	ld	hl,#0x0000
	ld	(__in_buffer_index),hl
00102$:
;./include/z80uart.h:405: __endasm;
	pop iy
	pop hl
	pop de
	pop bc
	pop af
	ei
	ret
	ret
;./include/z80uart.h:409: void uart_print(const uint8_t* str){
;	---------------------------------
; Function uart_print
; ---------------------------------
_uart_print::
;./include/z80uart.h:412: while(*str)       
	pop	bc
	pop	hl
	push	hl
	push	bc
00101$:
	ld	a,(hl)
	or	a, a
	ret	Z
;./include/z80uart.h:413: uart_write(*str++); // envía el siguiente caracter. 
	inc	hl
	push	hl
	push	af
	inc	sp
	call	_uart_write
	inc	sp
	pop	hl
	jr	00101$
;./include/z80uart.h:416: int uart_read_line(uint8_t* str){
;	---------------------------------
; Function uart_read_line
; ---------------------------------
_uart_read_line::
	push	ix
	ld	ix,#0
	add	ix,sp
;./include/z80uart.h:418: int n=0;
	ld	bc,#0x0000
;./include/z80uart.h:420: while(n<MAXLINE-1 && (c=uart_read()) != '\n' && c !='\r'){
00111$:
	ld	a,c
	sub	a, #0x63
	ld	a,b
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	NC,00113$
	push	bc
	call	_uart_read
	ld	a,l
	pop	bc
	ld	d,a
	sub	a, #0x0A
	jr	Z,00113$
;./include/z80uart.h:422: if(c == 0x7F || c==0x08){
	ld	a,d
	cp	a,#0x0D
	jr	Z,00113$
	cp	a,#0x7F
	jr	Z,00105$
	sub	a, #0x08
	jr	NZ,00106$
00105$:
;./include/z80uart.h:424: if(n>0){
	xor	a, a
	cp	a, c
	sbc	a, b
	jp	PO, 00149$
	xor	a, #0x80
00149$:
	jp	P,00111$
;./include/z80uart.h:425: str[--n]='\0';
	dec	bc
	ld	l,4 (ix)
	ld	h,5 (ix)
	add	hl,bc
	ld	(hl),#0x00
;./include/z80uart.h:426: uart_write(c);
	push	bc
	push	de
	push	de
	inc	sp
	call	_uart_write
	inc	sp
	ld	a,#0x20
	push	af
	inc	sp
	call	_uart_write
	inc	sp
	inc	sp
	call	_uart_write
	inc	sp
	pop	bc
	jr	00111$
00106$:
;./include/z80uart.h:432: if(isprint(c))
	ld	l,d
	ld	h,#0x00
	push	bc
	push	de
	push	hl
	call	_isprint
	pop	af
	pop	de
	pop	bc
	ld	a,h
	or	a,l
	jr	Z,00111$
;./include/z80uart.h:434: str[n++]=c;
	push	bc
	pop	iy
	inc	bc
	push	bc
	ld	c,4 (ix)
	ld	b,5 (ix)
	add	iy, bc
	pop	bc
	ld	0 (iy), d
;./include/z80uart.h:435: uart_write(c);
	push	bc
	push	de
	inc	sp
	call	_uart_write
	inc	sp
	pop	bc
	jp	00111$
00113$:
;./include/z80uart.h:439: str[n]='\0';     
	ld	l,4 (ix)
	ld	h,5 (ix)
	add	hl,bc
	ld	(hl),#0x00
;./include/z80uart.h:440: uart_write('\n');
	push	bc
	ld	a,#0x0A
	push	af
	inc	sp
	call	_uart_write
	inc	sp
;./include/z80uart.h:441: return n;
	pop	hl
	pop	ix
	ret
;./include/z80uart.h:445: void uart_disable_interrupts(){
;	---------------------------------
; Function uart_disable_interrupts
; ---------------------------------
_uart_disable_interrupts::
;./include/z80uart.h:446: URIER = 0;
	ld	a,#0x00
	out	(_URIER),a
;./include/z80uart.h:447: _is_interrupt_enable = 0;
	ld	hl,#__is_interrupt_enable + 0
	ld	(hl), #0x00
	ret
;./include/z80uart.h:450: void uart_enable_interrupts(uart_interrupt_t int_cfg){
;	---------------------------------
; Function uart_enable_interrupts
; ---------------------------------
_uart_enable_interrupts::
;./include/z80uart.h:451: URIER = int_cfg;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	out	(_URIER),a
;./include/z80uart.h:452: _is_interrupt_enable = int_cfg;
	ld	(#__is_interrupt_enable + 0),a
	ret
;./include/z80uart.h:455: void printBuffer()
;	---------------------------------
; Function printBuffer
; ---------------------------------
_printBuffer::
;./include/z80uart.h:458: for (i=0;i<UART_BUFFER_SIZE;i++)
	ld	de,#0x0000
00102$:
;./include/z80uart.h:459: uart_write(_uart_in_buffer[i]);
	ld	hl,#__uart_in_buffer
	add	hl,de
	ld	h,(hl)
	push	de
	push	hl
	inc	sp
	call	_uart_write
	inc	sp
	pop	de
;./include/z80uart.h:458: for (i=0;i<UART_BUFFER_SIZE;i++)
	inc	de
	ld	a,d
	xor	a, #0x80
	sub	a, #0x84
	jr	C,00102$
	ret
;./include/z80eeprom.h:73: uint8_t eeprom_write(uint16_t address, uint8_t number){
;	---------------------------------
; Function eeprom_write
; ---------------------------------
_eeprom_write::
;./include/z80eeprom.h:80: dir_low = address;
	ld	iy,#2
	add	iy,sp
	ld	d,0 (iy)
;./include/z80eeprom.h:81: dir_hight = (address >> 8);
	ld	b,1 (iy)
;./include/z80eeprom.h:83: if(address > BOOT_RESET_ADDR && address < BOOT_START_ADDR){
	ld	a,#0x05
	cp	a, 0 (iy)
	ld	a,#0x00
	sbc	a, 1 (iy)
	jr	NC,00102$
	ld	a,1 (iy)
	sub	a, #0x68
	jr	NC,00102$
;./include/z80eeprom.h:84: write_byte_EEPROM_ptr(dir_hight,dir_low,number);//apuntador a funcion en ram para escritura en ram.
	ld	hl, #4+0
	add	hl, sp
	ld	a, (hl)
	push	af
	inc	sp
	push	de
	inc	sp
	push	bc
	inc	sp
	ld	hl,(_write_byte_EEPROM_ptr)
	call	___sdcc_call_hl
	pop	af
	inc	sp
;./include/z80eeprom.h:87: NOP();
	NOP
;./include/z80eeprom.h:88: return 1;
	ld	l,#0x01
	ret
00102$:
;./include/z80eeprom.h:91: return 0;
	ld	l,#0x00
	ret
;./include/z80eeprom.h:95: void eeprom_erase(uint16_t address, uint16_t count) {
;	---------------------------------
; Function eeprom_erase
; ---------------------------------
_eeprom_erase::
	push	ix
	ld	ix,#0
	add	ix,sp
;./include/z80eeprom.h:98: for(addr = address; addr < (address+count); addr ++)
	ld	c,4 (ix)
	ld	b,5 (ix)
	ld	a,6 (ix)
	add	a, c
	ld	d,a
	ld	a,7 (ix)
	adc	a, b
	ld	e,a
00103$:
	ld	a,c
	sub	a, d
	ld	a,b
	sbc	a, e
	jr	NC,00105$
;./include/z80eeprom.h:99: eeprom_write(addr, 0xFF);
	push	bc
	push	de
	ld	a,#0xFF
	push	af
	inc	sp
	push	bc
	call	_eeprom_write
	pop	af
	inc	sp
	pop	de
	pop	bc
;./include/z80eeprom.h:98: for(addr = address; addr < (address+count); addr ++)
	inc	bc
	jr	00103$
00105$:
	pop	ix
	ret
;./include/z80eeprom.h:102: uint8_t eeprom_write_buffer(uint16_t address, uint8_t* data_buffer, uint16_t data_length){
;	---------------------------------
; Function eeprom_write_buffer
; ---------------------------------
_eeprom_write_buffer::
	push	ix
	ld	ix,#0
	add	ix,sp
;./include/z80eeprom.h:106: for (i = 0; i < data_length; i++){
	ld	de,#0x0000
00105$:
	ld	c, e
	ld	b, d
	ld	a,c
	sub	a, 8 (ix)
	ld	a,b
	sbc	a, 9 (ix)
	jr	NC,00103$
;./include/z80eeprom.h:108: if(!eeprom_write(address+i, data_buffer[i]))
	ld	l,6 (ix)
	ld	h,7 (ix)
	add	hl,de
	ld	h,(hl)
	ld	a,4 (ix)
	add	a, c
	ld	c,a
	ld	a,5 (ix)
	adc	a, b
	ld	b,a
	push	de
	push	hl
	inc	sp
	push	bc
	call	_eeprom_write
	pop	af
	inc	sp
	ld	a,l
	pop	de
;./include/z80eeprom.h:109: return 0;
	or	a,a
	jr	NZ,00102$
	ld	l,a
	jr	00107$
00102$:
;./include/z80eeprom.h:110: NOP();
	NOP
;./include/z80eeprom.h:106: for (i = 0; i < data_length; i++){
	inc	de
	jr	00105$
00103$:
;./include/z80eeprom.h:112: delay_ms(1000);
	ld	hl,#0x03E8
	push	hl
	call	_delay_ms
	pop	af
;./include/z80eeprom.h:113: return 1;
	ld	l,#0x01
00107$:
	pop	ix
	ret
;./include/z80eeprom.h:117: void eeprom_read(uint16_t address, uint8_t* data){
;	---------------------------------
; Function eeprom_read
; ---------------------------------
_eeprom_read::
;./include/z80eeprom.h:119: if(address <= EEPROM_SIZE){
	ld	a,#0xFF
	ld	iy,#2
	add	iy,sp
	cp	a, 0 (iy)
	ld	a,#0x7F
	sbc	a, 1 (iy)
	ret	C
;./include/z80eeprom.h:122: *data = *(uint8_t*)address;
	ld	hl, #4
	add	hl, sp
	ld	e, (hl)
	inc	hl
	ld	d, (hl)
	pop	bc
	pop	hl
	push	hl
	push	bc
	ld	a,(hl)
	ld	(de),a
	ret
;./include/z80eeprom.h:126: void eeprom_read_buffer(uint16_t address, uint8_t* data_buffer, uint16_t data_length){
;	---------------------------------
; Function eeprom_read_buffer
; ---------------------------------
_eeprom_read_buffer::
	push	ix
	ld	ix,#0
	add	ix,sp
;./include/z80eeprom.h:128: for (i = 0; i < data_length; i++)
	ld	de,#0x0000
00103$:
	ld	c, e
	ld	b, d
	ld	a,c
	sub	a, 8 (ix)
	ld	a,b
	sbc	a, 9 (ix)
	jr	NC,00105$
;./include/z80eeprom.h:129: eeprom_read(address+i,data_buffer+i);
	ld	l,6 (ix)
	ld	h,7 (ix)
	add	hl,de
	ld	a,4 (ix)
	add	a, c
	ld	c,a
	ld	a,5 (ix)
	adc	a, b
	ld	b,a
	push	de
	push	hl
	push	bc
	call	_eeprom_read
	pop	af
	pop	af
	pop	de
;./include/z80eeprom.h:128: for (i = 0; i < data_length; i++)
	inc	de
	jr	00103$
00105$:
	pop	ix
	ret
;./include/z80eeprom.h:133: void write_byte(uint8_t dir_alta ,uint8_t dir_baja , uint8_t dato)
;	---------------------------------
; Function write_byte
; ---------------------------------
_write_byte::
;./include/z80eeprom.h:136: data = dato; // byte que se va a escribir
	ld	iy,#4
	add	iy,sp
	ld	a,0 (iy)
	ld	iy,#_data
	ld	0 (iy),a
;./include/z80eeprom.h:137: address_hight = dir_alta; // direccion en la que se va a escribir
	ld	iy,#2
	add	iy,sp
	ld	a,0 (iy)
	ld	iy,#_address_hight
	ld	0 (iy),a
;./include/z80eeprom.h:138: address_low= dir_baja;
	ld	iy,#3
	add	iy,sp
	ld	a,0 (iy)
	ld	iy,#_address_low
	ld	0 (iy),a
;./include/z80eeprom.h:147: __endasm;
	LD A,(_address_hight)
	LD H,A
	LD A,(_address_low)
	LD L,A
	LD A,(_data)
	LD (HL), A
;./include/z80eeprom.h:152: __endasm;
	call 0xB000
	ret
;./include/packet.h:96: void packet_fill(packet_t *nuevo, uint8_t packet_type, uint8_t packet_number, uint8_t* packet_data, uint8_t data_length){
;	---------------------------------
; Function packet_fill
; ---------------------------------
_packet_fill::
	push	ix
	ld	ix,#0
	add	ix,sp
	push	af
	push	af
	dec	sp
;./include/packet.h:100: nuevo->mark = PACKET_MARK;
	ld	e,4 (ix)
	ld	d,5 (ix)
	ld	a,#0x3A
	ld	(de),a
;./include/packet.h:102: nuevo->data_length = data_length;
	ld	l, e
	ld	h, d
	inc	hl
	ld	a,10 (ix)
	ld	(hl),a
;./include/packet.h:103: checksum+= data_length;
	ld	a,10 (ix)
	add	a, #0x3A
	ld	b,a
;./include/packet.h:104: nuevo->number = packet_number;
	ld	l, e
	ld	h, d
	inc	hl
	inc	hl
	ld	a,7 (ix)
	ld	(hl),a
;./include/packet.h:105: checksum+= packet_number;
	ld	a,b
	add	a, 7 (ix)
	ld	b,a
;./include/packet.h:106: nuevo->type = packet_type;
	ld	l, e
	ld	h, d
	inc	hl
	inc	hl
	inc	hl
	ld	a,6 (ix)
	ld	(hl),a
;./include/packet.h:107: checksum+= packet_type;
	ld	a,b
	add	a, 6 (ix)
	ld	-3 (ix),a
;./include/packet.h:109: for (i= 0; i < data_length; ++i)
	ld	hl,#0x0004
	add	hl,de
	ld	-2 (ix),l
	ld	-1 (ix),h
	ld	bc,#0x0000
00103$:
	ld	h,10 (ix)
	ld	l,#0x00
	ld	a,c
	sub	a, h
	ld	a,b
	sbc	a, l
	jp	PO, 00116$
	xor	a, #0x80
00116$:
	jp	P,00101$
;./include/packet.h:111: nuevo->data[i] = packet_data[i];
	ld	a,-2 (ix)
	add	a, c
	ld	-5 (ix),a
	ld	a,-1 (ix)
	adc	a, b
	ld	-4 (ix),a
	push	hl
	ld	l,8 (ix)
	ld	h,9 (ix)
	push	hl
	pop	iy
	pop	hl
	add	iy, bc
	ld	a, 0 (iy)
	pop	hl
	push	hl
	ld	(hl),a
;./include/packet.h:112: checksum+= packet_data[i];
	ld	h, 0 (iy)
	ld	a,-3 (ix)
	add	a, h
	ld	-3 (ix),a
;./include/packet.h:109: for (i= 0; i < data_length; ++i)
	inc	bc
	jr	00103$
00101$:
;./include/packet.h:115: nuevo->checksum = checksum;
	ld	hl,#0x0103
	add	hl,de
	ld	a,-3 (ix)
	ld	(hl),a
	ld	sp, ix
	pop	ix
	ret
;./include/packet.h:118: uint8_t packet_check(packet_t *p)
;	---------------------------------
; Function packet_check
; ---------------------------------
_packet_check::
	push	ix
	ld	ix,#0
	add	ix,sp
	push	af
	push	af
;./include/packet.h:123: check_sum+= p->mark;
	ld	c,4 (ix)
	ld	b,5 (ix)
	ld	a,(bc)
	ld	d,a
;./include/packet.h:124: check_sum+= p->data_length;
	ld	l, c
	ld	h, b
	inc	hl
	ld	a,(hl)
	ld	-3 (ix),a
	ld	a,d
	add	a, -3 (ix)
	ld	d,a
;./include/packet.h:125: check_sum+= p->number;
	ld	l, c
	ld	h, b
	inc	hl
	inc	hl
	ld	h,(hl)
	ld	a,d
	add	a, h
	ld	d,a
;./include/packet.h:126: check_sum+= p->type;
	push	bc
	pop	iy
	ld	h,3 (iy)
	ld	a,d
	add	a, h
	ld	-4 (ix),a
;./include/packet.h:128: for (i= 0; i < p->data_length; ++i)
	ld	hl,#0x0004
	add	hl,bc
	ld	-2 (ix),l
	ld	-1 (ix),h
	ld	de,#0x0000
00106$:
	ld	h,-3 (ix)
	ld	l,#0x00
	ld	a,e
	sub	a, h
	ld	a,d
	sbc	a, l
	jp	PO, 00123$
	xor	a, #0x80
00123$:
	jp	P,00101$
;./include/packet.h:129: check_sum+= p->data[i];
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	add	hl,de
	ld	h,(hl)
	ld	a,-4 (ix)
	add	a, h
	ld	-4 (ix),a
;./include/packet.h:128: for (i= 0; i < p->data_length; ++i)
	inc	de
	jr	00106$
00101$:
;./include/packet.h:132: if(check_sum == p->checksum)
	ld	l, c
	ld	h, b
	ld	de, #0x0103
	add	hl, de
	ld	a,-4 (ix)
	sub	a,(hl)
	jr	NZ,00103$
;./include/packet.h:133: return 1;
	ld	l,#0x01
	jr	00108$
00103$:
;./include/packet.h:135: return 0;
	ld	l,#0x00
00108$:
	ld	sp, ix
	pop	ix
	ret
;./include/packet.h:138: uint8_t packet_read(packet_t *nuevo)
;	---------------------------------
; Function packet_read
; ---------------------------------
_packet_read::
	push	ix
	ld	ix,#0
	add	ix,sp
	push	af
	push	af
;./include/packet.h:148: while((c=uart_read()) != PACKET_MARK)
	ld	bc,#0x0000
00103$:
	push	bc
	call	_uart_read
	ld	a,l
	pop	bc
	ld	e,a
	sub	a, #0x3A
	jr	Z,00105$
;./include/packet.h:150: intent_count++;
	inc	bc
;./include/packet.h:151: if(intent_count>=MAX_PACKET_READ_INTENTS)
	ld	a,c
	sub	a, #0x32
	ld	a,b
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00103$
;./include/packet.h:152: return 0;
	ld	l,#0x00
	jr	00110$
00105$:
;./include/packet.h:154: nuevo->mark = c;                    // Asigna marca a paquete
	ld	c,4 (ix)
	ld	b,5 (ix)
	ld	a,e
	ld	(bc),a
;./include/packet.h:155: nuevo->data_length =uart_read();   // Lee numero de datos que contiene el paquete.
	ld	hl,#0x0001
	add	hl,bc
	ex	(sp), hl
	push	bc
	call	_uart_read
	ld	a,l
	pop	bc
	pop	hl
	push	hl
	ld	(hl),a
;./include/packet.h:156: nuevo->number=uart_read();     // Lee el numero de paquete
	ld	l, c
	ld	h, b
	inc	hl
	inc	hl
	push	hl
	push	bc
	call	_uart_read
	ld	a,l
	pop	bc
	pop	hl
	ld	(hl),a
;./include/packet.h:157: nuevo->type=uart_read();     // Lee tipo de paquete.
	ld	l, c
	ld	h, b
	inc	hl
	inc	hl
	inc	hl
	push	hl
	push	bc
	call	_uart_read
	ld	a,l
	pop	bc
	pop	hl
	ld	(hl),a
;./include/packet.h:158: for(i= 0; i< nuevo->data_length; i++)
	ld	hl,#0x0004
	add	hl,bc
	ld	-2 (ix),l
	ld	-1 (ix),h
	ld	de,#0x0000
00108$:
	pop	hl
	push	hl
	ld	h,(hl)
	ld	l,#0x00
	ld	a,e
	sub	a, h
	ld	a,d
	sbc	a, l
	jp	PO, 00134$
	xor	a, #0x80
00134$:
	jp	P,00106$
;./include/packet.h:160: nuevo->data[i]=uart_read(); // Lee los datos del paquete.
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	add	hl,de
	push	hl
	push	bc
	push	de
	call	_uart_read
	ld	a,l
	pop	de
	pop	bc
	pop	hl
	ld	(hl),a
;./include/packet.h:158: for(i= 0; i< nuevo->data_length; i++)
	inc	de
	jr	00108$
00106$:
;./include/packet.h:162: nuevo->checksum = uart_read();      // Lee el checksum de el paquete.
	ld	hl,#0x0103
	add	hl,bc
	push	hl
	call	_uart_read
	ld	a,l
	pop	hl
	ld	(hl),a
;./include/packet.h:163: return 1;
	ld	l,#0x01
00110$:
	ld	sp, ix
	pop	ix
	ret
;./include/packet.h:166: void packet_send(packet_t *p){
;	---------------------------------
; Function packet_send
; ---------------------------------
_packet_send::
	push	ix
	ld	ix,#0
	add	ix,sp
;./include/packet.h:168: uart_write(p->mark);            // Envía la marca.
	ld	e,4 (ix)
	ld	d,5 (ix)
	ld	a,(de)
	push	de
	push	af
	inc	sp
	call	_uart_write
	inc	sp
	pop	de
;./include/packet.h:169: uart_write(p->data_length);     // Envia el tamaño de datos.
	ld	c, e
	ld	b, d
	inc	bc
	ld	a,(bc)
	push	bc
	push	de
	push	af
	inc	sp
	call	_uart_write
	inc	sp
	pop	de
	pop	bc
;./include/packet.h:170: uart_write(p->number);      // Envía el número de paquete.
	ld	l, e
	ld	h, d
	inc	hl
	inc	hl
	ld	h,(hl)
	push	bc
	push	de
	push	hl
	inc	sp
	call	_uart_write
	inc	sp
	pop	de
	pop	bc
;./include/packet.h:171: uart_write(p->type);     // Envía el tipo de paquete.
	push	de
	pop	iy
	ld	h,3 (iy)
	push	bc
	push	de
	push	hl
	inc	sp
	call	_uart_write
	inc	sp
	pop	de
	pop	bc
;./include/packet.h:172: uart_write_buffer(p->data, p->data_length); // Envia los datos del paquete.
	ld	a,(bc)
	ld	c,a
	ld	b,#0x00
	ld	hl,#0x0004
	add	hl,de
	push	de
	push	bc
	push	hl
	call	_uart_write_buffer
	pop	af
	pop	af
;./include/packet.h:173: uart_write(p->checksum);        // Envia el checksum del paquete.
	pop	hl
	ld	de, #0x0103
	add	hl, de
	ld	h,(hl)
	push	hl
	inc	sp
	call	_uart_write
	inc	sp
	pop	ix
	ret
;./include/z80bootloader.h:75: void bootloader_init(){
;	---------------------------------
; Function bootloader_init
; ---------------------------------
_bootloader_init::
	push	af
	push	af
	dec	sp
;./include/z80bootloader.h:83: uart_config.baudrate    = UART_BAUDRATE_9600; 
	ld	hl,#0x0000
	add	hl,sp
	ld	(hl),#0x1A
;./include/z80bootloader.h:84: uart_config.stop_bits   = UART_STOP_BITS_1;
	ld	hl,#0x0000
	add	hl,sp
	ld	e,l
	ld	d,h
	inc	hl
	ld	(hl),#0x00
;./include/z80bootloader.h:85: uart_config.parity      = UART_PARITY_NONE;
	ld	l, e
	ld	h, d
	inc	hl
	inc	hl
	ld	(hl),#0x00
;./include/z80bootloader.h:86: uart_config.word_length = UART_WORD_LENGTH_8;
	ld	l, e
	ld	h, d
	inc	hl
	inc	hl
	inc	hl
	ld	(hl),#0x03
;./include/z80bootloader.h:87: uart_config.interrupt   = UART_INTERRUPT_RX;
	ld	hl,#0x0004
	add	hl,de
	ld	(hl),#0x01
;./include/z80bootloader.h:89: uart_init(&uart_config);
	push	de
	call	_uart_init
	pop	af
;./include/z80bootloader.h:93: old_app_int_isr_addr = *ptr_int_isr;    
	ld	hl,(_ptr_int_isr)
	ld	a,(hl)
	ld	iy,#_old_app_int_isr_addr
	ld	0 (iy),a
	inc	hl
	ld	a,(hl)
	ld	(#_old_app_int_isr_addr + 1),a
;./include/z80bootloader.h:94: old_app_int_isr_addr_l = *ptr_int_isr_l; 
	ld	hl,(_ptr_int_isr_l)
	ld	a,(hl)
	ld	(#_old_app_int_isr_addr_l + 0),a
;./include/z80bootloader.h:95: old_app_int_isr_addr_h = *ptr_int_isr_h;  
	ld	hl,(_ptr_int_isr_h)
	ld	a,(hl)
	ld	(#_old_app_int_isr_addr_h + 0),a
;./include/z80bootloader.h:96: old_app_nmi_isr_addr = *ptr_nmi_isr;    
	ld	hl,(_ptr_nmi_isr)
	ld	a,(hl)
	ld	iy,#_old_app_nmi_isr_addr
	ld	0 (iy),a
	inc	hl
	ld	a,(hl)
	ld	(#_old_app_nmi_isr_addr + 1),a
;./include/z80bootloader.h:97: old_app_nmi_isr_addr_l = *ptr_nmi_isr_l; 
	ld	hl,(_ptr_nmi_isr_l)
	ld	a,(hl)
	ld	(#_old_app_nmi_isr_addr_l + 0),a
;./include/z80bootloader.h:98: old_app_nmi_isr_addr_h = *ptr_nmi_isr_h;    
	ld	hl,(_ptr_nmi_isr_h)
	ld	a,(hl)
	ld	(#_old_app_nmi_isr_addr_h + 0),a
;./include/z80bootloader.h:101: eeprom_write((uint16_t)(ptr_int_isr_l),(uint8_t)&uart_interrupt_isr);
	ld	b,#<(_uart_interrupt_isr)
	ld	de,(_ptr_int_isr_l)
	push	bc
	inc	sp
	push	de
	call	_eeprom_write
	pop	af
	inc	sp
;./include/z80bootloader.h:102: eeprom_write((uint16_t)ptr_int_isr_h,(uint8_t)((uint16_t)(&uart_interrupt_isr)>> 8));
	ld	hl,#_uart_interrupt_isr
	ld	a, h
	ld	hl, (_ptr_int_isr_h)
	push	af
	inc	sp
	push	hl
	call	_eeprom_write
	pop	af
	inc	sp
;./include/z80bootloader.h:103: IM(1);  // Modo de interrupción 1
	IM 1 
;./include/z80bootloader.h:104: EI();   // Habilita interrupciones.
	EI
	pop	af
	pop	af
	inc	sp
	ret
;./include/z80bootloader.h:107: uint8_t bootloader_check_program_commnad(){
;	---------------------------------
; Function bootloader_check_program_commnad
; ---------------------------------
_bootloader_check_program_commnad::
;./include/z80bootloader.h:114: while(1){
	ld	de,#0x0000
00108$:
;./include/z80bootloader.h:117: if(uart_available()){
	push	de
	call	_uart_available
	pop	de
	ld	a,h
	or	a,l
	jr	Z,00104$
;./include/z80bootloader.h:119: if(uart_read() == BOOTLOADER_PROGRAM_COMMAND)
	push	de
	call	_uart_read
	ld	a,l
	pop	de
	sub	a, #0x40
	jr	NZ,00104$
;./include/z80bootloader.h:122: return 1;
	ld	l,#0x01
	ret
00104$:
;./include/z80bootloader.h:126: delay_ms(1);
	push	de
	ld	hl,#0x0001
	push	hl
	call	_delay_ms
	pop	af
	pop	de
;./include/z80bootloader.h:128: time_spend+=1;
	inc	de
;./include/z80bootloader.h:132: if(time_spend >= BOOTLOADER_PROGRAM_COMMAND_TIMEOUT)
	ld	a,e
	sub	a, #0x20
	ld	a,d
	rla
	ccf
	rra
	sbc	a, #0x83
	jr	C,00108$
;./include/z80bootloader.h:133: return 0;
	ld	l,#0x00
	ret
;./include/z80bootloader.h:138: int bootloader_run(){
;	---------------------------------
; Function bootloader_run
; ---------------------------------
_bootloader_run::
	push	ix
	ld	ix,#0
	add	ix,sp
	ld	hl,#-16390
	add	hl,sp
	ld	sp,hl
;./include/z80bootloader.h:140: uint8_t is_exit=0;
	ld	-3 (ix),#0x00
;./include/z80bootloader.h:141: uint8_t intent_count=0;
	ld	iy,#0
	add	iy,sp
	ld	0 (iy),#0x00
;./include/z80bootloader.h:145: int mem_buffer_index=0;
	ld	hl, #1
	add	hl, sp
	xor	a, a
	ld	(hl), a
	inc	hl
	ld	(hl), a
;./include/z80bootloader.h:146: delay_ms(300);
	ld	hl,#0x012C
	push	hl
	call	_delay_ms
	pop	af
;./include/z80bootloader.h:148: while(is_exit==0) 
	ld	hl,#0x0003
	add	hl,sp
	ld	-2 (ix),l
	ld	-1 (ix),h
00131$:
	ld	a,-3 (ix)
	or	a, a
	jp	NZ,00133$
;./include/z80bootloader.h:152: if(packet_read(&pkg_in))
	ld	hl,#_pkg_in
	push	hl
	call	_packet_read
	pop	af
	ld	a,l
	or	a, a
	jp	Z,00129$
;./include/z80bootloader.h:154: intent_count=0;
	ld	iy,#0
	add	iy,sp
	ld	0 (iy),#0x00
;./include/z80bootloader.h:156: if(packet_check(&pkg_in) == 0){
	ld	hl,#_pkg_in+0
	push	hl
	call	_packet_check
	pop	af
	ld	a,l
	or	a, a
	jr	NZ,00124$
;./include/z80bootloader.h:158: packet_fill(&pkg_out, PACKET_TYPE_NAK,pkg_in.number, NULL, 0);
	ld	hl, #(_pkg_in + 0x0002) + 0
	ld	c,(hl)
	ld	de,#_pkg_out
	xor	a, a
	push	af
	inc	sp
	ld	hl,#0x0000
	push	hl
	ld	b, c
	ld	c,#0x4E
	push	bc
	push	de
	call	_packet_fill
	ld	hl,#7
	add	hl,sp
	ld	sp,hl
;./include/z80bootloader.h:160: packet_send(&pkg_out);
	ld	hl,#_pkg_out
	push	hl
	call	_packet_send
	pop	af
;./include/z80bootloader.h:161: uart_flush();
	call	_uart_flush
	jr	00131$
00124$:
;./include/z80bootloader.h:166: packet_fill(&pkg_out, PACKET_TYPE_ACK, pkg_in.number, NULL, 0);
	ld	hl,#_pkg_in+2
	ld	d,(hl)
	ld	bc,#_pkg_out+0
	xor	a, a
	push	af
	inc	sp
	ld	hl,#0x0000
	push	hl
	ld	e, #0x41
	push	de
	push	bc
	call	_packet_fill
	ld	hl,#7
	add	hl,sp
	ld	sp,hl
;./include/z80bootloader.h:167: uart_flush();
	call	_uart_flush
;./include/z80bootloader.h:170: switch(pkg_in.type){
	ld	a,(#_pkg_in+3)
	cp	a,#0x44
	jr	Z,00109$
	cp	a,#0x46
	jp	Z,00119$
	cp	a,#0x53
	jr	Z,00101$
	sub	a, #0x5A
	jp	Z,00118$
	jp	00122$
;./include/z80bootloader.h:171: case PACKET_TYPE_ADDRES: // Si es paquete de direccion.
00101$:
;./include/z80bootloader.h:176: if(INT_ISR_ADDR == pkg_in.data[1] && INT_ISR_ADDR>>8 == pkg_in.data[0])
	ld	a,(#_pkg_in+5)
	sub	a, #0x38
	jr	NZ,00106$
	ld	a, (#(_pkg_in + 0x0004) + 0)
	or	a, a
	jr	NZ,00106$
;./include/z80bootloader.h:179: aux_address_l= pkg_in.data[1];
	ld	a,(#_pkg_in+5)
	ld	(#_aux_address_l + 0),a
;./include/z80bootloader.h:180: aux_address_h= pkg_in.data[0];
	ld	a,(#_pkg_in+4)
	ld	(#_aux_address_h + 0),a
	jp	00122$
00106$:
;./include/z80bootloader.h:183: if(NMI_ISR_ADDR == pkg_in.data[1] && NMI_ISR_ADDR>>8 == pkg_in.data[0]){
	ld	a,(#_pkg_in+5)
	sub	a, #0x66
	jp	NZ,00122$
	ld	a, (#(_pkg_in + 0x0004) + 0)
	or	a, a
	jp	NZ,00122$
;./include/z80bootloader.h:185: aux_address_l= pkg_in.data[1];
	ld	a,(#_pkg_in+5)
	ld	(#_aux_address_l + 0),a
;./include/z80bootloader.h:186: aux_address_h= pkg_in.data[0];
	ld	a,(#_pkg_in+4)
	ld	(#_aux_address_h + 0),a
;./include/z80bootloader.h:189: break;
	jp	00122$
;./include/z80bootloader.h:190: case PACKET_TYPE_DATA:  // Si es paquete de datos.
00109$:
;./include/z80bootloader.h:192: if(INT_ISR_ADDR == aux_address_l && INT_ISR_ADDR>>8 == aux_address_h){
	ld	a,(#_aux_address_l + 0)
	sub	a, #0x38
	jr	NZ,00115$
	ld	a,(#_aux_address_h + 0)
	or	a, a
	jr	NZ,00115$
;./include/z80bootloader.h:194: old_app_int_isr_addr_l= pkg_in.data[0];
	ld	a,(#_pkg_in+4)
	ld	(#_old_app_int_isr_addr_l + 0),a
;./include/z80bootloader.h:195: old_app_int_isr_addr_h= pkg_in.data[1];
	ld	a,(#_pkg_in+5)
	ld	(#_old_app_int_isr_addr_h + 0),a
;./include/z80bootloader.h:196: aux_address_l=0x00;
	ld	hl,#_aux_address_l + 0
	ld	(hl), #0x00
;./include/z80bootloader.h:197: aux_address_h=0x00;
	ld	hl,#_aux_address_h + 0
	ld	(hl), #0x00
	jp	00122$
00115$:
;./include/z80bootloader.h:200: if(NMI_ISR_ADDR == aux_address_l && NMI_ISR_ADDR>>8 == aux_address_h){
	ld	a,(#_aux_address_l + 0)
	sub	a, #0x66
	jr	NZ,00111$
	ld	a,(#_aux_address_h + 0)
	or	a, a
	jr	NZ,00111$
;./include/z80bootloader.h:202: old_app_nmi_isr_addr_l= pkg_in.data[0];
	ld	a,(#_pkg_in+4)
	ld	(#_old_app_nmi_isr_addr_l + 0),a
;./include/z80bootloader.h:203: old_app_nmi_isr_addr_h= pkg_in.data[1];
	ld	a,(#_pkg_in+5)
	ld	(#_old_app_nmi_isr_addr_h + 0),a
;./include/z80bootloader.h:204: aux_address_l=0x00;
	ld	hl,#_aux_address_l + 0
	ld	(hl), #0x00
;./include/z80bootloader.h:205: aux_address_h=0x00;
	ld	hl,#_aux_address_h + 0
	ld	(hl), #0x00
	jr	00122$
00111$:
;./include/z80bootloader.h:209: memcpy(&mem_buffer[mem_buffer_index], pkg_in.data, pkg_in.data_length);
	ld	a,-2 (ix)
	ld	hl,#1
	add	hl,sp
	add	a, (hl)
	ld	e,a
	ld	a,-1 (ix)
	inc	hl
	adc	a, (hl)
	ld	d,a
	ld	bc,#_pkg_in+4
	ld	hl,#_pkg_in+1
	ld	l,(hl)
	ld	h,#0x00
	push	hl
	push	bc
	push	de
	call	_memcpy
	ld	hl,#6
	add	hl,sp
	ld	sp,hl
;./include/z80bootloader.h:210: mem_buffer_index += pkg_in.data_length;
	ld	hl,#_pkg_in+1
	ld	e,(hl)
	ld	d,#0x00
	ld	hl,#1
	add	hl,sp
	ld	a,(hl)
	add	a, e
	ld	(hl),a
	inc	hl
	ld	a,(hl)
	adc	a, d
	ld	(hl),a
;./include/z80bootloader.h:224: break;
	jr	00122$
;./include/z80bootloader.h:226: case PACKET_TYPE_EOF:  // Si es paquete de fin de archivo
00118$:
;./include/z80bootloader.h:227: is_exit=1;         // Termina el programa bootloader correctamente.
	ld	-3 (ix),#0x01
;./include/z80bootloader.h:228: break;
	jr	00122$
;./include/z80bootloader.h:230: case PACKET_TYPE_FILE_HEADER:
00119$:
;./include/z80bootloader.h:233: app_program_size = *(uint16_t*)pkg_in.data;
	ld	hl,#_pkg_in+4
	ld	d,(hl)
	inc	hl
	ld	h,(hl)
;./include/z80bootloader.h:235: if(app_program_size >= APP_SIZE){
	ld	a,d
	sub	a, #0xFA
	ld	a,h
	sbc	a, #0x17
	jr	C,00122$
;./include/z80bootloader.h:237: packet_fill(&pkg_out, PACKET_TYPE_ERROR,pkg_in.number, NULL, 0);
	ld	hl,#_pkg_in+2
	ld	d,(hl)
	ld	bc,#_pkg_out+0
	xor	a, a
	push	af
	inc	sp
	ld	hl,#0x0000
	push	hl
	ld	e, #0x45
	push	de
	push	bc
	call	_packet_fill
	ld	hl,#7
	add	hl,sp
	ld	sp,hl
;./include/z80bootloader.h:238: return 0;
	ld	hl,#0x0000
	jr	00134$
;./include/z80bootloader.h:242: }
00122$:
;./include/z80bootloader.h:244: packet_send(&pkg_out);
	ld	hl,#_pkg_out+0
	push	hl
	call	_packet_send
	pop	af
	jp	00131$
00129$:
;./include/z80bootloader.h:249: intent_count++;
	ld	iy,#0
	add	iy,sp
	inc	0 (iy)
;./include/z80bootloader.h:251: if(intent_count >= MAX_READS_INTENTS)
	ld	a,0 (iy)
	sub	a, #0x0A
	jp	C,00131$
;./include/z80bootloader.h:253: return 0;
	ld	hl,#0x0000
	jr	00134$
00133$:
;./include/z80bootloader.h:260: eeprom_write_buffer(0x0080, mem_buffer, mem_buffer_index+1);
	ld	hl, #1
	add	hl, sp
	ld	e, (hl)
	inc	hl
	ld	d, (hl)
	inc	de
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	push	de
	push	hl
	ld	hl,#0x0080
	push	hl
	call	_eeprom_write_buffer
	ld	hl,#6
	add	hl,sp
	ld	sp,hl
;./include/z80bootloader.h:261: packet_fill(&pkg_out, PACKET_TYPE_EOF,pkg_in.number, NULL, 0);
	ld	hl, #(_pkg_in + 0x0002) + 0
	ld	b,(hl)
	ld	de,#_pkg_out
	xor	a, a
	push	af
	inc	sp
	ld	hl,#0x0000
	push	hl
	push	bc
	inc	sp
	ld	a,#0x5A
	push	af
	inc	sp
	push	de
	call	_packet_fill
	ld	hl,#7
	add	hl,sp
	ld	sp,hl
;./include/z80bootloader.h:262: delay_ms(500);
	ld	hl,#0x01F4
	push	hl
	call	_delay_ms
;./include/z80bootloader.h:263: packet_send(&pkg_out);
	ld	hl, #_pkg_out
	ex	(sp),hl
	call	_packet_send
;./include/z80bootloader.h:264: packet_send(&pkg_out);
	ld	hl, #_pkg_out
	ex	(sp),hl
	call	_packet_send
;./include/z80bootloader.h:265: packet_send(&pkg_out);
	ld	hl, #_pkg_out
	ex	(sp),hl
	call	_packet_send
	pop	af
;./include/z80bootloader.h:266: return 1;
	ld	hl,#0x0001
00134$:
	ld	sp, ix
	pop	ix
	ret
;./include/z80bootloader.h:271: void bootloader_start_app(){
;	---------------------------------
; Function bootloader_start_app
; ---------------------------------
_bootloader_start_app::
;./include/z80bootloader.h:275: eeprom_write((uint16_t)ptr_int_isr_l,old_app_int_isr_addr_l);
	ld	de,(_ptr_int_isr_l)
	ld	a,(_old_app_int_isr_addr_l)
	push	af
	inc	sp
	push	de
	call	_eeprom_write
	pop	af
	inc	sp
;./include/z80bootloader.h:276: eeprom_write((uint16_t)ptr_int_isr_h,old_app_int_isr_addr_h);
	ld	de,(_ptr_int_isr_h)
	ld	a,(_old_app_int_isr_addr_h)
	push	af
	inc	sp
	push	de
	call	_eeprom_write
	pop	af
	inc	sp
;./include/z80bootloader.h:277: eeprom_write((uint16_t)ptr_nmi_isr_l,old_app_nmi_isr_addr_l);
	ld	de,(_ptr_nmi_isr_l)
	ld	a,(_old_app_nmi_isr_addr_l)
	push	af
	inc	sp
	push	de
	call	_eeprom_write
	pop	af
	inc	sp
;./include/z80bootloader.h:278: eeprom_write((uint16_t)ptr_nmi_isr_h,old_app_nmi_isr_addr_h);
	ld	de,(_ptr_nmi_isr_h)
	ld	a,(_old_app_nmi_isr_addr_h)
	push	af
	inc	sp
	push	de
	call	_eeprom_write
	pop	af
	inc	sp
;./include/z80bootloader.h:281: if(*((uint8_t*)(0x0080)) == 0x00 || *((uint8_t*)(0x0080)) == 0xFF)
	ld	hl,#0x0080
	ld	a,(hl)
	or	a, a
	jr	Z,00101$
	inc	a
	jr	NZ,00102$
00101$:
;./include/z80bootloader.h:283: eeprom_write(0x0080,0x76);
	ld	a,#0x76
	push	af
	inc	sp
	ld	hl,#0x0080
	push	hl
	call	_eeprom_write
	pop	af
	inc	sp
00102$:
;./include/z80bootloader.h:288: __endasm;      
	call #0x0080
	ret
;./include/smz80.h:328: void io_write(char port_addr, char data){
;	---------------------------------
; Function io_write
; ---------------------------------
_io_write::
;./include/smz80.h:339: __endasm;
	ld ix, #2
	add ix,sp
	ld c, (ix)
	inc ix
	ld a,(ix)
	out (c), a
	ret
;./include/smz80.h:353: char io_read(char port_addr){
;	---------------------------------
; Function io_read
; ---------------------------------
_io_read::
;./include/smz80.h:365: __endasm;
	LD IX, #2
	ADD IX,SP
	LD C, (IX)
	IN A,(C)
	LD (___ret_aux),A
;./include/smz80.h:367: return __ret_aux;
	ld	iy,#___ret_aux
	ld	l,0 (iy)
	ret
;./include/smz80.h:379: void io_write_buffer(char port_addr, char* buffer_out, char count){
;	---------------------------------
; Function io_write_buffer
; ---------------------------------
_io_write_buffer::
;./include/smz80.h:395: __endasm;
	LD IX, #2
	ADD IX,SP
	LD C, (IX)
	INC IX
	LD L,(IX)
	INC IX
	LD H,(IX)
	INC IX
	LD B,(IX)
	OTIR
	ret
;./include/smz80.h:406: void io_read_buffer(char port_addr, char* buffer_in, char count){
;	---------------------------------
; Function io_read_buffer
; ---------------------------------
_io_read_buffer::
;./include/smz80.h:423: __endasm;
	LD IX, #2
	ADD IX,SP
	LD C, (IX)
	INC IX
	LD L,(IX)
	INC IX
	LD H,(IX)
	INC IX
	LD B,(IX)
	INIR
	ret
;./include/smz80.h:436: void ppi_init(const ppi_cfg_t *ppi_config){
;	---------------------------------
; Function ppi_init
; ---------------------------------
_ppi_init::
	push	ix
	ld	ix,#0
	add	ix,sp
;./include/smz80.h:438: PPI_CTRL = 0x80 | ppi_config->mode | (ppi_config->pcl_dir << PCPCL) | (ppi_config->pch_dir << PCPCH) | (ppi_config->pa_dir << PCPA) | (ppi_config->pb_dir << PCPB);
	ld	c,4 (ix)
	ld	b,5 (ix)
	ld	a,(bc)
	set	7, a
	ld	e,a
	push	bc
	pop	iy
	ld	a,3 (iy)
	or	a, e
	ld	e,a
	push	bc
	pop	iy
	ld	a,4 (iy)
	rlca
	rlca
	rlca
	and	a,#0xF8
	or	a, e
	ld	e,a
	ld	l, c
	ld	h, b
	inc	hl
	ld	a,(hl)
	rlca
	rlca
	rlca
	rlca
	and	a,#0xF0
	or	a, e
	ld	d,a
	ld	l, c
	ld	h, b
	inc	hl
	inc	hl
	ld	a,(hl)
	add	a, a
	or	a, d
	out	(_PPI_CTRL),a
	pop	ix
	ret
;./include/smz80.h:447: void ppi_set_portc_bit(const char bit){
;	---------------------------------
; Function ppi_set_portc_bit
; ---------------------------------
_ppi_set_portc_bit::
;./include/smz80.h:449: PPI_CTRL = 1 | bit << 1;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	add	a, a
	set	0, a
	out	(_PPI_CTRL),a
	ret
;./include/smz80.h:458: void ppi_clear_portc_bit(const char bit){
;	---------------------------------
; Function ppi_clear_portc_bit
; ---------------------------------
_ppi_clear_portc_bit::
;./include/smz80.h:460: PPI_CTRL = bit << 1;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	add	a, a
	out	(_PPI_CTRL),a
	ret
;main.c:44: ISR_NMI(){
;	---------------------------------
; Function isr_vector66
; ---------------------------------
_isr_vector66::
	push	af
	push	bc
	push	de
	push	hl
	push	iy
;main.c:48: }
	pop	iy
	pop	hl
	pop	de
	pop	bc
	pop	af
	retn
;main.c:50: ISR_INT_38(){
;	---------------------------------
; Function isr_vector38
; ---------------------------------
_isr_vector38::
	push	af
	push	bc
	push	de
	push	hl
	push	iy
;main.c:52: }
	pop	iy
	pop	hl
	pop	de
	pop	bc
	pop	af
	reti
;main.c:54: void init_system(){
;	---------------------------------
; Function init_system
; ---------------------------------
_init_system::
;main.c:55: PPI_CTRL=0x80;
	ld	a,#0x80
	out	(_PPI_CTRL),a
;main.c:57: write_byte_EEPROM_ptr = (void*)write_byte_EEPROM_RAM;  // apuntador de fincion guardada en ram para escribir un byte en eeprom 
	ld	hl,#0xA000
	ld	(_write_byte_EEPROM_ptr),hl
;main.c:58: delay_1ms_ptr = (void*)delay_1ms_RAM; // apuntador de funcion guardada en ram para esperar un mili-segundo.
	ld	h, #0xB0
	ld	(_delay_1ms_ptr),hl
;main.c:59: copeaBloque((uint16_t)&write_byte,write_byte_EEPROM_RAM,0x50); // copea funcion write_byte de eprom a ram.
	ld	de,#_write_byte
	ld	a,#0x50
	push	af
	inc	sp
	ld	h, #0xA0
	push	hl
	push	de
	call	_copeaBloque
	pop	af
	pop	af
	inc	sp
;main.c:60: copeaBloque((uint16_t)&delay_1ms,delay_1ms_RAM,0x30);// copea funcion de delay_1ms de eeprom a ram.
	ld	de,#_delay_1ms
	ld	a,#0x30
	push	af
	inc	sp
	ld	hl,#0xB000
	push	hl
	push	de
	call	_copeaBloque
	pop	af
	pop	af
	inc	sp
;main.c:61: bootloader_init();
	jp	_bootloader_init
;main.c:64: int main(){
;	---------------------------------
; Function main
; ---------------------------------
_main::
	ld	hl,#-260
	add	hl,sp
	ld	sp,hl
;main.c:79: init_system();
	call	_init_system
;main.c:95: uart_write('1');
	ld	a,#0x31
	push	af
	inc	sp
	call	_uart_write
	inc	sp
;main.c:100: if(bootloader_check_program_commnad())
	call	_bootloader_check_program_commnad
	ld	a, l
	or	a, a
	jr	Z,00104$
;main.c:102: uart_print("OK");
	ld	hl,#___str_0
	push	hl
	call	_uart_print
	pop	af
;main.c:112: if(!bootloader_run())
	call	_bootloader_run
	ld	a,h
	or	a,l
	jr	NZ,00102$
;main.c:115: eeprom_write(0x0080,0x76);//escribe halt en direccion 80
	ld	a,#0x76
	push	af
	inc	sp
	ld	hl,#0x0080
	push	hl
	call	_eeprom_write
;main.c:116: delay_ms(100);
	inc	sp
	ld	hl,#0x0064
	ex	(sp),hl
	call	_delay_ms
	pop	af
;main.c:120: __endasm;
	call #0x0080
;main.c:121: nop();
	NOP
00102$:
;main.c:123: packet_fill(&pkg_out, PACKET_TYPE_EOF,200, NULL, 0);
	ld	hl,#0x0000
	add	hl,sp
	ex	de,hl
	ld	c, e
	ld	b, d
	push	de
	xor	a, a
	push	af
	inc	sp
	ld	hl,#0x0000
	push	hl
	ld	hl,#0xC85A
	push	hl
	push	bc
	call	_packet_fill
	ld	hl,#7
	add	hl,sp
	ld	sp,hl
	call	_packet_send
	pop	af
00104$:
;main.c:128: bootloader_start_app();
	call	_bootloader_start_app
;main.c:131: return 0;
	ld	hl,#0x0000
	ld	iy,#260
	add	iy,sp
	ld	sp,iy
	ret
___str_0:
	.ascii "OK"
	.db 0x00
;main.c:134: void test_program_command() {
;	---------------------------------
; Function test_program_command
; ---------------------------------
_test_program_command::
;main.c:136: uart_print("Esperando comando de programacion: @");
	ld	hl,#___str_1
	push	hl
	call	_uart_print
	pop	af
;main.c:137: if(bootloader_check_program_commnad())
	call	_bootloader_check_program_commnad
	ld	a,l
	or	a, a
	jr	Z,00102$
;main.c:138: uart_print("Comando OK! :D");
	ld	hl,#___str_2
	push	hl
	call	_uart_print
	pop	af
	jr	00103$
00102$:
;main.c:140: uart_print("No se recibio @");
	ld	hl,#___str_3+0
	push	hl
	call	_uart_print
	pop	af
00103$:
;main.c:142: HALT();
	HALT
	ret
___str_1:
	.ascii "Esperando comando de programacion: @"
	.db 0x00
___str_2:
	.ascii "Comando OK! :D"
	.db 0x00
___str_3:
	.ascii "No se recibio @"
	.db 0x00
	.area _CODE
	.area _INITIALIZER
__xinit__ptr_int_isr:
	.dw #0x0038
__xinit__ptr_int_isr_l:
	.dw #0x0039
__xinit__ptr_int_isr_h:
	.dw #0x003A
__xinit__ptr_nmi_isr:
	.dw #0x0038
__xinit__ptr_nmi_isr_l:
	.dw #0x0067
__xinit__ptr_nmi_isr_h:
	.dw #0x0068
__xinit__app_main_addr:
	.dw #0x0080
	.area _CABS (ABS)
