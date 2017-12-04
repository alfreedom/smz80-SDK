;--------------------------------------------------------------------------------------
;  crt0.s - Archivo crt0.s para el mapa de memoria del Sistema Mínimo SMz80,
;			diseñado en el Laboratorio de Hardware Avanzado del Área de 
;			Computación en Informática de la Universidad Autónoma de San Luis Potosí
;			UASLP
;
;	Archivo de inicialización del Z80 para el runtime de C.
;
;	crt (del ingles C Run Time), es un archivo que inicializa el hardware,
;	(en este caso el Z80) para dar soporte a código en C.
;
;	Inicializa el Stack Pointer, define segmentos de código, vectores de
;	interrupción e iniciliaza las variables antes de mandar llamar a la
;	función main.
;
;	Autor:   Alfredo Orozco de la Paz
;   e-mail:  alfredoopa@gmail.com
;
;				  	 		  		  ________
; 						<============|  FRED  |============>
; 						<============|________|============>
;
;--------------------------------------------------------------------------------------

	.module init
	.globl	_main
	;.globl 	_isr_vector08
	;.globl 	_isr_vector10
	;.globl 	_isr_vector18
	;.globl 	_isr_vector20
	;.globl 	_isr_vector28
	;.globl 	_isr_vector30
	.globl 	_isr_vector38
	.globl 	_isr_vector66

	.area	_HEADER (ABS)
	;; Reset vector
	.org 	0
	jp	init

	.org	0x08
	;jp		_isr_vector08

	.org	0x10
	;jp		_isr_vector10

	.org	0x18
	;jp		_isr_vector18

	.org	0x20
	;jp		_isr_vector20

	.org	0x28
	;jp		_isr_vector28

	.org	0x30
	;jp		_isr_vector30

	.org	0x38
	jp		_isr_vector38

	.org	0x66
	jp		_isr_vector66


	;; Inicio del programa.
	;; El programa comienza en la dirección 80H, para salvaguardar 
	;; los vectores de interrupción.
	.org	0x80
init:

	;; Inicializa el Stack Pointer con la dirección más alta de la RAM.
	ld	sp,#0xFFFF

    ;; Inicializa las variables globales
    call    gsinit
	call	_main
	halt

	;; Orden de los segmentos para el enlazador
	.area	_HOME
	.area	_CODE
	.area	_INITIALIZER
	.area   _GSINIT
	.area   _GSFINAL

	.area	_DATA
	.area	_INITIALIZED
	.area	_BSEG
	.area   _BSS
	.area   _HEAP

	.area   _CODE

	.area   _GSINIT

	;; Rutina de inicialización de variables.
gsinit::
	ld	bc, #l__INITIALIZER
	ld	a, b
	or	a, c
	jr	Z, gsinit_next
	ld	de, #s__INITIALIZED
	ld	hl, #s__INITIALIZER
	ldir
gsinit_next:
	.area   _GSFINAL
	ret

