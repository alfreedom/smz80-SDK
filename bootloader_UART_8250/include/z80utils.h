#ifndef _Z80UTILS_H_
#define _Z80UTILS_H_

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h> 

#define F_CPU    8000000  // Frecuencia del reloj del Z80

#define TRUE 1      ///< Definición para verdadero.
#define FALSE 0     ///< Definición para falso.

#define INPUT TRUE      ///< Definición para Entrada.
#define OUTPUT FALSE    ///< Definición para Salida.

#define LOW FALSE       ///< Definición para Bajo.
#define HIGH TRUE       ///< Definición para Alto.

/************************************************************************
*          DEFINICION DE FUNCIONES A VECTORES DE INTERRUPCION
************************************************************************/
#define ISR_NMI()     void isr_vector66()  __critical __interrupt   /**< Función para el vector de interrupción NO ENMASCARABLE 0x66 */
#define ISR_INT_08()  void isr_vector08()  __interrupt 1    /**< Función para el vector de interrupción 0x08 */
#define ISR_INT_10()  void isr_vector10()  __interrupt 2    /**< Función para el vector de interrupción 0x10 */
#define ISR_INT_18()  void isr_vector18()  __interrupt 3    /**< Función para el vector de interrupción 0x18 */
#define ISR_INT_20()  void isr_vector20()  __interrupt 4    /**< Función para el vector de interrupción 0x20 */
#define ISR_INT_28()  void isr_vector28()  __interrupt 5    /**< Función para el vector de interrupción 0x28 */
#define ISR_INT_30()  void isr_vector30()  __interrupt 6    /**< Función para el vector de interrupción 0x30 */
#define ISR_INT_38()  void isr_vector38()  __interrupt 7    /**< Función para el vector de interrupción 0x38 */
/***********************************************************************/

/************************************************************************
*                       DEFINICION DE MACROS
************************************************************************/
#define HALT()  __asm__ ("HALT")    ///< Macro para ejecutar la instrucción HALT del Z80.
#define halt()  __asm__ ("HALT")    ///< Macro para ejecutar la instrucción HALT del Z80.
#define Halt()  __asm__ ("HALT")    ///< Macro para ejecutar la instrucción HALT del Z80.
#define NOP()   __asm__ ("NOP")     ///< Macro para ejecutar la instrucción NOP del Z80.
#define nop()   __asm__ ("NOP")     ///< Macro para ejecutar la instrucción NOP del Z80.
#define Nop()   __asm__ ("NOP")     ///< Macro para ejecutar la instrucción NOP del Z80.
#define SLEEP() __asm__ ("HALT")    ///< Macro para poner al Z80 en modo Sleep.
#define sleep() __asm__ ("HALT")    ///< Macro para poner al Z80 en modo Sleep.
#define Sleep() __asm__ ("HALT")    ///< Macro para poner al Z80 en modo Sleep.

#define BV(BIT) (1<<(BIT))          ///< Macro que obtiene la mascara de bits para el bit BIT.
#define SFR_IO __sfr __at           ///< Macro para declarar variables a puertos de E/S.

#define RST(VECTR) __asm\
                     RST VECTR\
                   __endasm        ///< Macro para ejecutar la instrucción RST a un vector de interrupción del Z80.

#define EI()    __asm__ ("EI")  /**< Habilita interrupción INT */
#define DI()    __asm__ ("DI")  /**< Deshabilita interrupción INT */
#define IM(MODE) __asm\
                    IM MODE\
                __endasm        /**< Cambia el modo de la interrupción INT */


/************************************************************************
*                       DEFINICION DE TIPOS DE DATOS
************************************************************************/
typedef uint8_t     Byte;   ///< Definición de tipo de dato byte.
typedef uint8_t     byte;   ///< Definición de tipo de dato byte.
typedef uint16_t    Word;  ///< Definición de tipo de dato word.
typedef uint16_t    word;  ///< Definición de tipo de dato word.
typedef uint32_t    DWord; ///< Definición de tipo de dato double word.
typedef uint32_t    dword; ///< Definición de tipo de dato double word.
/***********************************************************************/

/************************************************************************
*       DEFINICION VARIABLES USADAS EN COPEO DE BLOQUES DE MEMORIA
************************************************************************/
 uint16_t dir_origin;
 uint16_t dir_destination;
 uint16_t size; 
 
 /***********************************************************************/
void (*delay_1ms_ptr)();
void delay_ms(int ms);
void delay_1ms();
void delay_10us();
void delay_100us();
void copeaBloque(uint16_t origen,uint16_t destino, uint8_t tam); 
//void cleanRAM(uint16_t destino, uint8_t tam);
void delay_1ms(){

    int i;
    int j;
  __asm
            EXX
            EX      AF,AF'
       __endasm;
    #if ( defined(F_CPU) )
    #if (F_CPU == 8000000)
     for(j=0;j<0x04;j++)
        for(i=0;i<0x1FF;i++)
            __asm__("nop");
    #elif (F_CPU == 10000000)
    for(j=0;j<0x04;j++)
        for(i=0;i<0x150;i++)
            __asm__("nop");
    #else
        #warning "No hay funciones de delay para el Z80 con la F_CPU definida"
    #endif 
    #else
        #warning "No se definio la frecuencia del Z80 (F_CPU) para las funciones de delay"
    #endif
    __asm
            EX      AF,AF'
            EXX
    __endasm;
}

void delay_ms(int ms){

    int i;
  __asm
            EXX
            EX      AF,AF'
       __endasm;
    #if ( defined(F_CPU) )
    #if (F_CPU == 8000000)
    
    while(ms--)
        for(i=0;i<0x10A;i++)
            __asm__("nop");
    #elif (F_CPU == 10000000)
    while(ms--)
        for(i=0;i<0x14E;i++)
            __asm__("nop");
    #else
        #warning "No hay funciones de delay para el Z80 con la F_CPU definida"
    #endif 
    #else
        #warning "No se definio la frecuencia del Z80 (F_CPU) para las funciones de delay"
    #endif
    __asm
            EX      AF,AF'
            EXX
    __endasm;
}

void delay_10us(){

    #if ( defined(F_CPU) )
    #if (F_CPU == 8000000)
    __asm
            EXX
            EX      AF,AF'
            LD      B,#0x2
    LOOP_10:
            DJNZ    LOOP_10
            EX      AF,AF'
            EXX
    __endasm;
    
    #elif (F_CPU == 10000000)
    // codigo para el retardo de 10us a 10mhz
    #else
        #warning "No hay funciones de delay para el Z80 con la F_CPU definida"
    #endif 
    #else
        #warning "No se definio la frecuencia del Z80 (F_CPU) para las funciones de delay"
    #endif
}

void delay_100us(){

    #if ( defined(F_CPU) )
    #if (F_CPU == 8000000)
    __asm
            EXX
            EX      AF,AF'
            LD      B,#0x3A
    LOOP_100:
            DJNZ    LOOP_100
            EX      AF,AF'
            EXX
            RET
    __endasm;
    #elif (F_CPU == 10000000)
    // codigo para el retardo de 100us a 10mhz
    #else
        #warning "No hay funciones de delay para el Z80 con la F_CPU definida"
    #endif 
    #else
        #warning "No se definio la frecuencia del Z80 (F_CPU) para las funciones de delay"
    #endif
}

void copeaBloque(uint16_t origen,uint16_t destino, uint8_t tam)
{
    dir_origin = origen;
    dir_destination=destino;
    size = tam;
    __asm
       LD HL,(_dir_origin) 
       LD DE,(_dir_destination)
       LD BC,(_size)
       LDIR
    __endasm;
}

/*void cleanRAM(uint16_t destino, uint8_t tam)
{
    dir_destination=destino;
    size = tam;
     __asm
            LD      HL,(_dir_origin) 
            LD      BC,(_size)
    LOOP_100:
            LD     (HL),#0X00
            INC    HL
            DEC    BC
            JP     NZ,LOOP_100
            RET
    __endasm;
}*/

#endif // _Z80UTILS_H_
