/***************************************************************************
 *  main.c
 *
 *  Plantilla para un programa en C para le microprocesador Z80 con
 *  el compilador SDCC.
 *  
 *  Se incluye la librería smz80.h, que es una API con funciones para
 *  el PPI 8255, la UART 8250, funciones para puertos de E/S, funciones
 *  de delay y macros con instrucciones para el Z80 a bajo nivel. 
 *
 ***************************************************************************
 *
 *      Autor:  Alfredo Orozco de la Paz
 *      e-mail: alfredoopa@gmail.com 
 *                                                                  *FRED*
 ***************************************************************************
 */

/**
 * Definicion de las direcciones para el PPI y la UART
 * Si no se definen, no se pueden usar las funciones de la librería smz80.h.
 * 
 * se deben definir antes de incluir la librería.
 */
#define PPI_BASE_ADDR  0x00
#define UART_BASE_ADDR 0x10

#include "smz80.h"

ISR_NMI(){

    /* Código de servicio de la interrupción NMI.*/
}

ISR_INT_38(){
    
    /* Código de servicio de la interrupción INT.*/
}


void system_init(){

    /*Código para inicializar los dispositivos E/S del Z80 y las variables del sistema*/
    PPI_CTRL = 0x80;
}

int main(){

    // Inicialización del sistema.
    system_init(); 
    

    // Ciclo infinito del programa.
    while(TRUE){
        /* CÓDIGO AQUI*/
        PPI_PORTA = 0;
        PPI_PORTB = 0;
        PPI_PORTC = 0;
        delay_ms(300);
        
        PPI_PORTA = 0xFF;
        PPI_PORTB = 0xFF;
        PPI_PORTC = 0xFF;
        delay_ms(300);
    }      

}
