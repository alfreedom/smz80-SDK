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
#define UART_BASE_ADDR 0x70

#include "smz80.h"

ISR_NMI(){

    /* Código de servicio de la interrupción NMI.*/
}

ISR_INT_38(){
    URTHR = URRBR;
    PPI_PORTA=0xFF;
    EI();
}


void system_init(){

    /* Código para inicializar los dispositivos E/S del Z80 y las variables del sistema */
    uart_cfg_t uart_config; // Estructura para configuración de la UART
   
    // Inicializa la UART a 9600 8N1 con interrupcion habilitada por dato recibido.
    uart_config.baudrate    = UART_BAUDRATE_9600; 
    uart_config.stop_bits   = UART_STOP_BITS_1;
    uart_config.parity      = UART_PARITY_NONE;
    uart_config.word_length = UART_WORD_LENGTH_8;
    uart_config.interrupt   = UART_INTERRUPT_RX;

    uart_init(&uart_config);
    
    // Incializa todos los puertos de PPI como salida
    PPI_CTRL = 0x80;
    
    IM(1);
    EI();
}

int main(){

    // Inicialización del sistema.
    system_init(); 
    
    // Imprime mensaje de prueba
    uart_print("Ejemplo de eco con la UART, escriba lo que sea :D:\n");
    
    //Entra en modo sleep (HALT)
    sleep();    
    
    // Ciclo infinito del programa.
    while(TRUE){
        

    }      

}
