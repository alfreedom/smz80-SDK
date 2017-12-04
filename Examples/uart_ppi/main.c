/***************************************************************************
 *  main.c
 *
 *  Ejemplo de control de Leds con la UART y el PPI con la biblioteca smz80.h
 * 
 *  El programa configura la uart a 9600 8N1 con la interrupción por dato
 *  recibido habilitada, por lo que hay que asegurarse de que la señal
 *  interrupción INT de la placa UART esté conectada a la señal INT de la
 *  placa BASE, mediante el JUMPER de la placa UARTT.
 * 
 *  Lo que hace el programa es atender la interrupción de la UART y leer 
 *  el dato que se recibió del registro de la UART, lo devuelve y cambia
 *  la bandera de estado de los leds. El CPU se mantiene en HALT hasta que
 *  se reciba la interrupción, cuando despierta y cambia la bandera checa
 *  el estado de esta y si es 1 enciende los leds, de lo contrario los apaga
 *  y en seguida vuelve a entrar en estado HALT.
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

#define UART_BASE_ADDR 0x70
#define PPI_BASE_ADDR 0x00
#include "smz80.h"

int state;

/* Función de servicio de interrupción NMI.
   Esta función es llamada cada que se genera una
   interrupción en el PIN NMI, sin importar lo que
   esté haciendo el procesador. Si no se usa esta
   interrupción, la función se debe dejar vacía, 
   pero no se debe borrar, ya que si es borra y 
   por error se activa la interrupción NMI, el
   programa intentará ejecutar lo que exista en
   el vector de interrupción NMI y obtendremos
   un comportamiento del programa impredecible.*/
ISR_NMI(){

    /* Código de servicio de la interrupción NMI.*/
}

/* Función de servicio de interrupción INT.
   Esta función es llamada cada que se genera una
   interrupción en el PIN INT del z80, siempre y
   cuando la bandera de interrupción este activa
   en el cpu.*/
ISR_INT_38(){
    
    char c;

    c=uart_read();  // Lee el dato recibido
    uart_write(c);  // Envía el dato recibido
    
    // Cambia la bandera de estado.
    if(state == 0)
        state = 1;
    else
        state = 0;
    
    // Vuelve a habilita la interrupción INT.    
    EI();
}


void system_init(){

    // Estructura de configuración para la UART.
    uart_cfg_t uart_config;

    state = 0;          //Estado de los Leds a 0;
    PPI_CTRL = 0x80;    // Programa el PPI con todos los puertos como salida.

    // Configura los parámetros de la UART.
    uart_config.baudrate    = UART_BAUDRATE_9600;   // Baudaje a 1200.
    uart_config.stop_bits   = UART_STOP_BITS_1;     // 1 bit de parada.
    uart_config.parity      = UART_PARITY_NONE;     // Sin paridad
    uart_config.word_length = UART_WORD_LENGTH_8;   // Dato de 8 bits
    uart_config.interrupt   = UART_INTERRUPT_RX;    // Interrupción por dato recibido activada.

    // Inicializa la UART con su configuración.
    uart_init(&uart_config);

    IM(1);              // Modo de interrupcion 1 en el Z80.
    EI();               // Habilita la interrupción INT en el Z80
    
}

int main(){
    
    
    // Inicialización del sistema.
    system_init(); 
    
    // Ciclo infinito del programa.
    while(TRUE){
        
        // Si el estado es 0, apaga los leds del puerto C
        // si el estado es 1, enciende los leds del puerto C
        if(state==0)
            PPI_PORTC = 0X00;
        else
            PPI_PORTC = 0xff;

        halt();
    }      

}