/***************************************************************************
 *  main.c
 *
 *  Corrimiento de 1 bit sobre los puertos A, B y C del PPI.
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

    int val, dir, i, j;
    // Inicialización del sistema.
    system_init(); 
    
    val = 0x01;
    dir = 0x01;
    
    // Ciclo infinito del programa.
    while(TRUE){
        
        /* CÓDIGO AQUI*/
        for( i = 0; i < 64; i++) 
        {
            
            PPI_PORTA = val;
            PPI_PORTB = val;
            PPI_PORTC = val;
            
            if(dir)
                val <<= 1;
            else
                val >>= 1;
                
            if(val == 0 | val > 0x80)
            {
                dir = !dir;
                if(val == 0)
                    val = 1;
                else
                    val = 0x80;
            }
            
            delay_ms(50);
        }
        
        for(i = 0; i < 4; i++) {
            
            PPI_PORTA = PPI_PORTB = PPI_PORTC = 0x81;
            delay_ms(50);
            PPI_PORTA = PPI_PORTB = PPI_PORTC = 0x42;
            delay_ms(50);
            PPI_PORTA = PPI_PORTB = PPI_PORTC = 0x24;
            delay_ms(50);
            PPI_PORTA = PPI_PORTB = PPI_PORTC = 0x18;
            delay_ms(50);
            PPI_PORTA = PPI_PORTB = PPI_PORTC = 0x18;
            delay_ms(50);
            PPI_PORTA = PPI_PORTB = PPI_PORTC = 0x24;
            delay_ms(50);
            PPI_PORTA = PPI_PORTB = PPI_PORTC = 0x42;
            delay_ms(50);
            PPI_PORTA = PPI_PORTB = PPI_PORTC = 0x81;
            delay_ms(50);

        }
        
        val = 0;
        for (i = 0; i < 8; i++) {
            dir = 0x80;
            
            for (j = 0; j < 7-i; j++) {
                PPI_PORTA = PPI_PORTB = PPI_PORTC = val | dir;
                dir>>=1;
                delay_ms(90);
            }
            val |= dir;
            
            
        }
        
        delay_ms(2000);
        PPI_PORTA = PPI_PORTB = PPI_PORTC = 0;
        delay_ms(500);
        PPI_PORTA = PPI_PORTB = PPI_PORTC = 255;
        delay_ms(500);
        PPI_PORTA = PPI_PORTB = PPI_PORTC = 0;
        delay_ms(500);
        PPI_PORTA = PPI_PORTB = PPI_PORTC = 255;
        delay_ms(500);
        PPI_PORTA = PPI_PORTB = PPI_PORTC = 0;
        delay_ms(500);
        PPI_PORTA = PPI_PORTB = PPI_PORTC = 255;
        delay_ms(500);
        PPI_PORTA = PPI_PORTB = PPI_PORTC = 0;
        delay_ms(500);
        PPI_PORTA = PPI_PORTB = PPI_PORTC = 255;
        delay_ms(500);

    }      

}

