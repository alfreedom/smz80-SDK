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

#define ISR_NMI()     void isr_vector66()  __critical __interrupt   /**< Función para el vector de interrupción NO ENMASCARABLE 0x66 */
#define ISR_INT_38()  void isr_vector38()  __interrupt 7    /**< Función para el vector de interrupción 0x38 */
#define PPI_BASE_ADDR  0x00
#define UART_BASE_ADDR 0x10
#define write_byte_EEPROM_RAM  0xA000 
#define delay_1ms_RAM 0xB000


#include "z80bootloader.h"
#include "z80utils.h"
#include "z80uart.h"
#include "smz80.h"
 int cont;
 int flag;


void test_program_command();
 
ISR_NMI(){
    // NO SE USA EN EL BOOTLOADER, PERO SIEMPRE SE DEFINE PORQUE ES
    // INTERRUPCION NO ENMASCARABLE Y SE PUEDE LANZAR EN CUALQUIER
    // MOMENTO.
}

ISR_INT_38(){
    // NO SE USA EN EL BOOTLOADER,  SE USA LA DEFINIDA POR LA UART.
}

void init_system(){
 PPI_CTRL=0x80;
 // Se copean las funciones para escritura en eeprom en ram 
 write_byte_EEPROM_ptr = (void*)write_byte_EEPROM_RAM;  // apuntador de fincion guardada en ram para escribir un byte en eeprom 
 delay_1ms_ptr = (void*)delay_1ms_RAM; // apuntador de funcion guardada en ram para esperar un mili-segundo.
 copeaBloque((uint16_t)&write_byte,write_byte_EEPROM_RAM,0x50); // copea funcion write_byte de eprom a ram.
 copeaBloque((uint16_t)&delay_1ms,delay_1ms_RAM,0x30);// copea funcion de delay_1ms de eeprom a ram.
 bootloader_init();
}

int main(){
    //int i;
    packet_t pkg_out;
    //init_system();
    //for(i=0;i<10;i++)
    //{
    //eeprom_write(0x0080+i,0x76);//escribe halt en direccion 80
    //}
    //HALT();
  //int i;//Variable usada como dato para prueba de escritura en eeprom
  //uint16_t j; //Variable usada como direccion para prueba de escritura en eeprom  
  //uint8_t d;
  //uint8_t datos[10] = {0,1,2,3,4,5,6,7,8,9};
  //packet_t nuevo;
    // Inicialización del sistema.
    init_system();
    /* Prueba del comando de programacion */
   // test_program_command();
    // Checa si se recibio un comando de programación
    // para el bootloader.
    
    /*
    // Prueba de lectura en eeprom
    /eeprom_read(0x0000,&d);
    if(d== 0xC3)
    {
    eeprom_write(0x0080,0x76);
         }
    */
  

    uart_write('1');
    ´// HALT();
    /*****Prueba de escritura sucesiva en eeprom*****/
    //eeprom_write_buffer(0x0080, datos,10);
    //HALT();
    /************************************************/
    if(bootloader_check_program_commnad())
     {
        uart_print("OK");
        // si se recibio el comando de programacion, 
        // ejecuta las rutinas del bootloader y checa 
        // si se ejecuto correctamente.
        
        // Envia respuesta al host para saber que entro en
        // modo bootloader.
        // Si no se grabo correctamente la aplicación, graba la instrucción
        // HALT en la direccion de la aplicación
        
        if(!bootloader_run())
        {
            //uart_print("No se grabo programa");
            eeprom_write(0x0080,0x76);//escribe halt en direccion 80
            delay_ms(100);
            //cleanRAM();
            __asm
                call  #0x0080 //salta a la aplicación 
            __endasm;
            nop();
        }
        packet_fill(&pkg_out, PACKET_TYPE_EOF,200, NULL, 0);
        packet_send(&pkg_out);
    }
    //cleanRAM(write_byte_EEPROM_RAM,0x40);
    //uart_print("salto a aplicacion");
    bootloader_start_app();
    
    
   return 0;
 }

void test_program_command() {
    
    uart_print("Esperando comando de programacion: @");
    if(bootloader_check_program_commnad())
        uart_print("Comando OK! :D");
    else
        uart_print("No se recibio @");
        
    HALT();
}