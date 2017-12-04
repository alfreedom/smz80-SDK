#ifndef _BOOTLOADER_H_
#define _BOOTLOADER_H_

#include "z80utils.h"
#include "z80uart.h"
#include "z80eeprom.h"
#include "packet.h"
// Tiempo de espera para recibir un comando de programación
// de bootloader antes de entrara a la aplicación.
#define BOOTLOADER_PROGRAM_COMMAND_TIMEOUT    800                                           

// Tamaño de la aplicación en bytes.
#define APP_SIZE EEPROM_SIZE - BOOT_START_ADDR - BOOT_RESET_ADDR

// Comando de inicio para entrara en el modo programación del bootloader.
#define BOOTLOADER_PROGRAM_COMMAND    '@'

#define INT_ISR_ADDR  0x0038
#define INT_ISR_ADDR_L  0x0039
#define INT_ISR_ADDR_H  0x003A

#define NMI_ISR_ADDR  0x0066
#define NMI_ISR_ADDR_L  0x0067
#define NMI_ISR_ADDR_H  0x0068

#define MAX_READS_INTENTS  10
//#define MEM_BUFFER_SIZE  4096//4Kb
#define MEM_BUFFER_SIZE  16384 //16 Kb

/**
 * @brief Inicializa el bootloader
 * @details Inicializa la UART para ser usada por el bootloader.
 */
void bootloader_init();
uint8_t bootloader_check_program_commnad();
int bootloader_run();
void bootloader_start_app();

// Apuntador a la dirección donde se guarda la rutina de interrupción
// INT en la memoria EEPROM, la dirección 0x38 contiene la instruccion JP.
static uint16_t* ptr_int_isr = (uint16_t*)INT_ISR_ADDR;
static uint8_t* ptr_int_isr_l = (uint8_t*)INT_ISR_ADDR_L;
static uint8_t* ptr_int_isr_h = (uint8_t*)INT_ISR_ADDR_H;

// Apuntador a la dirección donde se guarda la rutina de interrupción
// NMI en la memoria EEPROM, la dirección 0x38 contiene la instruccion JP.
static uint16_t* ptr_nmi_isr = (uint16_t*)INT_ISR_ADDR;
static uint8_t* ptr_nmi_isr_l = (uint8_t*)NMI_ISR_ADDR_L;
static uint8_t* ptr_nmi_isr_h = (uint8_t*)NMI_ISR_ADDR_H;

// Variable para guardar direccion auxiliar
static uint8_t aux_address_l;
static uint8_t aux_address_h;

// Variable para guardar la rutina de interrupcion INT de la aplicación
static uint16_t old_app_int_isr_addr;
static uint8_t old_app_int_isr_addr_l;
static uint8_t old_app_int_isr_addr_h;

// Variable para guardar la rutina de interrupcion NMI de la aplicación
static uint16_t old_app_nmi_isr_addr;
static uint8_t old_app_nmi_isr_addr_l;
static uint8_t old_app_nmi_isr_addr_h;

// Dirección de inicio de la aplicación.
static uint16_t app_main_addr = 0x0080;

// variable para guardar el paquete leido
static volatile packet_t pkg_in;

// variable para guardar el paquete a enviar
static volatile packet_t pkg_out;


void bootloader_init(){

    /*Código para inicializar los dispositivos E/S del Z80 y las variables del sistema*/
    
    uart_cfg_t uart_config; // Estructura para configuración de la UART
   
    // Configura los aprametros de la UART.
    uart_config.divisor    =  UART_MR ;   // divisor x64.

    // Inicializa la UART con su configuración.
    uart_init(&uart_config);
    // Inicializa la UART a 57600 8N1 con interrupcion habilitada por dato recibido.
   delay_ms(500);
    uart_config.configuracionDePalabra   = UART_8BITS_1STOPBIT;     // 1 bit de parada, 8 bits por palabra.
    uart_config.interrupcionesDelReceptor      = UART_RECIVE_INTERRUPT_ENABLE;     // interrupcion de recepcion 
    uart_config.interrupcionesDelTransmisor =  UART_TRANSMIT_INTERRUPT_DISABLE_RTS_LOW;   // Dato de 8 bits
    uart_config.divisor = UART_DIV_64;
    
    uart_init(&uart_config);

    // Guarda las direcciones de las rutinas de servicio de interrupciones de la aplicación
    // para sustituirlas por las del bootloader.
    old_app_int_isr_addr = *ptr_int_isr;    
    old_app_int_isr_addr_l = *ptr_int_isr_l; 
    old_app_int_isr_addr_h = *ptr_int_isr_h;  
    old_app_nmi_isr_addr = *ptr_nmi_isr;    
    old_app_nmi_isr_addr_l = *ptr_nmi_isr_l; 
    old_app_nmi_isr_addr_h = *ptr_nmi_isr_h;    
    
    // Escribe la dirección de la rutina de servicio de interrupción de la uart
    eeprom_write((uint16_t)(ptr_int_isr_l),(uint8_t)&uart_interrupt_isr);
    eeprom_write((uint16_t)ptr_int_isr_h,(uint8_t)((uint16_t)(&uart_interrupt_isr)>> 8));
    IM(1);  // Modo de interrupción 1
    EI();   // Habilita interrupciones.
}

uint8_t bootloader_check_program_commnad(){

    int time_spend=0;

    // ciclo de espera.
    // Espera a que se reciba el comando de programacion o hasta
    // que se agote el tiempo de espera de este comando
    while(1){

        // Si hay datos disponibles para leer de la uart.
        if(uart_available()){
            // lee un dato y si es comando de programacion, regresa 1. de lo contrario espera
            if(uart_read() == BOOTLOADER_PROGRAM_COMMAND)
            {
                
                return 1;
            }
                            }   
            // Hce un retardo de 1 ms
            delay_ms(1);
           // Incremente al contador de tiempo transcurrido
             time_spend+=1;
        
            // Si el tiempo de espera transcurrido es mayor que el tiempo de espera máximo
           // devuelve 0, no se recibio comando de programación
           if(time_spend >= BOOTLOADER_PROGRAM_COMMAND_TIMEOUT)
            return 0;
    }

}

int bootloader_run(){

    uint8_t is_exit=0;
    uint8_t intent_count=0;
    uint16_t eeprom_addr=0x0080;
    uint16_t app_program_size = 0;
    uint8_t mem_buffer[MEM_BUFFER_SIZE];
    int mem_buffer_index=0;
    delay_ms(300);
    
    while(is_exit==0) 
    {

        // Si hay paquete disponible.
        if(packet_read(&pkg_in))
        {
            intent_count=0;
            // checa errores en el paquete.
            if(packet_check(&pkg_in) == 0){
                // si el paquete tiene errores, llena el paquete de salida con NAK y envia el paquete
                packet_fill(&pkg_out, PACKET_TYPE_NAK,pkg_in.number, NULL, 0);
                // Envía el paquete
                packet_send(&pkg_out);
                uart_flush();
            }
            else  // Si no hay errores en el paquete, checa el tipo de paquete y lo procesa.
            {
                // Crea un packete ACK.
                packet_fill(&pkg_out, PACKET_TYPE_ACK, pkg_in.number, NULL, 0);
                uart_flush();

                // checa el paquete recibido.
                switch(pkg_in.type){
                    case PACKET_TYPE_ADDRES: // Si es paquete de direccion.
                        // guarda la dirección en la variable de dirección para la eeprom.
                        // Si la dirección actual de la eeprom es 0x39, lee
                        // la dirección del vector de interrupciones del paquete.
                        // Este es el vector de interrupcion de la aplicación.
                         if(INT_ISR_ADDR == pkg_in.data[1] && INT_ISR_ADDR>>8 == pkg_in.data[0])
                            {
                                //old_app_int_isr_addr =  eeprom_addr;
                                aux_address_l= pkg_in.data[1];
                                aux_address_h= pkg_in.data[0];
                            }
                            else
                            if(NMI_ISR_ADDR == pkg_in.data[1] && NMI_ISR_ADDR>>8 == pkg_in.data[0]){
                                //old_app_int_isr_addr =  eeprom_addr;
                                aux_address_l= pkg_in.data[1];
                                aux_address_h= pkg_in.data[0];
                            }
                   
                    break;
                    case PACKET_TYPE_DATA:  // Si es paquete de datos.

                            if(INT_ISR_ADDR == aux_address_l && INT_ISR_ADDR>>8 == aux_address_h){
                                //old_app_int_isr_addr =  eeprom_addr;
                                old_app_int_isr_addr_l= pkg_in.data[0];
                                old_app_int_isr_addr_h= pkg_in.data[1];
                                aux_address_l=0x00;
                                aux_address_h=0x00;
                            }
                            else
                            if(NMI_ISR_ADDR == aux_address_l && NMI_ISR_ADDR>>8 == aux_address_h){
                                //old_app_int_isr_addr =  eeprom_addr;
                                old_app_nmi_isr_addr_l= pkg_in.data[0];
                                old_app_nmi_isr_addr_h= pkg_in.data[1];
                                aux_address_l=0x00;
                                aux_address_h=0x00;
                            }
                            else{
                            // guarda los datos en el buffer
                            memcpy(&mem_buffer[mem_buffer_index], pkg_in.data, pkg_in.data_length);
                            mem_buffer_index += pkg_in.data_length;
                            }
                            /*
                            if(eeprom_write_buffer(eeprom_addr, pkg_in.data, pkg_in.data_length)){
                                eeprom_addr += pkg_in.data_length;
                            }
                            else // Si no se pudo escribir en la eeprom, significa que se intentó escribir en una dirección del bootloader.
                            {
                                // Crea un paquete ERROR.
                                packet_fill(&pkg_out, PACKET_TYPE_ERROR,pkg_in.number, NULL, 0);
                                return 0;
                            }
                            */
                        //uart_write('D');
                    break;

                    case PACKET_TYPE_EOF:  // Si es paquete de fin de archivo
                        is_exit=1;         // Termina el programa bootloader correctamente.
                    break;

                    case PACKET_TYPE_FILE_HEADER:
                        
                        // guarda el tamaño del programa.
                        app_program_size = *(uint16_t*)pkg_in.data;

                        if(app_program_size >= APP_SIZE){
                            // Crea un paquete ERROR.
                            packet_fill(&pkg_out, PACKET_TYPE_ERROR,pkg_in.number, NULL, 0);
                            return 0;
                        }
                    break;

                }
                // Envía el paquete
                packet_send(&pkg_out);
            }
        }
        else // si no hay paquete disponible.
        {
            intent_count++;

            if(intent_count >= MAX_READS_INTENTS)
            {
                return 0;
            }
        }
    }


   // eeprom_erase(0x0080, MEM_BUFFER_SIZE);
    eeprom_write_buffer(0x0080, mem_buffer, mem_buffer_index+1);
    packet_fill(&pkg_out, PACKET_TYPE_EOF,pkg_in.number, NULL, 0);
    delay_ms(500);
    packet_send(&pkg_out);
    packet_send(&pkg_out);
    packet_send(&pkg_out);
    return 1;
}



void bootloader_start_app(){

    // Restablece la dirección de rutina de interrupcion INT de la
    // aplicacion, no del bootloader.
    eeprom_write((uint16_t)ptr_int_isr_l,old_app_int_isr_addr_l);
    eeprom_write((uint16_t)ptr_int_isr_h,old_app_int_isr_addr_h);
    eeprom_write((uint16_t)ptr_nmi_isr_l,old_app_nmi_isr_addr_l);
    eeprom_write((uint16_t)ptr_nmi_isr_h,old_app_nmi_isr_addr_h);
    // Si no hay instrucciones a ejecutar graba instrucción HALT
    //if(*((uint16_t*)(0x0080)) == 0x00 || *((uint16_t*)(0x0080)) == 0xFF)
    if(*((uint8_t*)(0x0080)) == 0x00 || *((uint8_t*)(0x0080)) == 0xFF)
    {
      eeprom_write(0x0080,0x76);
    }
    // llama a la aplicación.
    __asm
        call  #0x0080
    __endasm;      
}


/*void bootloader_start_app(){
    uint8_t byte;
    // Restablece la dirección de rutina de interrupcion INT de la
    // aplicacion, no del bootloader.
    eeprom_write((uint16_t)ptr_int_isr_l,(uint8_t)old_app_int_isr_addr_l);
    eeprom_write((uint16_t)ptr_int_isr_h,(uint8_t)old_app_int_isr_addr_h);
    // Si no hay instrucciones a ejecutar graba instrucción HALT
    eeprom_read(0x0080,&byte);
    if(byte == 0x00 || byte == 0xFF)
    {
      eeprom_write((uint16_t)0x0080,(uint8_t)0x76);
    }
    // llama a la aplicación.
    __asm
        call  #0x0080
    __endasm;      
}*/

#endif // _BOOTLOADER_H_
