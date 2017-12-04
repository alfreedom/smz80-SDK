
/***************************************************************************
 *  packet.h
 *  
 *         Versión:  1.0
 *
 *           Autor:  Alfredo Orozco de la Paz
 *                   Sergio Arturo Torres Ramirez
 *         Archivo:  packet.h
 *           Fecha:  18 de Marzo del 2016
 *
 *      Procesador:  Z80 CPU
 *      Compilador:  SDCC
 *
 *    ############################# Última Modficación #############################
 *      
 *    ##############################################################################
 ***************************************************************************************
 *  Descripción:
 *  
 *      Contiene los metodos para la lectura  y comprobacion de paquetes.
 *      Defindos segun la estructura packet_t.
 *                         
 ***************************************************************************************/
#ifndef _PACKET_H_
#define _PACKET_H_

#include "z80utils.h"
#include "z80uart.h"

#define PACKET_MARK ':'             //Indica el inicio de un paquete.
#define MAX_PACKET_READ_INTENTS 50 // Numero de intentos máximos para leer un paquete

typedef enum{
    PACKET_TYPE_DATA        = 'D',
    PACKET_TYPE_ACK         = 'A',
    PACKET_TYPE_NAK         = 'N',
    PACKET_TYPE_ADDRES      = 'S',
    PACKET_TYPE_BREAK       = 'B',
    PACKET_TYPE_FILE_HEADER = 'F',
    PACKET_TYPE_EOF         = 'Z',
    PACKET_TYPE_ERROR       = 'E'
}packet_type_t;

typedef struct
{
    uint8_t mark;        // Identificador de inicio de paquete.
    uint8_t data_length; // Longitud de campo de los datos.
    uint8_t number;    // Numero de paquete.
    uint8_t type;   // Tipo de Paquete.
    uint8_t data[255];   // Campo de datos.
    uint8_t checksum;    // campo para comprobacion de errores.

}packet_t;

/**
 * @brief Crea un paquete nuevo
 * @details Llena un paquete con la información proporcionada.
 * 
 * @param nuevo Paquete a llenar
 * @param packet_type Tipo del Packete
 * @param packet_number Numero del paquete
 * @param packet_data Buffer de datos del paquete
 * @param data_length tamaño del buffer de datos del paquete.
 */
void packet_fill(packet_t *nuevo, uint8_t packet_type, uint8_t packet_number, uint8_t* packet_data, uint8_t data_length);


/**
 * @brief Lee un paquete de la UART
 * @details El sistema espera hasta recibir el caracter de inicio de paquete ':' al recibirlo
 *          guarda la informacion en la estructura y retorna el paquete para comprobarlo
 * @param nuevo paquete capturado del puerto
 * @return 1 si se leyó algún paquete, 0 si no hay paquetes disponibles
 */
uint8_t packet_read(packet_t *nuevo);

/**
 * @brief Envia un paquete
 * @details Envia el paquete especificado por medio de la UART, en el orde especifico por el protocolo.
 * 
 * @param p [description]
 */
void packet_send(packet_t *p);

/**
 * @brief Comprobacion de paquete
 * @details Revisa si el paquete no perdio informacion al ser trasmitido mediante la comparacion de]
 *          checksum.
 * @param p Paquete que va a ser comprobado.
 * @return bandera que indica si el paquete esta dañado o correcto 1=correcto 0=dañado
 */
uint8_t packet_check(packet_t *p);
//void guardarPaquete(packet_t *p);

void packet_fill(packet_t *nuevo, uint8_t packet_type, uint8_t packet_number, uint8_t* packet_data, uint8_t data_length){

    uint8_t checksum=0;
    int i;
    nuevo->mark = PACKET_MARK;
    checksum += PACKET_MARK;
    nuevo->data_length = data_length;
    checksum+= data_length;
    nuevo->number = packet_number;
    checksum+= packet_number;
    nuevo->type = packet_type;
    checksum+= packet_type;
    
    for (i= 0; i < data_length; ++i)
    {
        nuevo->data[i] = packet_data[i];
        checksum+= packet_data[i];
    }

    nuevo->checksum = checksum;

}
uint8_t packet_check(packet_t *p)
{
    int i;
    uint8_t check_sum=0;

    check_sum+= p->mark;
    check_sum+= p->data_length;
    check_sum+= p->number;
    check_sum+= p->type;

    for (i= 0; i < p->data_length; ++i)
        check_sum+= p->data[i];

    //check_sum = ~check_sum;  //calculo de checksum de paquete recibido
    if(check_sum == p->checksum)
        return 1;
    else 
        return 0;
}

uint8_t packet_read(packet_t *nuevo)
{
    uint8_t c=' ';
    int intent_count = 0;
    int i;
    // si no hay suficientes datos en el buffer devuelve 0
    //if(uart_available() < 5)
      //  return 0;
        
    // Mientras no reciba inicio de paquete incrementa los intentos de lectura
    while((c=uart_read()) != PACKET_MARK)
    {
       intent_count++;
         if(intent_count>=MAX_PACKET_READ_INTENTS)
          return 0;
    }
    nuevo->mark = c;                    // Asigna marca a paquete
    nuevo->data_length =uart_read();   // Lee numero de datos que contiene el paquete.
    nuevo->number=uart_read();     // Lee el numero de paquete
    nuevo->type=uart_read();     // Lee tipo de paquete.
   for(i= 0; i< nuevo->data_length; i++)
    {
            nuevo->data[i]=uart_read(); // Lee los datos del paquete.
    }
    nuevo->checksum = uart_read();      // Lee el checksum de el paquete.
    return 1;
}

void packet_send(packet_t *p){

    uart_write(p->mark);            // Envía la marca.
    uart_write(p->data_length);     // Envia el tamaño de datos.
    uart_write(p->number);      // Envía el número de paquete.
    uart_write(p->type);     // Envía el tipo de paquete.
    uart_write_buffer(p->data, p->data_length); // Envia los datos del paquete.
    uart_write(p->checksum);        // Envia el checksum del paquete.

}
/*
void guardarPaquete(packet_t *p)
{
   // hace esto LD (DE),(HL) y decrementa BC
  __asm
       ld HL,p->data
       ld DE,p->data
       ld BC,p->data_length  
       ldi   
    __endasm;
}
*/
#endif // _PACKET_H_
