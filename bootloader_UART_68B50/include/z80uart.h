#ifndef _UART_H_
#define _UART_H_

#include "z80utils.h"
//#include "z80eeprom.h"


#ifndef UART_BASE_ADDR
#warning "No se definio la direccion base de la UART"
#warning "Para usar las funciones con la UART defina:     #define UART_BASE_ADDR 0xNN, antes de incluir smz80.h"
#endif



#define MAXLINE 100     /**< Numero maximo de caracteres a leer por ReadLine() */

#if defined (UART_BASE_ADDR)

/************************************************************************
*                   DEFINICION DE REGISTROS DE LA UART
************************************************************************/
#define UART_RBR_ADDR   UART_BASE_ADDR + 0x01    /**< Dirección del Registro RX (A0=1) */
#define UART_THR_ADDR   UART_BASE_ADDR + 0x01    /**< Dirección del Registro TX (A0=1) */
#define UART_CSR_ADDR   UART_BASE_ADDR + 0x00    /**< Dirección del Registro de control A0=0 */
/***********************************************************************/

/************************************************************************
*                 VARIABLES A LOS REGISTROS DE LA UART
************************************************************************/
SFR_IO  UART_RBR_ADDR  URRBR;   /**< Registro RBR - Receiver Buffer Register */
SFR_IO  UART_THR_ADDR  URTHR;   /**< Registro THR - Transmit Hold Register */
SFR_IO  UART_CSR_ADDR  URCONTROL;   /**< Registro de control*/

/* Bits si la señal R/W=0 se direcciona el Control Register */
#define UCDS1     0          /**Counter Divide Select 1 */
#define UCDS2     1              /**Counter Divide Select 2 */
#define UWS1      2              /**World select 1 */
#define UWS2      3              /**World select 2 */
#define UWS3      4              /**World select 3 */
#define UTC1      5              /**Trasmit Control 1 */
#define UTC2      6              /**Trasmit Control 2 */
#define URIE      7              /**Recive Interrupt Enable*/

/* Bits si la señal R/W=1  direcciona el Status Register  */
#define URDRF     0          /**Recive Data Register Full */
#define UTDRE     1              /**Trasmite Data register Empty*/
#define UDCD      2              /**Data Carrier Detect */
#define UCTS      3              /**Clear to send */
#define UFE       4              /**Framing Error */
#define UROVRN    5              /**Reseiver Overrun */
#define UPE       6              /**Parity Error */
#define UIRQ      7              /**Interrupt Request*/

/* Bits si la señal R/W=1  direcciona el Status Register  */
#define URDRF     0          /**Recive Data Register Full */
#define UTDRE     1              /**Trasmite Data register Empty*/
#define UDCD      2              /**Data Carrier Detect */
#define UCTS      3              /**Clear to send */
#define UFE       4              /**Framing Error */
#define UROVRN    5              /**Reseiver Overrun */
#define UPE       6              /**Parity Error */
#define UIRQ      7              /**Interrupt Request*/

/***********************************************************************/

/************************************************************************
*                     ENUMERACIONES DE LA UART
************************************************************************/
/**Configuracion de la frecuencia*/
/** Bits de configuracion  y su significado Registro URCONTROL*/
/** CR1 | CR0 | Fincion */
/**   0 |   0 | +1      */
/**   0 |   1 | +16     */
/**   1 |   0 | +64     */
/**   1 |   1 | Master Reset */

typedef enum{
    UART_DIV_1     =  0x00, /**Mantiene la frecuencia*/
    UART_DIV_16    =  0x01, /**Divide la frecuencia en 16*/
    UART_DIV_64    =  0x02, /**Divide la frecuencia en 64*/
    UART_MR        =  0x03 /**Master reset*/
}uart_div_t;

/** Bits de configuracion  y su significado Registro URCONTROL*/
/** CR4 | CR3 | CR2 | Fincion */
/** Configuracion de palabra*/
/**   0 |   0 |  0 | 7 bits + Even Parity + 2 Stop Bits     */
/**   0 |   0 |  1 | 7 bits + Odd Parity + 2 stop bits    */
/**   0 |   1 |  0 | 7 bits + even parity + 1 stop bit      */
/**   0 |   1 |  1 | 7 bits + odd parity + 1 stop bit  */
/**   1 |   0 |  0 | 8 bits + 2 Stop bits      */
/**   1 |   0 |  1 | 8 bits + 1 Stop bits      */
/**   1 |   1 |  0 | 8 bits + Event Parity + 1 Stop bit     */
/**   1 |   1 |  1 | 8 bits + Odd Parity + 1 stop bit  */

typedef enum{
    UART_7BITS_EVENTPARITY_2STOPBITS    = 0x00,         /**<7 bits + Even Parity + 2 Stop Bits */
    UART_7BITS_ODDPARITY_2STOPBITS      = 0x04,         /**<7 bits + Odd Parity + 2 stop bits */
    UART_7BITS_EVENTPARITY_1STOPBIT     = 0x08,         /**<7 bits + even parity + 1 stop bit */
    UART_7BITS_ODDPARITY_1STOPBIT       = 0x0C,         /**<7 bits + odd parity + 1 stop bit */
    UART_8BITS_2STOPBITS                = 0x10,         /**<8 bits + 2 Stop bits   */
    UART_8BITS_1STOPBIT                 = 0x14,         /**<8 bits + 1 Stop bits   */
    UART_8BITS_EVENTPARITY_1STOPBIT     = 0x18,         /**<8 bits + Event Parity + 1 Stop bit    */
    UART_8BITS_ODDPARITY_1STOPBIT       = 0x1C         /**<8 bits + Odd Parity + 1 stop bit  */
}uart_world_t;

/** Bits de configuracion  y su significado Registro URCONTROL*/
/** CR6 | CR5 | Funcion */
/**   0 |   0 | RTS = Low, Transmiting Interrupt Disable*/
/**   0 |   1 | RTS = LOW, Transmiting Interrupt Enable */
/**   1 |   0 | RTS = HIGH, Transmiting Interrupt Disable */
/**   1 |   1 | RTS = LOW, Transmits a break level on the transmit data output. Transmiting  interrup disable*/


typedef enum{
    UART_TRANSMIT_INTERRUPT_DISABLE_RTS_LOW             = 0x00,        /**< RTS = Low, Transmiting Interrupt Disable*/
    UART_TRANSMIT_INTERRUPT_ENABLE_RTS_LOW              = 0x20,        /**< RTS = Low, Transmiting Interrupt enable*/
    UART_TRANSMIT_INTERRUPT_DISABLE_RTS_HIGH            = 0x40,        /**< RTS = High, Transmiting Interrupt Disable*/
    UART_TRANSMIT_INTERRUPT_DISABLE_RTS_LOW_BREAK_LEVEL = 0x60        /**< RTS = Low, Transmits a break level on the transmit data output. Transmiting  interrup disable*/
}uart_trasmit_interrupt_t;

/** CR7 | Fincion */
/**   0 | Interrupt revive disable    */
/**   1 | Interrupt recive data register full, overrun, low-to-high transition on the datacarrier detect (DCD) signal line      */

typedef enum{
    UART_RECIVE_INTERRUPT_DISABLE  = 0x00,        /**< Interrupt revive disable */
    UART_RECIVE_INTERRUPT_ENABLE   = 0x80        /**< Interrupt revive enable */
}uart_recive_interrupt_t;

/************************************************************************/

/************************************************************************
*                 ESTRUCTURAS DE DATOS DE LA UART
************************************************************************/
typedef struct{
    uart_div_t divisor;/**< Divisor de frecuencia*/
    uart_world_t configuracionDePalabra; /**< Configuracion de la palabra*/
    uart_trasmit_interrupt_t interrupcionesDelTransmisor; /**< Interrupciones del transmisor*/
    uart_recive_interrupt_t interrupcionesDelReceptor; /**< Interrupciones del receptor*/
}uart_cfg_t;

#define UART_DEFAULT_CONFIG {       \
    UART_DIV_64,                    \
    UART_8BITS_1STOPBIT,            \
    UART_RECIVE_INTERRUPT_DISABLE,  \
    UART_TRANSMIT_INTERRUPT_DISABLE_RTS_LOW }
#endif // UART_BASE_ADDR

/***********************************************************************/

// 1Kb de tamaño para el buffer de entrada
#define UART_BUFFER_SIZE    1024
#define UART_TIME_READ      1500
volatile static uint8_t _uart_in_buffer[UART_BUFFER_SIZE];
volatile static int  _in_buffer_index;
volatile static int  _out_buffer_index;
volatile static uint8_t _is_interrupt_enable;


/**
 * @brief Configuracion de UART
 * @details Configura los parametrospara la comunicacion de la UART.
 * 
 * @param uart_config estructura donde se define la configuracion.
 */ 
void uart_init(const uart_cfg_t *uart_config);


/**
 * @brief Escribe un byte
 * @details Escribe un byte por medio de la UART
 * 
 * @param c Byte que se va a escribir.
 */ 
void uart_write(uint8_t c);
/**
 * @brief Escribe un conjunto de bytes
 * @details Escribe un conjunto de bytes por medio de la UART
 * 
 * @param buffer Bytes que se van a escribir.
 * @param count Numero de bytes a escribir.
 */
void uart_write_buffer(uint8_t* buffer, int count);
/**
 * @brief Lee un byte
 * @details Lee un byte por medio de la UART
 * 
 * Retorna el byte leido
 */ 
uint8_t uart_read();
/**
 * @brief Lee un conjunto de bytes
 * @details Lee un conjunto de bytes por medio de la UART
 * 
 * @param buffer apuntador donde se guardaran los bytes que se van a leer.
 * @param count Numero de bytes a leer.
 */
int  uart_read_buffer(uint8_t* buffer, int count);
/**
 * @brief    Hay datos en el buffer
 * @details  Indica si se han recibido datos y estan almacenados en el buffer
 * 
 */
int  uart_available();

//void uart_print(const char* str);
int  uart_read_line(uint8_t* str);
/**
 * @brief    Limpia buffer
 * 
 */
void uart_flush();
void printBuffer();
void uart_print(const uint8_t* str);


void uart_init(const uart_cfg_t *uart_config) {
    URCONTROL = (uart_config->divisor) | (uart_config->configuracionDePalabra) | (uart_config->interrupcionesDelTransmisor)| (uart_config->interrupcionesDelReceptor);
}

void uart_write(uint8_t c){
    // Mientras el registro transmisor no esté vacío...
    while( !(URCONTROL & BV(UTDRE)))
        NOP();       
    // Cargamos el dato a enviar en el registro transmisor.
    URTHR = (char)c;
}

void uart_write_buffer(uint8_t* buffer, int count){
    int i;
    for (i = 0; i < count; i++)
        uart_write(buffer[i]);    
}

uint8_t uart_read(){

    int incoming=-1;
    //if(uart_available() <= 0) // si no hay datos disponibles devuelve -1
      //  return -1;
    while(uart_available()<=0)
    {
    }
                 
    incoming = _uart_in_buffer[_out_buffer_index++];
    if(_out_buffer_index == UART_BUFFER_SIZE)
          _out_buffer_index=0;
    return incoming;
        
}

int uart_read_buffer(uint8_t* buffer, int count){

    int i;
    if(uart_available() < count)
        return -1;

    for (i = 0; i < count; i++)
        buffer[i]=uart_read();

    return i;
}

int uart_available(){
    int count=_in_buffer_index - _out_buffer_index;
    
    return (count < 0) ? UART_BUFFER_SIZE - _out_buffer_index-1 : count ;

}

void uart_flush(){

    _in_buffer_index = _out_buffer_index = 0;
}

// Funcion para atender interrupciones de la UART.
// Guarda todos los datos recibidos en un buffer.
// El tamaño del buffer es de 512 bytes
void uart_interrupt_isr(){

    __asm
        push    af
        push    bc
        push    de
        push    hl
        push    iy
    __endasm;
    //eeprom_write(0x0090,(uint8_t)URRBR); //solo para comprobar interrupcion.
   //  Código de servicio de interrupción
    _uart_in_buffer[_in_buffer_index++] = URRBR;
    if(_in_buffer_index == UART_BUFFER_SIZE)
        _in_buffer_index=0;
    //*****************************************
    __asm
        pop iy
        pop hl
        pop de
        pop bc
        pop af
        ei
        ret
    __endasm;
}


void uart_print(const uint8_t* str){

    // Mientras no sea \0
    while(*str)       
       uart_write(*str++); // envía el siguiente caracter. 
}

int uart_read_line(uint8_t* str){

    int n=0;
    uint8_t c;
    while(n<MAXLINE-1 && (c=uart_read()) != '\n' && c !='\r'){
        // Si es BACKSPACE (0x7F o 0x08)
        if(c == 0x7F || c==0x08){
        
            if(n>0){
                str[--n]='\0';
                uart_write(c);
                uart_write(' ');
                uart_write(c);
            }
        }
        else
        if(isprint(c))
        {
            str[n++]=c;
            uart_write(c);
        }
    }       
       
    str[n]='\0';     
    uart_write('\n');
    return n;
}

void printBuffer()
{
   int i;
   for (i=0;i<UART_BUFFER_SIZE;i++)
        uart_write(_uart_in_buffer[i]);
}
#endif // _UART_H_
