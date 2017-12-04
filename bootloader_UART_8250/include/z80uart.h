#ifndef _UART_H_
#define _UART_H_

#include "z80utils.h"
//#include "z80eeprom.h"

#define UART_BASE_ADDR 0x70

#define MAXLINE 100     /**< Numero maximo de caracteres a leer por ReadLine() */

/************************************************************************
*                       DEFINICION DE MACROS
************************************************************************/
#define UART_DIV_VALUE(F_UART,BAUDRATE) ((uint32_t)(F_UART) / ((uint32_t)(BAUDRATE)*16)) /**< Macro para calcular el valor de los registros DLL y DLM de la UART*/
/***********************************************************************/

/************************************************************************
*                   DEFINICION DE REGISTROS DE LA UART
************************************************************************/
#define UART_RBR_ADDR   UART_BASE_ADDR + 0x00    /**< Dirección del Registro RX (DLAB=0) */
#define UART_THR_ADDR   UART_BASE_ADDR + 0x00    /**< Dirección del Registro TX (DLAB=0) */
#define UART_IER_ADDR   UART_BASE_ADDR + 0x01    /**< Dirección del Registro Interrupt Enable (DLAB=0) */
#define UART_IIR_ADDR   UART_BASE_ADDR + 0x02    /**< Dirección del Registro Interrupt Identification */
#define UART_FCR_ADDR   UART_BASE_ADDR + 0x02    /**< Dirección del Registro Interrupt Identification */
#define UART_LCR_ADDR   UART_BASE_ADDR + 0x03    /**< Dirección del Registro Line Control */
#define UART_MCR_ADDR   UART_BASE_ADDR + 0x04    /**< Dirección del Registro Modem Control */
#define UART_LSR_ADDR   UART_BASE_ADDR + 0x05    /**< Dirección del Registro Line Status */
#define UART_MSR_ADDR   UART_BASE_ADDR + 0x06    /**< Dirección del Registro Modem Status */
#define UART_DLL_ADDR   UART_BASE_ADDR + 0x00    /**< Dirección del Registro Divisor Latch LSB (DLAB=1) */
#define UART_DLM_ADDR   UART_BASE_ADDR + 0x01    /**< Dirección del Registro Divisor Latch MSB (DLAB=1) */
/***********************************************************************/

/************************************************************************
*                 VARIABLES A LOS REGISTROS DE LA UART
************************************************************************/
SFR_IO  UART_RBR_ADDR  URRBR;   /**< Registro RBR - Receiver Buffer Register */
SFR_IO  UART_THR_ADDR  URTHR;   /**< Registro THR - Transmit Hold Register */

SFR_IO  UART_IER_ADDR  URIER;   /**< Registro IER - Interrupt Enable Register */
#define UERBFI   0              /**< Número de bit para el campo Enabled Received Data Availabre Interrupt */
#define UETBEI   1              /**< Número de bit para el campo Transmitter Holding Register Empty Interrupt */
#define UELSI    2              /**< Número de bit para el campo Error Receiver Line Status Interrupt */
#define UEDSSI   3              /**< Número de bit para el campo Enable MODEM Status Interrupt */

SFR_IO  UART_IIR_ADDR  URIIR;   /**< Registro IIR - Interrupt Identification Register */
#define UIP      0              /**< Número de bit para el campo Interrupt Pending */
#define UIID0    1              /**< Número de bit para el campo Interrupt ID bit 0 */
#define UIID1    2              /**< Número de bit para el campo Interrupt ID bit 1 */

SFR_IO UART_FCR_ADDR  URFCR;    /**< Registro FCR - FIFO Control Register */
#define UFIE      0
#define URFR     1
#define UXFR     2
#define UDMS     3
#define URTL     6
#define URTM     7

SFR_IO  UART_LCR_ADDR  URLCR;   /**< Registro LCR - Line Control Register */
#define UWLS0    0              /**< Número de bit para el campo Word Length Select Bit 0 */
#define UWLS1    1              /**< Número de bit para el campo Word Length Select Bit 1 */
#define USTB     2              /**< Número de bit para el campo Number of Stop Bits */
#define UPEN     3              /**< Número de bit para el campo Parity Enable */
#define UEPS     4              /**< Número de bit para el campo Even Parity Select */
#define USTP     5              /**< Número de bit para el campo Stick Parity */
#define USBK     6              /**< Número de bit para el campo Set Break */
#define UDLAB    7              /**< Número de bit para el campo Divisor Latch Acces Bit */

SFR_IO  UART_LSR_ADDR  URLSR;   /**< Registro LSR - Line Status Register */
#define UDR      0              /**< Número de bit para el campo Data Ready */
#define UOR      1              /**< Número de bit para el campo Overrun Error */
#define UPE      2              /**< Número de bit para el campo Parity Error */
#define UFE      3              /**< Número de bit para el campo Framming Error */
#define UBI      4              /**< Número de bit para el campo Break Interrupt */
#define UTHRE    5              /**< Número de bit para el campo Transmitter Holding Register Empty */
#define UTSRE    6              /**< Número de bit para el campo Transmitter Register Empty */

SFR_IO  UART_MCR_ADDR  URMCR;   /**< Registro MCR - Modem Control Register */
#define UDTR     0              /**< Número de bit para el campo Data Terminal Ready */
#define URTS     1              /**< Número de bit para el campo Request to Send */
#define UOUT1    2              /**< Número de bit para el campo Out 1 */
#define UOUT2    3              /**< Número de bit para el campo Out 2 */
#define ULOOP    4              /**< Número de bit para el campo Loop */

SFR_IO  UART_MSR_ADDR  URMSR;   /**< Registro MSR - Modem Status Register */
SFR_IO  UART_DLL_ADDR  URDLL;   /**< Registro DLL - Divisor Latch LSB */
SFR_IO  UART_DLM_ADDR  URDLM;   /**< Registro DLM - Divisor Latch MSB */
/***********************************************************************/

/************************************************************************
*                     ENUMERACIONES DE LA UART
************************************************************************/
/**
 *  @brief Enumeración de valores para el divisor de baudaje alto y bajo para una frecuencia de 4 Mhz. 
 *         Nota: Cambiar estos valores para una frecuencia de reloj diferente a 4 Mhz
 */
typedef enum{
    UART_BAUDRATE_1200    =  0x00D0, /**<Valor para un baudaje de 1200*/
    UART_BAUDRATE_2400    =  0x0068, /**<Valor para un baudaje de 2400*/
    UART_BAUDRATE_4800    =  0x0034, /**<Valor para un baudaje de 4800*/
    UART_BAUDRATE_9600    =  0x001A, /**<Valor para un baudaje de 9600*/
    UART_BAUDRATE_19200   =  0x000D, /**<Valor para un baudaje de 19200*/
    UART_BAUDRATE_38400_1 =  0x0006, /**<Valor para un baudaje de 38400*/
    UART_BAUDRATE_38400_2 =  0x0007, /**<Valor para un baudaje de 38400*/
    UART_BAUDRATE_38400_3 =  0x0005  /**<Valor para un baudaje de 38400*/
}uart_baudrate_t;

/**
 * @brief Enumeración de valores para en numero de bits de parada.
 * @details Definiciones para 1 o 2 bits de parada en la comunicación
 *          de la UART.
 */
typedef enum{
    UART_STOP_BITS_1    = 0,         /**< 1 bit de parada*/
    UART_STOP_BITS_2    = BV(USTB)   /**< 2 bits de parada*/
}uart_stopbits_t;

/**
 * @brief   Enumeración de valores para la paridad.
 * @details Define los tipos de paridad a usar en la comunicación de
 *          la UART.
 */
typedef enum{
    UART_PARITY_NONE    = 0,                                 /**< Sin paridad*/
    UART_PARITY_ODD     = BV(UPEN),                          /**< Paridad par*/
    UART_PARITY_EVEN    = BV(UPEN) | BV(UEPS),               /**< Paridad impar*/
    UART_PARITY_MARK    = BV(UPEN) | BV(USTP),               /**< Marca*/
    UART_PARITY_SPACE   = BV(UPEN) | BV(UEPS) | BV(USTP)     /**< Espacio*/
}uart_parity_t;

/**
 * @brief Enumeración de valores para el tamaño de dato de la UART.
 * @details Define los tamaños de dato a ser usados por la UART.
 */
typedef enum{
    UART_WORD_LENGTH_5   = 0,                         /**< Dato de 5 bits*/
    UART_WORD_LENGTH_6   = BV(UWLS0),                 /**< Dato de 6 bits*/
    UART_WORD_LENGTH_7   = BV(UWLS1),                 /**< Dato de 7 bits*/
    UART_WORD_LENGTH_8   = BV(UWLS0) | BV(UWLS1),     /**< Dato de 8 bits*/
}uart_word_length_t;

/**
 * @brief Enumeración de valores para los tipos de interrupción.
 * @details Define los valores para habilitar las interrupciones de la UART.
 */
typedef enum{
    UART_INTERRUPT_NONE  = 0,                            /**< No habilita interrupciones de la UART.*/
    UART_INTERRUPT_TX    = BV(UETBEI),                   /**< Habilita la interrupción por transmisor vacío.*/
    UART_INTERRUPT_RX    = BV(UERBFI),                   /**< Habilita la interrupción por dato disponible.*/
    UART_INTERRUPT_ERROR = BV(UELSI),                    /**< Habilita la interrupción por error de recepción.*/
    UART_INTERRUPT_MSR   = BV(UEDSSI),                   /**< Habilita la interrupción por cambio en el registro MSR.*/
    UART_INTERRUPT_RXTX  = BV(UETBEI)|BV(UERBFI),        /**< Habilita la interrupciones de dato disponible y transmisor vacío.*/
    UART_INTERRUPT_RX_ERROR  = BV(UERBFI)|BV(UELSI),     /**< Habilita la interrupciones de dato disponible y error de recepción.*/
    UART_INTERRUPT_TX_ERROR  = BV(UETBEI)|BV(UELSI),     /**< Habilita la interrupciones de transmisor vacío y error de recepción.*/
    UART_INTERRUPT_RXTX_ERROR  = BV(UETBEI)|BV(UERBFI)|BV(UELSI),   /**< Habilita la interrupciones de dato disponible, transmisor vacío y transmisor vacío.*/
    UART_INTERRUPT_ALL  = BV(UELSI)                    /**< Habilita todas las interrupciones de la UART.*/
}uart_interrupt_t;
/************************************************************************/

/************************************************************************
*                 ESTRUCTURAS DE DATOS DE LA UART
************************************************************************/
/**
 * @brief Estructura para configuración de la UART 8250.
 * @details Contiene los parámetros de configuración de la
 *          UART 8250.
 */
typedef struct{
    uart_baudrate_t     baudrate;       /**< Baudaje de la UART*/
    uart_stopbits_t     stop_bits;      /**< Bits de parada*/
    uart_parity_t       parity;         /**< Paridad*/
    uart_word_length_t  word_length;    /**< Tamaño de palabra (5, 6, 7 u 8 bits)*/
    uart_interrupt_t    interrupt;      /**< Tipo de interrupciones usadas */
}uart_cfg_t;

/**
 * @brief Configuración por default de la UART 8250.
 * @details Configuración por default de la UART 8250, la configura
 *          8N1 a 19200 sin paridad.
 */
#define UART_DEFAULT_CONFIG {   \
    UART_BAUDRATE_19200,        \
    UART_STOP_BITS_1,           \
    UART_PARITY_NONE,           \
    UART_WORD_LENGTH_8,         \
    UART_INTERRUPT_RX }
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
 * @brief Establece el baudrate de la UART
 * @details Establece el baudrate de la UART para la sincronizacion con otros dispositivos.
 * 
 * @param baudarte enumeracion donde se establece el baudrate.
 */ 
void uart_set_baudrate(const uart_baudrate_t baudarte);

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
 * @brief    Deshabilita interrupcion
 * @details  Deshabilita las interupciones por software de la UART
 * 
 */
void uart_disable_interrupts();
/**
 * @brief    habilita interrupcion
 * @details  habilita las interupciones por software de la UART
 * 
 * @param int_cfg Configuracion de registro de interrupciones de la UART.
 */
void uart_enable_interrupts(uart_interrupt_t int_cfg);
/**
 * @brief    Limpia buffer
 * 
 */
void uart_flush();
void printBuffer();


void uart_init(const uart_cfg_t *uart_config){

    // Programa el Baudrate en la UART
    uart_set_baudrate(uart_config->baudrate);
    // Programa las interrupciones
    URIER = uart_config->interrupt;
    _is_interrupt_enable = uart_config->interrupt;
    // Programa los parámetros de la UART (tamaño de dato, paridad, bits de parada)
    URLCR = (uart_config->stop_bits) | (uart_config->parity) | (uart_config->word_length);
    _in_buffer_index = _out_buffer_index = 0;
}

void uart_set_baudrate(const uart_baudrate_t baudrate){
    // DLAB = 1
    URLCR |= BV(UDLAB);
    // Divisor de baudaje bajo
    URDLL = baudrate;
    // Divisor de baudaje alto
    URDLM = ((uint16_t)baudrate)>>8;
    // DLAB = 0
    URLCR &= ~BV(UDLAB);
}

void uart_write(uint8_t c){
    // Mientras el registro transmisor no esté vacío...
    while( !(URLSR & BV(UTHRE)))
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

/*uint8_t uart_read(){

    int incoming=-1;
    int time_spend=0;
  
    // Si está habilitada la interrupción de dato recibido, leemos del buffer...
    if(_is_interrupt_enable & UART_INTERRUPT_RX)
    {
        if(!uart_available())
            return -1;

        // Devuelve el siguiente dato del buffer
        incoming = _uart_in_buffer[_out_buffer_index++];

        if(_out_buffer_index == UART_BUFFER_SIZE)
            _out_buffer_index=0;
    }
    else{ // si no está habilitada la interrupción lee por poleo.
        // Mientras no se reciba un dato, ejecuta nop's
        while(!(URLSR & BV(UDR))) 
            NOP();

        incoming == URRBR;
    }
    return incoming;
}
*/
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


void uart_disable_interrupts(){
    URIER = 0;
    _is_interrupt_enable = 0;
}

void uart_enable_interrupts(uart_interrupt_t int_cfg){
    URIER = int_cfg;
    _is_interrupt_enable = int_cfg;
}

void printBuffer()
{
   int i;
   for (i=0;i<UART_BUFFER_SIZE;i++)
        uart_write(_uart_in_buffer[i]);
}
#endif // _UART_H_
