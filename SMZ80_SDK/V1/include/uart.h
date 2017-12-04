
#ifndef UART_H_
#define UART_H_
#include <stdint.h>
#include <stdio.h>
#include <ctype.h> 

#ifndef PPI_BASE_ADDR
#warning "No se definio la direccion base del PPI"
#warning "Para usar las funciones con el PPI defina:      #define PPI_BASE_ADDR 0xNN, antes de incluir smz80.h"
#endif 

#ifndef UART_BASE_ADDR
#warning "No se definio la direccion base de la UART"
#warning "Para usar las funciones con la UART defina:     #define UART_BASE_ADDR 0xNN, antes de incluir smz80.h"
#endif

#define TRUE 1      ///< Definición para verdadero.
#define FALSE 0     ///< Definición para falso.
#define INPUT TRUE      ///< Definición para Entrada.
#define OUTPUT FALSE    ///< Definición para Salida.
#define LOW FALSE       ///< Definición para Bajo.
#define HIGH TRUE       ///< Definición para Alto.
#define MAXLINE 1024    /**< Numero maximo de caracteres a leer por ReadLine() */
#define ISR_NMI()     void isr_vector66()  __critical __interrupt   
#define ISR_INT_08()  void isr_vector08()  __interrupt 1    /**< Función para el vector de interrupción 0x08 */
#define ISR_INT_10()  void isr_vector10()  __interrupt 2    /**< Función para el vector de interrupción 0x10 */
#define ISR_INT_18()  void isr_vector18()  __interrupt 3    /**< Función para el vector de interrupción 0x18 */
#define ISR_INT_20()  void isr_vector20()  __interrupt 4    /**< Función para el vector de interrupción 0x20 */
#define ISR_INT_28()  void isr_vector28()  __interrupt 5    /**< Función para el vector de interrupción 0x28 */
#define ISR_INT_30()  void isr_vector30()  __interrupt 6    /**< Función para el vector de interrupción 0x30 */
#define ISR_INT_38()  void isr_vector38()  __interrupt 7    /**< Función para el vector de interrupción 0x38 */
#define HALT()  __asm__ ("HALT")    ///< Macro para ejecutar la instrucción HALT del Z80.
#define halt()  __asm__ ("HALT")    ///< Macro para ejecutar la instrucción HALT del Z80.
#define NOP()   __asm__ ("NOP")     ///< Macro para ejecutar la instrucción NOP del Z80.
#define nop()   __asm__ ("NOP")     ///< Macro para ejecutar la instrucción NOP del Z80.
#define SLEEP() __asm__ ("HALT")    ///< Macro para ejecutar la instrucción HALT del Z80.
#define sleep() __asm__ ("HALT")    ///< Macro para ejecutar la instrucción HALT del Z80.
#define BV(BIT) (1<<(BIT))          ///< Macro que obtiene la mascara de bit para la posición BIT.
#define SFR_IO __sfr __at           ///< Macro para declarar variables a puertos de E/S usados con instrucciones IN y OUT.
#define RST(VECTR) __asm\
                     RST VECTR\
                   __endasm        ///< Macro para ejecutar la instrucción RST a un vector de interrupción del Z80.

#define EI()    __asm__ ("EI")  /**< Habilita interrupción INT */
#define DI()    __asm__ ("DI")  /**< Deshabilita interrupción INT */
#define IM(MODE) __asm\
                    IM MODE\
                __endasm        /**< Cambia el modo de la interrupción INT */
                

typedef uint8_t     Byte;   ///< Definición de tipo de dato byte.
typedef uint16_t    Word;  ///< Definición de tipo de dato word.
typedef uint32_t    DWord; ///< Definición de tipo de dato double word.

#if defined (PPI_BASE_ADDR)
#define PPI_PORTA_ADDR      PPI_BASE_ADDR + 0x00      /**< Dirección del Puerto A del PPI */
#define PPI_PORTB_ADDR      PPI_BASE_ADDR + 0x01      /**< Dirección del Puerto B del PPI */
#define PPI_PORTC_ADDR      PPI_BASE_ADDR + 0x02      /**< Dirección del Puerto C del PPI */
#define PPI_CTRL_ADDR       PPI_BASE_ADDR + 0x03      /**< Dirección del Registro de Control del PPI */
SFR_IO  PPI_PORTA_ADDR    PPI_PORTA;    /**< Variable del Puerto A*/
#define PA0     0       /**< Numero de bit para el puerto PA0*/
#define PA1     1       /**< Numero de bit para el puerto PA1*/
#define PA2     2       /**< Numero de bit para el puerto PA2*/
#define PA3     3       /**< Numero de bit para el puerto PA3*/
#define PA4     4       /**< Numero de bit para el puerto PA4*/
#define PA5     5       /**< Numero de bit para el puerto PA5*/
#define PA6     6       /**< Numero de bit para el puerto PA6*/
#define PA7     7       /**< Numero de bit para el puerto PA7*/
SFR_IO  PPI_PORTB_ADDR    PPI_PORTB;    /**< Variable del Puerto B*/
#define PB0     0       /**< Numero de bit para el puerto PB0*/
#define PB1     1       /**< Numero de bit para el puerto PB1*/
#define PB2     2       /**< Numero de bit para el puerto PB2*/
#define PB3     3       /**< Numero de bit para el puerto PB3*/
#define PB4     4       /**< Numero de bit para el puerto PB4*/
#define PB5     5       /**< Numero de bit para el puerto PB5*/
#define PB6     6       /**< Numero de bit para el puerto PB6*/
#define PB7     7       /**< Numero de bit para el puerto PB7*/
SFR_IO  PPI_PORTC_ADDR    PPI_PORTC;    /**< Variable del Puerto C*/
#define PC0     0       /**< Numero de bit para el puerto PC0*/
#define PC1     1       /**< Numero de bit para el puerto PC1*/
#define PC2     2       /**< Numero de bit para el puerto PC2*/
#define PC3     3       /**< Numero de bit para el puerto PC3*/
#define PC4     4       /**< Numero de bit para el puerto PC4*/
#define PC5     5       /**< Numero de bit para el puerto PC5*/
#define PC6     6       /**< Numero de bit para el puerto PC6*/
#define PC7     7       /**< Numero de bit para el puerto PC7*/
SFR_IO  PPI_CTRL_ADDR  PPI_CTRL;   /**< Variable del Registro de Control */
#define PCPCL   0       /**< Numero de bit para el Control Word PCL bit */
#define PCPCH   3       /**< Numero de bit para el Control Word PCA bit */
#define PCPA    4       /**< Numero de bit para el PPI Control Word PA bit */
#define PCPB    1       /**< Numero de bit para el PPI Control Word PB bit */
#define PCMB    2       /**< Numero de bit para el PPI Control Word Mode Group B bit */
#define PCMA0   5       /**< Numero de bit para el PPI Control Word Mode Group A bit 0 */
#define PCMA1   6       /**< Numero de bit para el PPI Control Word Mode Group A bit 1 */
#define PCME    7       /**< Numero de bit para el PPI Control Word Mode Enable */
typedef enum{
    PPI_MODE_0  = 0x00,     /**< Modo 0 del PPI*/
    PPI_MODE_1  = 0x24,     /**< Modo 1 del PPI*/
    PPI_MODE_2  = 0x40      /**< Modo 2 del PPI*/
}ppi_mode_t;
typedef struct{
    ppi_mode_t          mode;           /**< Modo del PPI*/
    char                pa_dir;         /**< Direccion del puerto A*/
    char                pb_dir;         /**< Direccion del puerto B*/
    char                pcl_dir;        /**< Direccion del puerto C bajo*/
    char                pch_dir;        /**< Direccion del puerto C alto*/
}ppi_cfg_t;
#endif // PPI_BASE_ADDR


#if defined (UART_BASE_ADDR)

#define UART_RBR_ADDR   UART_BASE_ADDR + 0x01    /**< Dirección del Registro RX (A0=1) */
#define UART_THR_ADDR   UART_BASE_ADDR + 0x01    /**< Dirección del Registro TX (A0=1) */
#define UART_CSR_ADDR   UART_BASE_ADDR + 0x00    /**< Dirección del Registro de control A0=0 */

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

#if defined (PPI_BASE_ADDR)
void ppi_init(const ppi_cfg_t *ppi_config);
void ppi_set_portc_bit(const char bit);
void ppi_clear_portc_bit(const char bit);
#endif // PPI_BASE_ADDR


#if defined (UART_BASE_ADDR)
void uart_init(const uart_cfg_t *uart_config);
void uart_write(char c);
char uart_read();
int uart_available();
void uart_print(const char* str);
int uart_read_line(char* str);
#endif // UART_BASE_ADDR


void delay_ms(int ms) {

    int i;
    while(ms--)
        for(i=0;i<0x106;i++)
            __asm__("nop");
}



#if defined (PPI_BASE_ADDR)
void ppi_init(const ppi_cfg_t *ppi_config) {
    PPI_CTRL = 0x80 | ppi_config->mode | (ppi_config->pcl_dir << PCPCL) | (ppi_config->pch_dir << PCPCH) | (ppi_config->pa_dir << PCPA) | (ppi_config->pb_dir << PCPB);
}
void ppi_set_portc_bit(const char bit) {
    PPI_CTRL = 1 | bit << 1;
}
void ppi_clear_portc_bit(const char bit) {
    PPI_CTRL = bit << 1;
}
#endif // PPI_BASE_ADDR

#if defined (UART_BASE_ADDR)
void uart_init(const uart_cfg_t *uart_config) {
    URCONTROL = (uart_config->divisor) | (uart_config->configuracionDePalabra) | (uart_config->interrupcionesDelTransmisor)| (uart_config->interrupcionesDelReceptor);
}

void uart_write(char c) {
   /* while( !(URCONTROL & BV(UTDRE)))
        NOP(); */   
    URTHR = c;
}
char uart_read() {
   while(!(URCONTROL & BV(URDRF))) 
   NOP();
  return URRBR;
}
int uart_available(){
    return (URCONTROL & BV(URDRF));
}

void uart_print(const char* str) {
    while(*str)       
       putchar(*str++); // envía el siguiente caracter. 
}
int uart_read_line(char* str) {
    int n=0;
    char c;
    while(n<MAXLINE-1 && (c=getchar()) != '\n' && c !='\r') {
        if(c == 0x7F || c==0x08) {
            if(n>0){
                str[--n]='\0';
                putchar(c);
                putchar(' ');
                putchar(c);
            }
        }
        else
        if(isprint(c)) {
            str[n++]=c;
            putchar(c);
        }
    }       
    str[n]='\0';     
    putchar('\n');
    return n;
}

#endif // UART_BASE_ADDR

int putchar(int c) {
    #if defined (UART_BASE_ADDR)
        if(c=='\n')
            uart_write('\r');
        uart_write(c);
    #endif
    return c;
}
char getchar() {

    #if defined (UART_BASE_ADDR)
        return uart_read();
    #else
        return 0;
    #endif
}
#endif // SMZ80_H_