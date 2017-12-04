
#ifndef SMZ80_H_
#define SMZ80_H_
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
#define UART_DIV_VALUE(F_UART,BAUDRATE) ((uint32_t)(F_UART) / ((uint32_t)(BAUDRATE)*16)) 
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
#define UART_RBR_ADDR   UART_BASE_ADDR + 0x00    /**< Dirección del Registro RX (DLAB=0) */
#define UART_THR_ADDR   UART_BASE_ADDR + 0x00    /**< Dirección del Registro TX (DLAB=0) */
#define UART_IER_ADDR   UART_BASE_ADDR + 0x01    /**< Dirección del Registro Interrupt Enable (DLAB=0) */
#define UART_IIR_ADDR   UART_BASE_ADDR + 0x02    /**< Dirección del Registro Interrupt Identification */
#define UART_LCR_ADDR   UART_BASE_ADDR + 0x03    /**< Dirección del Registro Line Control */
#define UART_MCR_ADDR   UART_BASE_ADDR + 0x04    /**< Dirección del Registro Modem Control */
#define UART_LSR_ADDR   UART_BASE_ADDR + 0x05    /**< Dirección del Registro Line Status */
#define UART_MSR_ADDR   UART_BASE_ADDR + 0x06    /**< Dirección del Registro Modem Status */
#define UART_DLL_ADDR   UART_BASE_ADDR + 0x00    /**< Dirección del Registro Divisor Latch LSB (DLAB=1) */
#define UART_DLM_ADDR   UART_BASE_ADDR + 0x01    /**< Dirección del Registro Divisor Latch MSB (DLAB=1) */
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
typedef enum{
    UART_STOP_BITS_1    = 0,         /**< 1 bit de parada*/
    UART_STOP_BITS_2    = BV(USTB)   /**< 2 bits de parada*/
}uart_stopbits_t;
typedef enum{
    UART_PARITY_NONE    = 0,                                 /**< Sin paridad*/
    UART_PARITY_ODD     = BV(UPEN),                          /**< Paridad par*/
    UART_PARITY_EVEN    = BV(UPEN) | BV(UEPS),               /**< Paridad impar*/
    UART_PARITY_MARK    = BV(UPEN) | BV(USTP),               /**< Marca*/
    UART_PARITY_SPACE   = BV(UPEN) | BV(UEPS) | BV(USTP)     /**< Espacio*/
}uart_parity_t;
typedef enum{
    UART_WORD_LENGTH_5   = 0,                         /**< Dato de 5 bits*/
    UART_WORD_LENGTH_6   = BV(UWLS0),                 /**< Dato de 6 bits*/
    UART_WORD_LENGTH_7   = BV(UWLS1),                 /**< Dato de 7 bits*/
    UART_WORD_LENGTH_8   = BV(UWLS0) | BV(UWLS1),     /**< Dato de 8 bits*/
}uart_word_length_t;
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

typedef struct{
    uart_baudrate_t     baudrate;           /**< Baudaje de la UART*/
    uart_stopbits_t     stop_bits;          /**< Bits de parada*/
    uart_parity_t       parity;             /**< Paridad*/
    uart_word_length_t  word_length;        /**< Tamaño de palabra (5, 6, 7 u 8 bits)*/
    uart_interrupt_t    interrupt;         /**< Tipo de interrupciones usadas */
}uart_cfg_t;

#define UART_DEFAULT_CONFIG {   \
    UART_BAUDRATE_9600,         \
    UART_STOP_BITS_1,           \
    UART_PARITY_NONE,           \
    UART_WORD_LENGTH_8,          \
    UART_INTERRUPT_NONE }
#endif // UART_BASE_ADDR


void io_write(char port_addr, char data);
char io_read(char port_addr);
void io_write_buffer(char port_addr, char* buffer_out, char count);
void io_read_buffer(char port_addr, char* buffer_in, char count);

#if defined (UART_BASE_ADDR)
void uart_init(const uart_cfg_t *uart_config);
void uart_set_baudrate(const uart_baudrate_t baudarte);
void uart_write(char c);
char uart_read();
int uart_available();
void uart_print(const char* str);
int uart_read_line(char* str);
void uart_disable_interrupts();
void uart_enable_interrupts(uart_interrupt_t int_cfg);
#endif // UART_BASE_ADDR

#if defined (PPI_BASE_ADDR)
void ppi_init(const ppi_cfg_t *ppi_config);
void ppi_set_portc_bit(const char bit);
void ppi_clear_portc_bit(const char bit);
#endif // PPI_BASE_ADDR

void delay_10us();
void delay_100us();
void delay_ms(int ms);
void io_write(char port_addr, char data) {
    port_addr = port_addr;
    data = data;  
     __asm
        ld  ix, #2
        add ix,sp
        ld  c, (ix)
        inc ix
        ld  a,(ix)
        out (c), a        
    __endasm;
}
static char __ret_aux;     // Variable ausxiliar para devolver el dato de io_read.
char io_read(char port_addr) {
    port_addr = port_addr;
     __asm
        LD  IX, #2
        ADD IX,SP
        LD  C, (IX)
        IN  A,(C)
        LD  (___ret_aux),A 
    __endasm;
    return __ret_aux;
}
void io_write_buffer(char port_addr, char* buffer_out, char count) {
    port_addr = port_addr;
    buffer_out = buffer_out;
    count = count;
    __asm
        LD  IX, #2
        ADD IX,SP
        LD  C, (IX)
        INC IX
        LD  L,(IX)
        INC IX
        LD  H,(IX)
        INC IX
        LD  B,(IX)
        OTIR
    __endasm;
}
void io_read_buffer(char port_addr, char* buffer_in, char count) {
    port_addr = port_addr;
    buffer_in = buffer_in;
    count = count;
     __asm
        LD  IX, #2
        ADD IX,SP
        LD  C, (IX)
        INC IX
        LD  L,(IX)
        INC IX
        LD  H,(IX)
        INC IX
        LD  B,(IX)
        INIR
    __endasm;
}
#if defined (UART_BASE_ADDR)
void uart_init(const uart_cfg_t *uart_config) {
    
    uart_set_baudrate(uart_config->baudrate);
    URIER = uart_config->interrupt;
    URLCR = (uart_config->stop_bits) | (uart_config->parity) | (uart_config->word_length);
}
void uart_set_baudrate(const uart_baudrate_t baudrate) {
    URLCR |= BV(UDLAB);
    URDLL = baudrate;
    URDLM = ((uint16_t)baudrate)>>8;
    URLCR &= ~BV(UDLAB);
}
void uart_write(char c) {
    while( !(URLSR & BV(UTHRE)))
        NOP();    
    URTHR = c;
}
char uart_read() {
    while(!(URLSR & BV(UDR))) 
        NOP();
    return URRBR;
}
int uart_available(){
    return (URLSR & BV(UDR));
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
void uart_disable_interrupts() {
    URIER = 0;
}
void uart_enable_interrupts(uart_interrupt_t int_cfg) {
    URIER = int_cfg;
}
#endif // UART_BASE_ADDR
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
void delay_10us(){
    __asm
            EXX
            EX      AF,AF'
            LD      B,#0x2
    LOOP_10:
            DJNZ    LOOP_10
            EX      AF,AF'
            EXX
    __endasm;
}
void delay_100us(){
    __asm
            EXX
            EX      AF,AF'
            LD      B,#0x3A
    LOOP_100:
            DJNZ    LOOP_100
            EX      AF,AF'
            EXX
    __endasm;
}
void delay_ms(int ms) {

    int i;
    while(ms--)
        for(i=0;i<0x106;i++)
            __asm__("nop");
}
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