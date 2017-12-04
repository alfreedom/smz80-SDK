/***************************************************************************
 *  smz80.h
 *  
 *         Versión:  1.0
 *
 *           Autor:  Alfredo Orozco de la Paz
 *         Archivo:  smz80.h
 *           Fecha:  10 Enero del 2016
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
 *      Api con funciones, definiciones, estructuras de datos y macros
 *      para la programación de aplicaciones en C para el microprocesador
 *      Z80 con el compilador SDCC.
 *      
 *      Esta API cuenta con funciones relacionadas con el PPI 8255, la 
 *      UART 8250 de Intel y el mismo microprocesador Z80.
 *      
 *      Funciones que implementa la API:
 *      
 *       Para la UART:
 *       
 *              Funciones:
 *                  void uart_init(uart_cfg_t *uart_config)                         // Configura la UART
 *                  void uart_write(char c)                                         // Escribe un dato por la UART.
 *                  char uart_read();                                               // Lee un dato de la UART
 *                  char uart_print(const char *s);                                 // Imprime una cadena de caracteres por la UART 
 *                  int uart_read_line(const char *line);                           // Lee una linea de texto de la UART.
 *                  void uart_set_baudrate(uart_baudrate_t baudrate);               // Cambia el baudrate del la UART.
 *                  void uart_enable_interrupts(uart_interrupt_t interrupts);       // Habilita interrupciones de la UART.
 *                  void uart_disable_interrupts();                                 // Desabilita todas las interrupciones de la UART.
 *                 
 *                  UART_DIV_VALUE(BAUDRATE)                // Calcula el divisor de baudaje para la UART
 *                  
 *              Definiciones de variables a puertos de E/S:
 *               
 *                  URRBR       // Variable del Registro RX (DLAB=0).
 *                  URTHR       // Variable del Registro TX (DLAB=0).
 *                  URIER       // Variable del Registro Interrupt Enable.
 *                  URIIR       // Variable del Registro Interrupt Identification.
 *                  URLCR       // Variable del Registro Line Control.
 *                  URMCR       // Variable del Registro Modem Control.
 *                  URLSR       // Variable del Registro Line Status
 *                  URMSR       // Variable del Registro Modem Status
 *                  URDLL       // Variable del Registro Divisor Latch LSB (DLAB=1)
 *                  URDLM       // Variable del Registro Divisor Latch MSB (DLAB=1)
 *                  
 *                  
 *       Para el PPI:
 *       
 *              Funciones:
 *                  void ppi_init(ppi_cfg_t *ppi_config);       // Inicializa el PPI
 *                  void ppi_set_portc_bit(char bit_num);       // Pone a 1 un bit del puerto C del PPI
 *                  void ppi_clear_portc_bit(char bit_num);     // Pone a 0 un bit del puerto C del PPI
 *              
 *              Definiciones de variables a puertos de E/S:
 *               
 *                  PPI_PORTA       // Variable al puerto A del PPI.      
 *                  PPI_PORTB       // Variable al puerto B del PPI.      
 *                  PPI_PORTC       // Variable al puerto C del PPI.      
 *                  PPI_CTRL        // Variable al puerto de CONTROL del PPI.               
 *                  
 *                  
 *       Para el Z80:
 *       
 *              Funciones:
 *                  void io_write(char port_addr, char data);                               // Envia un dato a un puerto de E/S del Z80.
 *                  void io_write_buffer(char port_addr, char* buffer_out, char count);     // Envia un buffer de datos a un puerto de E/S del Z80.
 *                  char io_read(char port_adrr);                                           // Lee un dato de un puerto de E/S del Z80.
 *                  void io_read_buffer(char port_addr, char* buffer_in, char count);       // Lee un buffer de datos de un puerto de E/S del Z80.
 *                  
 *                  void delay_10us()       // Hace un delay de 10 microsegundos a 4 Mhz
 *                  void delay_100us()      // Hace un delay de 100 microsegundos a 4 Mhz
 *                  void delay_ms(int ms)   // Hace un delay de milisegundos microsegundos a 4 Mhz
 *                  
 *                  HALT()                      Ejecuta la instrucción HALT
 *                  halt()                      Ejecuta la instrucción HALT
 *                  Halt()                      Ejecuta la instrucción HALT
 *                  SLEEP()                     Ejecuta la instrucción HALT
 *                  sleep()                     Ejecuta la instrucción HALT
 *                  Sleep()                     Ejecuta la instrucción HALT
 *                  NOP()                       Ejecuta la instrucción NOP
 *                  nop()                       Ejecuta la instrucción NOP
 *                  EI()                        Habilita interrupciones: instruccion EI.
 *                  DI()                        Deshabilita interrupciones: instruccion DI
 *                  IM(MODE)                    Cambia el modo de interrupcion INT: instruccion IM
 *                  RST(VECT)                   Ejecuta la instrucción RST al vector VECT.
 *                  SFR_IO port_addr var_name   Macro para crear una variabale a un puerto de E/S (debe ser variable global)
 *                  BV(BIT)                     Macro para obtener el valor hexadecimal de un BIT en un registro.  
 *                  
 *                  
 *             
 *  
 ***************************************************************************************
 *
 *    Copyright:
 *
 *        Copyrigth© 2015
 *        Alfredo Orozco de la Paz   *FRED*
 *        e-mail:  alfredoopa@gmail.com
 *        www.vidaembebida.wordpress.com                                        
 *
 *     Licencia:
 *
 *        Este programa es software libre; usted lo puede distribuir y/o modificar
 *        bajo los terminos   de la General Public License GNU, según es publicada
 *        en la  Free  Software  Foundation;  ya se la versión 2 de la Licencia, o
 *        (a su elección) una versión posterior.
 *
 *        Este  programa se  distribuye  con la  esperanza  de que sea  útil, pero
 *        SIN   NINGUNA   GARANTÍA,  incluso   sin   la   garantía   implícita  de
 *        COMERCIALIZACIÓN  o  IDONEIDAD PARA UN PROPÓSITO PARTICULAR. consulte la
 *        Licencia Pública General de GNU para más detalles.
 *
 *        Debería haber recibido una copia de  la  Licencia Pública General de GNU
 *        junto con este programa; si no, visite http://www.gnu.org/licenses.
 *      
 ***************************************************************************************/

#ifndef SMZ80_H_
#define SMZ80_H_

#include <stdint.h>
#include <stdio.h>
#include <ctype.h> 

/************************************************************************
*                       DEFINICION DE CONSTANTES
************************************************************************/

// Direccionamiento del PPI
#ifndef PPI_BASE_ADDR
#warning "No se definio la direccion base del PPI"
#warning "Para usar las funciones con el PPI defina:      #define PPI_BASE_ADDR 0xNN, antes de incluir smz80.h"
#endif 


#define TRUE 1      ///< Definición para verdadero.
#define FALSE 0     ///< Definición para falso.

#define INPUT TRUE      ///< Definición para Entrada.
#define OUTPUT FALSE    ///< Definición para Salida.

#define LOW FALSE       ///< Definición para Bajo.
#define HIGH TRUE       ///< Definición para Alto.

#define MAXLINE 100     /**< Numero maximo de caracteres a leer por ReadLine() */
/***********************************************************************/


/************************************************************************
*          DEFINICION DE FUNCIONES A VECTORES DE INTERRUPCION
************************************************************************/
#define ISR_NMI()     void isr_vector66()  __critical __interrupt   /**< Función para el vector de interrupción NO ENMASCARABLE 0x66 */
#define ISR_INT_08()  void isr_vector08()  __interrupt 1    /**< Función para el vector de interrupción 0x08 */
#define ISR_INT_10()  void isr_vector10()  __interrupt 2    /**< Función para el vector de interrupción 0x10 */
#define ISR_INT_18()  void isr_vector18()  __interrupt 3    /**< Función para el vector de interrupción 0x18 */
#define ISR_INT_20()  void isr_vector20()  __interrupt 4    /**< Función para el vector de interrupción 0x20 */
#define ISR_INT_28()  void isr_vector28()  __interrupt 5    /**< Función para el vector de interrupción 0x28 */
#define ISR_INT_30()  void isr_vector30()  __interrupt 6    /**< Función para el vector de interrupción 0x30 */
#define ISR_INT_38()  void isr_vector38()  __interrupt 7    /**< Función para el vector de interrupción 0x38 */
/***********************************************************************/


/************************************************************************
*                       DEFINICION DE MACROS
************************************************************************/
#define HALT()  __asm__ ("HALT")    ///< Macro para ejecutar la instrucción HALT del Z80.
#define halt()  __asm__ ("HALT")    ///< Macro para ejecutar la instrucción HALT del Z80.
#define Halt()  __asm__ ("HALT")    ///< Macro para ejecutar la instrucción HALT del Z80.
#define NOP()   __asm__ ("NOP")     ///< Macro para ejecutar la instrucción NOP del Z80.
#define nop()   __asm__ ("NOP")     ///< Macro para ejecutar la instrucción NOP del Z80.
#define Nop()   __asm__ ("NOP")     ///< Macro para ejecutar la instrucción NOP del Z80.
#define SLEEP() __asm__ ("HALT")    ///< Macro para poner al Z80 en modo Sleep.
#define sleep() __asm__ ("HALT")    ///< Macro para poner al Z80 en modo Sleep.
#define Sleep() __asm__ ("HALT")    ///< Macro para poner al Z80 en modo Sleep.

#define BV(BIT) (1<<(BIT))          ///< Macro que obtiene la mascara de bits para el bit BIT.
#define SFR_IO __sfr __at           ///< Macro para declarar variables a puertos de E/S.

#define RST(VECTR) __asm\
                     RST VECTR\
                   __endasm        ///< Macro para ejecutar la instrucción RST a un vector de interrupción del Z80.

#define EI()    __asm__ ("EI")  /**< Habilita interrupción INT */
#define DI()    __asm__ ("DI")  /**< Deshabilita interrupción INT */
#define IM(MODE) __asm\
                    IM MODE\
                __endasm        /**< Cambia el modo de la interrupción INT */





#if defined (PPI_BASE_ADDR)

/************************************************************************
*                     DEFINICION DE REGISTROS DEL PPI
************************************************************************/
#define PPI_PORTA_ADDR      PPI_BASE_ADDR + 0x00      /**< Dirección del Puerto A del PPI */
#define PPI_PORTB_ADDR      PPI_BASE_ADDR + 0x01      /**< Dirección del Puerto B del PPI */
#define PPI_PORTC_ADDR      PPI_BASE_ADDR + 0x02      /**< Dirección del Puerto C del PPI */
#define PPI_CTRL_ADDR       PPI_BASE_ADDR + 0x03      /**< Dirección del Registro de Control del PPI */
/***********************************************************************/

/************************************************************************
*                   VARIABLES A LOS REGISTROS DEL PPI
************************************************************************/
// Registro del Puerto A
SFR_IO  PPI_PORTA_ADDR    PPI_PORTA;    /**< Variable del Puerto A*/
// Bits del PUERTO A
#define PA0     0       /**< Numero de bit para el puerto PA0*/
#define PA1     1       /**< Numero de bit para el puerto PA1*/
#define PA2     2       /**< Numero de bit para el puerto PA2*/
#define PA3     3       /**< Numero de bit para el puerto PA3*/
#define PA4     4       /**< Numero de bit para el puerto PA4*/
#define PA5     5       /**< Numero de bit para el puerto PA5*/
#define PA6     6       /**< Numero de bit para el puerto PA6*/
#define PA7     7       /**< Numero de bit para el puerto PA7*/

// Registro del Puerto B
SFR_IO  PPI_PORTB_ADDR    PPI_PORTB;    /**< Variable del Puerto B*/
// Bits del PUERTO B
#define PB0     0       /**< Numero de bit para el puerto PB0*/
#define PB1     1       /**< Numero de bit para el puerto PB1*/
#define PB2     2       /**< Numero de bit para el puerto PB2*/
#define PB3     3       /**< Numero de bit para el puerto PB3*/
#define PB4     4       /**< Numero de bit para el puerto PB4*/
#define PB5     5       /**< Numero de bit para el puerto PB5*/
#define PB6     6       /**< Numero de bit para el puerto PB6*/
#define PB7     7       /**< Numero de bit para el puerto PB7*/

// Registro del Puerto C
SFR_IO  PPI_PORTC_ADDR    PPI_PORTC;    /**< Variable del Puerto C*/
// Bits del PUERTO C
#define PC0     0       /**< Numero de bit para el puerto PC0*/
#define PC1     1       /**< Numero de bit para el puerto PC1*/
#define PC2     2       /**< Numero de bit para el puerto PC2*/
#define PC3     3       /**< Numero de bit para el puerto PC3*/
#define PC4     4       /**< Numero de bit para el puerto PC4*/
#define PC5     5       /**< Numero de bit para el puerto PC5*/
#define PC6     6       /**< Numero de bit para el puerto PC6*/
#define PC7     7       /**< Numero de bit para el puerto PC7*/

// Registro de control
SFR_IO  PPI_CTRL_ADDR  PPI_CTRL;   /**< Variable del Registro de Control */

#define PCPCL   0       /**< Numero de bit para el Control Word PCL bit */
#define PCPCH   3       /**< Numero de bit para el Control Word PCA bit */
#define PCPA    4       /**< Numero de bit para el PPI Control Word PA bit */
#define PCPB    1       /**< Numero de bit para el PPI Control Word PB bit */
#define PCMB    2       /**< Numero de bit para el PPI Control Word Mode Group B bit */
#define PCMA0   5       /**< Numero de bit para el PPI Control Word Mode Group A bit 0 */
#define PCMA1   6       /**< Numero de bit para el PPI Control Word Mode Group A bit 1 */
#define PCME    7       /**< Numero de bit para el PPI Control Word Mode Enable */
/***********************************************************************/


/************************************************************************
*                     ENUMERACIONES DEL PPI
************************************************************************/
/**
 * @brief Enumeración de modos para el PPI 8255
 * @details Define los modos de operación del PPI 8255.
 */
typedef enum{
    PPI_MODE_0  = 0x00,     /**< Modo 0 del PPI*/
    PPI_MODE_1  = 0x24,     /**< Modo 1 del PPI*/
    PPI_MODE_2  = 0x40      /**< Modo 2 del PPI*/
}ppi_mode_t;
/***********************************************************************/

/************************************************************************
*                 ESTRUCTURAS DE DATOS DEL PPI
************************************************************************/
/**
 * @brief Estructura para configuración del PPI 8255.
 * @details Contiene los parámetros de configuración del
 *          PPI 8255.
 */
typedef struct{

    ppi_mode_t          mode;           /**< Modo del PPI*/
    char                pa_dir;         /**< Direccion del puerto A*/
    char                pb_dir;         /**< Direccion del puerto B*/
    char                pcl_dir;        /**< Direccion del puerto C bajo*/
    char                pch_dir;        /**< Direccion del puerto C alto*/

}ppi_cfg_t;
/***********************************************************************/
#endif // defined (PPI_BASE_ADDR)


void io_write(char port_addr, char data);
char io_read(char port_addr);
void io_write_buffer(char port_addr, char* buffer_out, char count);
void io_read_buffer(char port_addr, char* buffer_in, char count);


#if defined (PPI_BASE_ADDR)
void ppi_init(const ppi_cfg_t *ppi_config);
void ppi_set_portc_bit(const char bit);
void ppi_clear_portc_bit(const char bit);
#endif

/***********************************************************************/



/************************************************************************
*                      IMPLEMENTACIÓN DE FUNCIONES                      *
************************************************************************/

/**
 * @brief    Escritura en un puerto. 
 * @details  Envía un dato a un puerto específico.
 * 
 * @param addr Dirección del puerto.
 * @param data Dato a enviar.
 */ 
void io_write(char port_addr, char data){

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

///@cond INTERNAL   // Para excluir de la documentación de doxygen
char __ret_aux;     // Variable ausxiliar para devolver el dato de io_read.
///@endcond

/**
 * @brief   Lectura de un puerto.
 * @details Lee un dato de un puerto específico.
 * 
 * @param addr Dirección del puerto a leer.
 * @return  Dalo leído por el puerto.
 */
char io_read(char port_addr){

    // Se antepone un '_' a los nombres de las
    // variables en C que se vallan a usar en
    // código ensamblador  
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

/**
 * @brief Envio de un buffer de datos por un puerto.
 * @details Envía un buffer de datos de 8 bits por un puerto
 *          específico.
 * 
 * @param port_addr              Dirección del puerto a escribir.
 * @param[in] buffer_out    Buffer de datos.
 * @param count             Número de datos a enviar.
 */
void io_write_buffer(char port_addr, char* buffer_out, char count){

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

/**
 * @brief   Lee un buffer de datos por un puerto.
 * @details Lee N datos de un puerto específico y los guarda en buffer_in.
 * 
 * @param port_addr         Dirección del puerto a leer.
 * @param[out] buffer_in    Buffer donde se guardarán los datos
 * @param count             Número de datos a leer.
 */
void io_read_buffer(char port_addr, char* buffer_in, char count){
  
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



#if defined (PPI_BASE_ADDR)

/**
 * @brief   Inicializa el PPI 8255
 * @details Inicializa el PPI 8255 con la configuración definida en ppi_config.
 *     
 * @param ppi_config Estructura de configuración del PPI.
 */
void ppi_init(const ppi_cfg_t *ppi_config){

    PPI_CTRL = 0x80 | ppi_config->mode | (ppi_config->pcl_dir << PCPCL) | (ppi_config->pch_dir << PCPCH) | (ppi_config->pa_dir << PCPA) | (ppi_config->pb_dir << PCPB);
}

/**
 * @brief   Setea un bit del Puerto C.
 * @details Pone en estado alto (1) un bit del Puerto C del PPI 8255.
 *     
 * @param bit   Numero de bit a setear.
 */
void ppi_set_portc_bit(const char bit){

    PPI_CTRL = 1 | bit << 1;
}

/**
 * @brief   Limpia un bit del Puerto C.
 * @details Pone en estado bajo (0) un bit del Puerto C del PPI 8255.
 *     
 * @param bit   Numero de bit a limpiar.
 */
void ppi_clear_portc_bit(const char bit){
  
    PPI_CTRL = bit << 1;
}
#endif // defined (PPI_BASE_ADDR)


/*******************************************************************************/
#endif // SMZ80_H_
