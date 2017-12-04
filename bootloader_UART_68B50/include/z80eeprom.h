#ifndef _EEPROM_H_
#define _EEPROM_H_


#define EEPROM_SIZE 0x7FFF
#define BOOT_START_ADDR 0X6800
#define BOOT_RESET_ADDR 0X0005 

static uint8_t* eeprom_ptr; 


/**
 * @brief Escribe un byte en memoria
 * @details Escribe un  byte en memoria a partir de la direccion indicada.
 * 
 * @param address direccion en la cual se va  a escribir el  byte.
 * @param number  byte que se va a escribir.
 */ 
uint8_t eeprom_write(uint16_t address, uint8_t data);
/**
 * @brief Escribe un paquete de bytes en memoria
 * @details Escribe un paquete de bytes en memoria a partir de la direccion y longitud de 
 * indicados.
 * 
 * @param address direccion a partir de la cual se va a comenzar a escribir el paquete de bytes
 * @param data_buffer apuntador al paquete de bytes.
 * @param data_lenght numero de bytes que se van a escribir a partir de la direccion.
 */
uint8_t eeprom_write_buffer(uint16_t address, uint8_t* data_buffer, uint16_t data_length);
/**
 * @brief Lee un byte de memoria
 * @details Lee un  byte de memoria a partir de la direccion indicada.
 * 
 * @param address direccion de la cual se va  a leer el byte.
 * @param data    apuntador a la localidad donde se va a guardar el dato.
 */
void eeprom_read(uint16_t address, uint8_t* data);

/**
 * @brief Lee un paquete de bytes de memoria
 * @details Lee un paquete de bytes de memoria a partir de la direccion y longitud de 
 * indicados.
 * 
 * @param address direccion a partir de la cual se va a comenzar a leer el paquete de bytes
 * @param data_buffer apuntador al paquete de bytes.
 * @param data_lenght numero de bytes que se van a leer a partir de la direccion.
 */
void eeprom_read_buffer(uint16_t address, uint8_t* data_buffer, uint16_t data_length);
/**
 * @brief Escribe un byte
 * @details Escribe un byte en memoria, mediante una direccion de 16 bits.
 * 
 * @param alta parte alta de la direccion
 * @param baja parte baja de la direccion
 * @param dato byte que se va a escribir
 */
void write_byte(uint8_t dir_alta,uint8_t dir_baja, uint8_t data);
void (*write_byte_EEPROM_ptr)(uint8_t , uint8_t , uint8_t );
void eeprom_erase(uint16_t address, uint16_t count);


/************************************************************************
*       DEFINICION VARIABLES USADAS EN LA ESCRITURA DE MEMORIA
*    ***Las variables usadas en ensamblador deben ser globales*** 
************************************************************************/
 uint8_t  data;  // byte que se quiere escribir
 uint8_t address_hight; // parte alta de la direccion a memoria
 uint8_t address_low;   // parte baja de la direccion a memoria
/************************************************************************
* 
************************************************************************/

uint8_t eeprom_write(uint16_t address, uint8_t number){

    // Checa si es una dirección valida para escribir y prevenir la sobreescritura
    // del bootlodaer.
    uint8_t dir_low;
    uint8_t dir_hight;
    
    dir_low = address;
    dir_hight = (address >> 8);
    
    if(address > BOOT_RESET_ADDR && address < BOOT_START_ADDR){
        write_byte_EEPROM_ptr(dir_hight,dir_low,number);//apuntador a funcion en ram para escritura en ram.
       // eeprom_ptr = (uint8_t*)address;
        //*eeprom_ptr = data;
        NOP();
        return 1;
    }

    return 0;

}

void eeprom_erase(uint16_t address, uint16_t count) {
    uint16_t addr;
    
    for(addr = address; addr < (address+count); addr ++)
        eeprom_write(addr, 0xFF);
}

uint8_t eeprom_write_buffer(uint16_t address, uint8_t* data_buffer, uint16_t data_length){

    int i;
    
    for (i = 0; i < data_length; i++){
        
        if(!eeprom_write(address+i, data_buffer[i]))
            return 0;
        NOP();
    }
    delay_ms(1000);
    return 1;
}


void eeprom_read(uint16_t address, uint8_t* data){

    if(address <= EEPROM_SIZE){
       //eeprom_ptr = (uint8_t*)address;
       //*data = *eeprom_ptr;
       *data = *(uint8_t*)address;
    }
}

void eeprom_read_buffer(uint16_t address, uint8_t* data_buffer, uint16_t data_length){
    int i;
    for (i = 0; i < data_length; i++)
        eeprom_read(address+i,data_buffer+i);
}


void write_byte(uint8_t dir_alta ,uint8_t dir_baja , uint8_t dato)
{
  
     data = dato; // byte que se va a escribir
     address_hight = dir_alta; // direccion en la que se va a escribir
     address_low= dir_baja;

        __asm
                LD  A,(_address_hight)//carga dato en acumulador
                LD H,A
                LD  A,(_address_low)//carga dato en acumulador
                LD L,A
                LD  A,(_data)//carga dato en acumulador
                LD  (HL), A  //Escribe en la direccion
        __endasm;

    __asm   //si se llama directamente a la funsion usa una instruccion de acceso de la eeprom y no funciona
     call delay_1ms_RAM

    __endasm;
}


#endif // _EEPROM_H_
