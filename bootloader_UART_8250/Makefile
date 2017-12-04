###############################################################################
#	Make  file para la  compilación de  un  proyecto para el 
#	Z80 en C con el compilador SDCC.
#	
#	Es  necesario  el archivo sysmap.asm para la  deficicion
#	de  los  segmentos,  el  archivo  main.c  con  el codigo 
#	fuente y el programa srec_cat para generar el ejecutable.
#	
#	Programas necesarios:
#	
#	SDCC > 3.4
#	srec_cat
#
#	Opciones:
#	
#	all: 
#	   Compila y genera los archivos hexadecimal (.hex) y el
#	   archivo ejecutable (.bin).
#	   
#	compile:
#	   Sólo  compila  el  programa pero no lo enlaza, genera
#	   los archivos objeto (.rel).
#	   
#	clean:
#	   Elimina los archivos generados por la compilación.
#
###############################################################################
#
#	Autor:	Alfredo Orozco de la Paz
#	e-mail	alfredoopa@gmail.com
#	
###############################################################################


#Nombre del Proyecto
PROYECT_NAME := Z80-Bootloader-v1
##################################################################################
# Comndo para eliminar archivos, descomentar dependiendo el sistema operativo.

# Windows
#RMCMD := del -F

# Unix/Linux
RMCMD := rm -Rf

###################################################################################

# Salida de la compilación
COUTEXT := hex
BUILD_FOLDER := _build
BIN_FOLDER := bin


# Programa srec_cat para convertir el  archivo intel hexadecimal generado 
# a un archivo binario que el z80 puede ejecutar.
HEX_TO_BIN:=srec_cat
HEX_TO_BIN_OPS:= -disable-sequence-warnings

################################################################
########	COMANDOS Y OPCIONES PARA EL COMPILADOR     #########
################################################################
# Compilador = SDCC

Z80CC := sdcc

# Banderas del compilador SDCC sin enlazar archivo crt0 por default.

CCFLAGS0 := -V -c --no-std-crt0 

# Banderas del compilador  SDCC enlazando archivos objeto.

CCFLAGS1 := -V --no-std-crt0 

# Definimos la arquitectura que se compilara = -mz80 para el Z80

CPU :=-mz80 --std-c99

#directorio donde se incluyen las librerías
CCINCLUDE_PATH := ./include

#archivos .c para compilar

CCFILES := main.c 
################################################################


################################################################
########	COMANDOS Y OPCIONES PARA EL ENSAMBLADOR    #########
################################################################
# Programa Ensamblador AS = sdasz80

Z80ASM := sdasz80

# Banderas para el ensamblador.

ASMFLAGS := -go

ASMINCLUDE_PATH := ./include
################################################################


################################################################
#######  OPCIONES PARA EL ENLAZADOR (MAPA DE MEMORIA)  #########
################################################################

# Estas 2 lineas definen las  direcciones de datos y de codigo
# dependiendo de el mapa de memoria ROM y RAM del sistema.

# CODEINIT indica la dirección en la que comienza el bootloader,
# se usa la dirección 0x6810, que son los ulimos 6k de la memoria
# EEPROM, lo cual deja 26k de memoria para el programa de usuario.
# Si el bootloader pesa mas de 6k, cambiar esta dirección.

CODEINIT:=0x6810

# DATAINIT indica la dirección en  la  que comienza la memoria
# RAM, para este caso es de 32k y va desde 0x8000 a 0xFFFF.

DATAINIT:=0x8000

################################################################


all: clean $(BUILD_FOLDER) $(BIN_FOLDER) $(BUILD_FOLDER)/$(PROYECT_NAME).hex 
	@echo "\n-------> GENERANDO EJECUTABLE..."
	@mv $(BUILD_FOLDER)/$(PROYECT_NAME).hex $(BIN_FOLDER)/$(PROYECT_NAME).hex
	@$(HEX_TO_BIN) $(HEX_TO_BIN_OPS) $(BIN_FOLDER)/$(PROYECT_NAME).hex -Intel -o $(BIN_FOLDER)/$(PROYECT_NAME).bin -binary
	@echo "-------> OK!"

compile: $(PROYECT_NAME).rel

$(BUILD_FOLDER):
	@mkdir $(BUILD_FOLDER)

$(BIN_FOLDER):
	@mkdir $(BIN_FOLDER)

$(BUILD_FOLDER)/$(PROYECT_NAME).hex: $(BUILD_FOLDER)/sysmap.rel $(BUILD_FOLDER)/$(PROYECT_NAME).rel 
	@echo "\n-------> GENERANDO ARCHIVO HEXADECIMAL..."
	@$(Z80CC) $(CPU) $(CCFLAGS1) --code-loc $(CODEINIT) --data-loc $(DATAINIT) $(BUILD_FOLDER)/sysmap.rel $(BUILD_FOLDER)/$(PROYECT_NAME).rel -o $(BUILD_FOLDER)/$(PROYECT_NAME).hex
	@echo "-------> OK!"
	

$(BUILD_FOLDER)/sysmap.rel: $(ASMINCLUDE_PATH)/sysmap.asm
	@echo "\n-------> ENSAMBLANDO MAPA DEL SISTEMA..."
	@$(Z80ASM) $(ASMFLAGS) -o $(BUILD_FOLDER)/sysmap.rel $(ASMINCLUDE_PATH)/sysmap.asm 
	@echo "-------> OK!"

$(BUILD_FOLDER)/$(PROYECT_NAME).rel:	main.c
	@echo "\n-------> COMPILANDO...\n"
	@$(Z80CC) $(CPU)  -I $(CCINCLUDE_PATH) $(CCFLAGS0)  $(CCFILES) -o $(BUILD_FOLDER)/$(PROYECT_NAME).rel
	@echo "\n-------> OK!"

clean: 
	@ echo "\n-------> LIMPIANDO..."
	@ $(RMCMD) $(BUILD_FOLDER)
	@ $(RMCMD) $(BIN_FOLDER)
	@ echo "-------> OK!"


