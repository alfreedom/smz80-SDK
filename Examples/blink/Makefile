###############################################################################
#	Make  file para la  compilación de  un  proyecto para el 
#	Z80 en C con el compilador SDCC.
#	
#	Es  necesario  tener instalado en el sistema el SDK del 
#	SMZ80 para poder compilar un proyecto.
#
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
PROYECT_NAME := Z80-Blink

ifeq ($(OS),Windows_NT)
    uname_S := Windows
    SDK_PATH := C:\SMZ80_SDK\V1
    include $(SDK_PATH)\Makefile.common
else
    uname_S := $(shell uname -s)
    SDK_PATH := ../../SMZ80_SDK/V1
    include $(SDK_PATH)/Makefile.common
endif




