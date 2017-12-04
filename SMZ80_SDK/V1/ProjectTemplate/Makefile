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
#	Autores:	
#	Alfredo Orozco de la Paz
#   alfredoopa@gmail.com
#	
#   Sergio Arturo Torres Ramírez
#	e-mail						
#	
#	
#	Última modificación: 05 de Octube del 2016
###############################################################################


#Nombre del Proyecto
PROYECT_NAME := Z80-Blink


# Cambiar la variable SDK_PATH con la ruta de instalación del
# SDK, por lo regular en windows es C:\SMZ80_SDK\V1 y en linux
# y Mac OS X es /opt/SMZ80_SDK/V1

ifeq ($(OS),Windows_NT)
	SDK_PATH := C:\SMZ80_SDK\V1
    uname_S := Windows
    include $(SDK_PATH)\Makefile.common
else
    SDK_PATH := ../
    uname_S := $(shell uname -s)
    include $(SDK_PATH)/Makefile.common
endif
