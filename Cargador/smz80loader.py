#! /usr/bin/python
# -*- coding: utf-8 -*-

import getopt
import sys
from serial import *
from rom import RomMemory
from packet import *
from empaqueta import *
import time
#from archivo import dameArchivo

SMZ80LOADER_VERSION = "1.0"
SMZ80LOADER_DATE =  "05-10-2016"

APP_MAX_SIZE = 0x67FF - 0x0080

def main():
    serial_port_name, hexfile, verbose = check_options(sys.argv)

    if(serial_port_name == None):
        sys.exit()
    
    lista_packetes = []
    
        
    if(verbose):
        print "Opening serial port..."
        
    try:
        port = serial.Serial(serial_port_name,9600, timeout=2)
    except serialutil.SerialException:
        print "Error: Can't open serial port "+ serial_port_name
        sys.exit()
        
    if(verbose):
        print "Serial port opened!\n"
        print "Opening hex file..."
    
    myrom = RomMemory()
    if (not myrom.OpenHex(hexfile)):
        print "Error: No souch file " + hexfile
        sys.exit()
    
    print "File loaded!\n"
    
    if(verbose):
        print "Creating data packets..."
        
    lista_packetes = empaqueta(myrom)
    num_packets = len(lista_packetes)
    program_size=0
    
    if(verbose):
        print "Data packets created!"
        print "Number of packets: " + str(num_packets) +"\n"
    
    for i in lista_packetes:
        if(i.ptype == PacketType.DATA):
            program_size += i.datalen

    
    print "Available eeprom: %d bytes (%.2f Kb)" % (APP_MAX_SIZE, (APP_MAX_SIZE*1.0/1024.0))
    print "Program Size: %d bytes (%.2f Kb)" %(program_size, program_size*1.0/1024.0)
    print "Used eeprom: %.2f%%" %(program_size*100.0/(APP_MAX_SIZE)) +"\n"
        
    if(program_size > APP_MAX_SIZE):
        print "Error, the program is very large for the SM-Z80, the limit is %d bytes (%.2f Kb)" %(APP_MAX_SIZE, APP_MAX_SIZE/1024.0)
        sys.exit()
            
    #lineas = dameArchivo(hexfile)
    if(verbose):
        print "Waitting SM-Z80 sync..."
        
    timeout=12
    for i in range(0, timeout):
        timeout -= 1
        #time.sleep(1)
        try:
            if(ord(port.read(1))== 0x01):
               break
        except:
            print "No sync received."
        
    if(timeout == 0):
        print("Error, timeout detecting the SM-Z80.")
        port.close();
        sys.exit()
        
    if(verbose):
        print "Sending programming command..."
        
    port.write('@')
    
    if(verbose):
        print "Waiting response..."
        
    timeout=3
    for i in range(0, timeout):
        timeout-=1
        try:
            if(port.read(2)== "OK"):
                break
        except:
            print "Not response received",
            
        time.sleep(1)
        
    if(timeout == 0):
        print("Error, the SM-Z80 not response, check the connections and try again.")
        sys.exit()
        
        
    if(verbose):
        print "Response recieved!\n"
    
    print "Programming the SM-Z80 on port \"" + serial_port_name + "\"..."
        
    data_writed=0
    bytes_writed=0
    for p in range(0, len(lista_packetes)):
        
        if(verbose):
            print "Sending data packet %d of %d..." %(p+1, num_packets)
            if(lista_packetes[p].ptype == PacketType.DATA):
                print "Writting data..."
            elif(lista_packetes[p].ptype == PacketType.ADDRESS):
                print "Setting Address..."
            elif(lista_packetes[p].ptype == PacketType.EOF):
                print "Finishing programming..."
            
        try:
            writePacket(port, lista_packetes[p])
            pn = readPacket(port)
        except:
            print "Communication error"
            sys.exit()
        #pn = Packet(ptype = PacketType.ACK)
        while(pn.Check() == False or pn.ptype != PacketType.ACK):
            
            try:
                writePacket(port, lista_packetes[p])
                pn = readPacket(port)
            except:
                print "Communication error"
                sys.exit()
                
            if(verbose):
                print "Error Writing packet %d of %d!" %(p+1, num_packets)
                print "Sending packet again...\n"
        
        bytes_writed += lista_packetes[p].datalen + 5
        if(verbose):
            print "Packet %d Writed successfully!" % (p+1)
            if(lista_packetes[p].ptype == PacketType.DATA):
                data_writed += lista_packetes[p].datalen
                print "Writed %d bytes of %d (%d%%)" %(data_writed, program_size, data_writed*100/program_size) + "\n"
            else:
                print ""
    start = time.time()
    print "Grabando programa en EEPROM"    
    timeout=60*10
    for i in range(0, timeout):
        timeout-=1
        try:
            if(readPacket(port).ptype == PacketType.EOF):
                break
        except:
            timeout=timeout
        print ".",
        time.sleep(1)
        
    if(timeout == 0):
        print("Error al grabar el programa en EEPROM, intente otra vez.")
        sys.exit()
        
    end= time.time()

    print "\nWrote %d bytes in %.2f seconds" % (bytes_writed, (end-start))
    print "Well done!, Program loaded successfully :)"
        
    port.close()

def check_options(args):
    try:
        opts, _args = getopt.getopt(args[1:], "hvp:f:", ["help", "verbose", "version"])
    except getopt.GetoptError as err:
        # print help information and exit:
        print str(err)  # will print something like "option -a not recognized"
        usage()
        return (None, None, None)

    verbose = False
    hexfile = None
    serial_port_name=None

    for o, a in opts:
        if o in("-v","--version"):
            print "SMZ80 Loader version "+SMZ80LOADER_VERSION +"\nDate: "+ SMZ80LOADER_DATE
            return(None, None, None)
        elif o in("--verbose"):
            verbose = True
        elif o in ("-h", "--help"):
            usage(True)
            return (None, None, None)
        elif o in ("-f"):
            hexfile = a
        elif o in ("-p"):
            serial_port_name = a
        else:
            assert False, "unhandled option"

    if (serial_port_name == None):
        print "serial port not selected, use the -p option"
        usage()
        return (None, None, None)


    if(hexfile == None):
        print "hex file not selected, use the -f option"
        usage()
        return (None, None, None)
        
    return (serial_port_name, hexfile, verbose)

    
def usage(show_info=False):
    
    if(show_info):
        print " "
        print "smz80loader (Loader for the SMZ80 development kit)"
        print "Version: " + SMZ80LOADER_VERSION
        print "Created on: " + SMZ80LOADER_DATE
        print "Universidad Autónoma de San Luis Potosí, México"
        print "Facultad de Ingeniería, Área de Computación e Informática"
    print ""
    print "Usage: smz80loader.py -p <SerialPort> -f <HexFile> [-v, --version] [--verbose] [-h, --help] "
    print ""
    if(show_info):
        print "       -p: The Serial Port Name. This name depends of the"
        print "           operating system, such COMx (Windows) or /devt/tty.ACMx," 
        print "           /dev/tty.USBx, /dev.tty.serialusb, etc. (Linux/Mac)."
        print ""
        print "       -f: Path to the hexfile. This is the program in hex format"
        print "           to be uploaded into the smz80 EEPROM memory."
        print ""
        print "       --verbose: Show the verbose."
        print ""
        print "       --version,"
        print "              -v: Show the loader version. "
        print ""
        print "       --help,"
        print "           -h: Show this help. "
        print ""
        print "Copyright (C) 2016, UASLP, FI, INFOCOMP"
        print ""
        print "Writted by:"
        print "Alfredo Orozco de la Paz   alfredoopa@gmail.com"
        print "Sergio Arturo Torres Ramirez  spartan.blue@hotmail.com"
        print ""

if __name__ == "__main__":
    main()
    
    