
from rom import RomMemory
from packet import *
import serial


def empaqueta(rom):

    pk_list = []

    # Empaqueta la direccion del vector de interrupcion INT y sus datos
    pk_addr = Packet(ptype=PacketType.ADDRESS, number=0, data=[0x00, 0x38], datalen=2)
    pk_data = Packet(ptype=PacketType.DATA, number=1, data=rom.ReadBytes(0x0039, 2), datalen=2)

    # Agrega los paquetes a la lista
    pk_list.append(pk_addr)
    pk_list.append(pk_data)

    # Empaqueta la direccion del vector de interrupcion NMI y sus datos
    pk_addr = Packet(ptype=PacketType.ADDRESS, number=2, data=[0x00, 0x66], datalen=2)
    pk_data = Packet(ptype=PacketType.DATA, number=3, data=rom.ReadBytes(0x0067, 2), datalen=2)

    # Agrega los paquetes a la lista
    pk_list.append(pk_addr)
    pk_list.append(pk_data)

    app_addr = 0x0080 & 0xFFFF
    n = 4
    need_addr = True

    while((app_addr + 256) < 0x6800):
        pk_addr = Packet(ptype=PacketType.ADDRESS, number=n, data=[(app_addr >> 8) & 0xFF, app_addr & 0xFF], datalen=2)
        pk_data = Packet(ptype=PacketType.DATA, number=n, data=rom.ReadBytes(app_addr, 255), datalen=255)
        if(not isEmptyPacket(pk_data)):
            if(need_addr):
                pk_list.append(pk_addr)
                n += 1
                pk_data = Packet(ptype=PacketType.DATA, number=n, data=rom.ReadBytes(app_addr, 255), datalen=255)
                pk_list.append(pk_data)
            else:
                pk_list.append(pk_data)
            n += 1
            need_addr = False
        else:
            need_addr = True

        app_addr += 255

    pk_list.append(Packet(ptype=PacketType.EOF, number=n))
    return pk_list


def isEmptyPacket(packet):
    if(packet.datalen == 0):
        return True

    for i in range(0, packet.datalen):
        if(packet.data[i] != 0xFF):
            return False

    return True


def writePacket(port, packet):

    port.write(packet.mark)
    port.write(chr(packet.datalen))
    port.write(chr(packet.number))
    port.write(packet.ptype)
    for i in range(0, packet.datalen):
        port.write(chr(packet.data[i]))

    port.write(chr(packet.checksum))

def readPacket(port):

    mark = port.read(1)
    datalen = ord(port.read(1))
    number = ord(port.read(1))
    ptype = port.read(1)
    data = []
    for i in range(0, datalen):
        data.append(ord(port.read(1)))
    checksum = ord(port.read(1))
    pn = Packet(mark=mark, ptype=ptype, datalen=datalen, number=number, data=data, checksum=checksum)
    return pn



#rom = RomMemory()
#rom.OpenHex('app3.hex')
#paquetes = empaqueta(rom)

#port = serial.Serial("COM5",9600, timeout=1)

#for p in range(0, len(paquetes)):
#    writePacket(port, paquetes[p])
#    pn = readPacket(port)
#    if (pn.Check()): pn.Print()
#port.close()
