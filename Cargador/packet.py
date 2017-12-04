
from rom import RomMemory

class PacketType:
    DATA    = 'D'
    ACK     = 'A'
    NACK    = 'N'
    ADDRESS = 'S'
    BREAK   = 'B'
    HEADER  = 'F'
    EOF     = 'Z'
    ERROR   = 'E'
    

class Packet:
    """Clase Packet. Maneja paquetes de datos"""
 
    def __init__(self, mark=':', number=0, ptype='Z', data=None, datalen=0, checksum=0):
        self.mark = mark

        self.checksum = 0x0000
        self.checksum = ord(self.mark)
        self.number = number
        self.checksum += self.number
        if(self.checksum > 255): self.checksum &= 0xFF
        self.ptype = ptype
        self.checksum += ord(self.ptype)
        if(self.checksum > 255): self.checksum &= 0xFF
        self.datalen = datalen
        self.checksum += self.datalen
        if(self.checksum > 255): self.checksum &= 0xFF
        self.data = data
        for i in range(0,self.datalen):
            self.checksum += self.data[i]
            if(self.checksum > 255): self.checksum &= 0xFF

        if(checksum != 0):
            self.checksum = checksum

    def Check(self, packet=None):
        if (packet == None):
            packet = self
        
        checksum = 0x0000
        checksum = ord(packet.mark);
        checksum += packet.number;
        if (checksum >255): checksum &= 0xFF
        checksum += ord(packet.ptype);
        if (checksum >255): checksum &= 0xFF
        checksum += packet.datalen;
        if (checksum >255): checksum &= 0xFF
        for i in range(0,packet.datalen):
            checksum += packet.data[i]
            if (checksum >255): checksum &= 0xFF
            
        if(packet.checksum == checksum):
            return True
        else:
            return False
            
    def Print(self):
        print ""
        print "Packet Mark:   '" + self.mark+"'"
        print "Packet Number:  " + str(self.number)
        if (self.ptype == PacketType.EOF): print "Packet Type:   'END OF FILE'"
        elif (self.ptype == PacketType.ACK): print "Packet Type:   'ACKNOWLEDGE'"
        elif (self.ptype == PacketType.NACK): print "Packet Type:   'NO ACKNOWLEDGE'"
        elif (self.ptype == PacketType.ADDRESS): print "Packet Type:   'ADDRESS'"
        elif (self.ptype == PacketType.BREAK): print "Packet Type:   'BREAK'"
        elif (self.ptype == PacketType.HEADER): print "Packet Type:   'FILE HEADER'"
        elif (self.ptype == PacketType.ERROR): print "Packet Type:   'ERROR'"
        elif (self.ptype == PacketType.DATA): print "Packet Type:   'DATA'"
        print "Packet DataLen: " + str(self.datalen)
        print "Packet Checksum: 0x%.2X" % self.checksum
        if(self.datalen !=0 ): print "Packet Data:  "
       
        for i in range(0, self.datalen):
            if ((i+1)%(16) == 0):
                print "%.2X " % self.data[i]
            else:
                print "%.2X " % self.data[i],

        print ""
            

        