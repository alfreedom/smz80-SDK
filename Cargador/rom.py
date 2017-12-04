
from intelhex import IntelHex

class RomMemory:
    """docstring for RomMemory"""
    pagesize = 256
    numpages = 128
    size = pagesize * numpages
    def __init__(self):
        self.rom = IntelHex()
        self.size=0
        
    def OpenHex(self, filename):
        self.rom = IntelHex()
        try:
            self.rom.fromfile(filename, format='hex')
        except:
            return None
            
        self.size = len(self.rom.tobinarray())
        return True
        
    def OpenBin(self, filename):
        self.rom = IntelHex()
        
        try:
            self.rom = fromfile(filename, format = 'bin')
        except:
            return None
            
        self.size = len(self.rom.tobinarray())
        
        return True
    def Print(self):
        self.rom.dump()
        
    def ReadPage(self, numpage):
        return self.rom.tobinarray(start=numpage*self.pagesize, size = self.pagesize)
        
    def ReadBytes(self, address, count):
        return self.rom.tobinarray(start=address, size = count)
