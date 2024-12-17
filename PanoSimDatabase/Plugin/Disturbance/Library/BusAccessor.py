import mmap
import struct

class BusAccessor:
    def __init__(self, busId, key, format):
        self.headerSize = 0
        self.bodySize = 0
        self.maxBodyWidth = 0
        self.headerFormat = "<"
        self.bodyFormat = "<"
        for item in format.split(','):
            name, type = item.split('@', 1)
            if type == '[':
                self.maxBodyWidth = int(name)
                self.headerSize += 4
                self.headerFormat += 'i'
            else:
                if self.maxBodyWidth > 0:
                    self.bodyFormat += type
                    self.bodySize += struct.calcsize(type)
                else:
                    self.headerFormat += type
                    self.headerSize += struct.calcsize(type)
        self.totalSize = self.headerSize + self.maxBodyWidth * self.bodySize
        self.bus = mmap.mmap(fileno = -1, length=self.totalSize, tagname="panosim." + str(busId) + "." + key)
        self.bus[0:self.totalSize] = b'\0'*self.totalSize

    def readHeader(self):
        return struct.unpack_from(self.headerFormat, self.bus)
    
    def writeHeader(self, *header):
        self.bus[0:self.headerSize] = struct.pack(self.headerFormat, *header)

    def readBody(self, index):
        return struct.unpack_from(self.bodyFormat, self.bus, self.headerSize + index * self.bodySize)
    
    def writeBody(self, index, *body):
        self.bus[self.headerSize + index * self.bodySize:self.headerSize + (index + 1) * self.bodySize] = struct.pack(self.bodyFormat, *body)

    def getBus(self):
        return self.bus
    
    def getHeaderSize(self):
        return self.headerSize
    
    def getBodySize(self):
        return self.bodySize
    
class DoubleBusReader:
    def __init__(self, busId, key, format):
        self.reader0 = BusAccessor(busId, key + ".0", format)
        self.reader1 = BusAccessor(busId, key + ".1", format)

    def getReader(self, time):
        if self.reader0.readHeader()[0] == time:
            return self.reader0
        elif self.reader1.readHeader()[0] == time:
            return self.reader1
        elif self.reader0.readHeader()[0] > self.reader1.readHeader()[0]:
            return self.reader0
        else:
            return self.reader1