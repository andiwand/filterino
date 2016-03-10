import time
import serial
import struct
import binascii

BUFFER_SIZE = 512
BAUD = 115200
TIMEOUT = 1
TIME_ERROR = 100
TYPE_MESSAGE = 0
TYPE_RECEIVED = 1
TYPE_SEND_REQUEST = 2
TYPE_SEND_RESPONSE = 3

CODES = [
         [236, 2640, 236, 288, 240, 1292, 240, 1280, 236, 296, 244, 284, 236, 1296, 232, 292, 240, 1292, 240, 284, 240, 1292, 236, 284, 248, 1288, 236, 288, 240, 1292, 236, 1284, 236, 312, 240, 1280, 240, 296, 240, 1280, 236, 304, 232, 1284, 240, 292, 240, 1284, 240, 296, 240, 1280, 236, 296, 244, 288, 232, 1296, 240, 1280, 240, 296, 236, 292, 232, 1308, 236, 1288, 232, 300, 232, 292, 240, 1292, 236, 1288, 232, 300, 232, 296, 236, 1292, 236, 1284, 236, 300, 236, 1284, 236, 300, 236, 1284, 240, 296, 236, 1284, 240, 308, 236, 1288, 228, 308, 236, 284, 240, 1292, 232, 292, 236, 1300, 232, 1288, 232, 300, 236, 1284, 236, 300, 236, 292, 232, 1296, 236, 1288, 232, 300, 236, 292, 232, 1292, 228, 10056],
         [304, 2584, 284, 244, 280, 1252, 284, 1244, 264, 272, 268, 264, 272, 1256, 284, 244, 300, 1236, 264, 260, 268, 1268, 288, 236, 292, 1244, 276, 252, 280, 1252, 280, 1248, 268, 280, 276, 1248, 280, 256, 272, 1252, 284, 256, 260, 1264, 276, 260, 276, 1252, 264, 268, 264, 1264, 272, 264, 272, 256, 260, 1276, 260, 1264, 264, 272, 264, 260, 268, 1280, 260, 1264, 264, 272, 256, 272, 260, 1272, 260, 1268, 272, 264, 264, 264, 260, 1276, 256, 1268, 264, 276, 256, 1264, 260, 280, 264, 1256, 264, 272, 264, 1264, 252, 292, 264, 1264, 260, 276, 264, 264, 256, 1284, 252, 268, 256, 1276, 260, 268, 256, 1280, 256, 1280, 252, 272, 256, 276, 252, 1292, 240, 1280, 248, 280, 256, 272, 252, 1272, 256, 10056],
         [244, 2636, 260, 260, 256, 1284, 252, 1276, 256, 268, 260, 272, 252, 1288, 248, 272, 252, 1288, 252, 272, 252, 1288, 252, 272, 252, 1284, 248, 276, 244, 1296, 240, 1284, 248, 292, 260, 1272, 248, 288, 248, 1276, 252, 280, 248, 1284, 248, 284, 248, 1284, 248, 276, 256, 1280, 240, 288, 256, 268, 252, 1292, 248, 1280, 244, 284, 252, 272, 252, 1304, 248, 1280, 244, 276, 264, 268, 260, 1284, 244, 1284, 244, 280, 244, 288, 248, 1292, 240, 1280, 244, 284, 256, 1280, 244, 284, 248, 1284, 244, 284, 248, 1288, 244, 296, 256, 1276, 236, 292, 244, 280, 256, 1292, 236, 284, 248, 1292, 248, 1280, 232, 300, 244, 1284, 244, 288, 244, 280, 244, 1296, 240, 1284, 240, 292, 252, 1280, 236, 288, 240, 10076],
         [244, 2640, 252, 264, 268, 1276, 256, 1264, 252, 280, 264, 264, 264, 1272, 260, 268, 252, 1284, 252, 276, 248, 1292, 252, 268, 248, 1292, 252, 268, 260, 1288, 248, 1268, 248, 300, 248, 1276, 252, 280, 248, 1284, 240, 288, 256, 1284, 240, 284, 244, 1292, 232, 296, 248, 1280, 244, 284, 256, 276, 252, 1292, 240, 1276, 252, 280, 256, 268, 256, 1304, 236, 1280, 248, 288, 252, 272, 256, 1292, 248, 1276, 236, 288, 264, 264, 248, 1300, 232, 1292, 236, 288, 244, 1292, 236, 288, 248, 1292, 236, 288, 252, 1284, 240, 300, 240, 1296, 236, 288, 252, 280, 244, 1296, 232, 284, 240, 1308, 228, 292, 240, 1304, 232, 1288, 236, 292, 244, 284, 244, 1300, 232, 1288, 236, 296, 240, 1296, 224, 292, 236, 10084],
         [120, 604, 120, 612, 116, 608, 120, 608, 120, 612, 476, 244, 120, 604, 484, 244, 120, 600, 128, 604, 120, 608, 480, 248, 116, 608, 480, 252, 116, 604, 484, 248, 116, 608, 116, 608, 484, 248, 476, 244, 120, 612, 116, 608, 480, 248, 480, 244, 120, 5692],
         [120, 604, 120, 604, 124, 604, 124, 604, 120, 604, 488, 240, 120, 604, 488, 236, 124, 604, 124, 604, 120, 612, 480, 240, 120, 612, 480, 248, 112, 608, 484, 244, 120, 608, 116, 604, 488, 240, 484, 240, 488, 244, 484, 244, 116, 612, 116, 612, 112, 5688],
         [128, 608, 120, 596, 128, 588, 140, 592, 132, 592, 496, 224, 140, 608, 480, 228, 136, 592, 136, 592, 136, 596, 492, 228, 132, 600, 492, 236, 124, 600, 488, 232, 496, 240, 488, 224, 140, 596, 128, 600, 124, 600, 128, 600, 488, 232, 496, 236, 128, 5680],
         [132, 588, 136, 604, 124, 592, 132, 596, 128, 596, 496, 224, 136, 596, 500, 220, 140, 592, 132, 592, 136, 592, 496, 220, 140, 604, 488, 236, 128, 592, 496, 228, 500, 236, 488, 236, 128, 604, 124, 600, 484, 236, 492, 236, 128, 600, 124, 604, 124, 5684],
         [120, 604, 120, 604, 124, 604, 120, 608, 120, 604, 488, 248, 116, 604, 484, 244, 116, 612, 120, 600, 124, 604, 488, 248, 112, 608, 484, 244, 480, 240, 488, 244, 120, 608, 116, 612, 112, 620, 112, 604, 124, 604, 120, 608, 484, 248, 476, 244, 120, 5692],
         [136, 584, 140, 596, 128, 592, 132, 592, 136, 596, 492, 232, 132, 596, 492, 228, 136, 592, 136, 592, 132, 592, 496, 236, 128, 596, 496, 232, 492, 232, 496, 232, 128, 604, 124, 600, 128, 600, 128, 604, 484, 236, 488, 240, 124, 600, 124, 600, 128, 5676]
         ]
NAMES = {0: "A ON", 1: "A OFF", 2: "B ON", 3: "B OFF", 4: "1 ON", 5: "1 OFF", 6: "2 ON", 7: "2 OFF", 8: "3 ON", 9: "3 OFF"}
TRANSLATION = {0: 6, 1: 7}

def checkTimes(a, b):
    if len(a) != len(b): return False
    return all((abs(t1 - t2) <= TIME_ERROR for t1, t2 in zip(a, b)))

def skip(s, n):
    while n > 0: n -= len(s.read(min(BUFFER_SIZE, n)))

def readFull(s, n):
    result = ""
    while len(result) < n: result += s.read(n - len(result))
    return result;

def readStruct(s, fmt):
    string = s.read(struct.calcsize(fmt))
    return struct.unpack(fmt, string)

def readStructFull(s, fmt):
    string = readFull(s, struct.calcsize(fmt))
    return struct.unpack(fmt, string)

def writePacket(s, t, payload):
    length = struct.calcsize("<B") + len(payload)
    s.write(struct.pack("<H", length))
    s.write(struct.pack("<B", t))
    s.write(payload)

def writeSendRequest(s, repeat, times):
    payload = ""
    payload += struct.pack("<B", repeat)
    for time in times: payload += struct.pack("<H", time)
    writePacket(s, TYPE_SEND_REQUEST, payload)

def handleSerial(s):
    length = readStructFull(s, "<H")[0]
    packet = s.read(length)
    if len(packet) == length: handlePacket(s, packet)
    else: skip(s, length - len(packet))

def handlePacket(s, packet):
    offset = 0
    t = struct.unpack_from("<B", packet, offset)[0]
    offset += struct.calcsize("<B")
    
    if t == TYPE_MESSAGE:
        mt = struct.unpack_from("<B", packet, offset)[0]
        offset += struct.calcsize("<B")
        code = struct.unpack_from("<B", packet, offset)[0]
        offset += struct.calcsize("<B")
        extra = packet[offset:]
        print "message type %d code %d extra %s" % (mt, code, extra)
    elif t == TYPE_RECEIVED:
        repeat = struct.unpack_from("<B", packet, offset)[0]
        offset += struct.calcsize("<B")
        
        times = []
        while offset < len(packet):
            time = struct.unpack_from("<H", packet, offset)[0]
            times.append(time)
            offset += struct.calcsize("<H")
        handleReceive(s, repeat, times)
    elif t == TYPE_SEND_REQUEST:
        pass
    elif t == TYPE_SEND_RESPONSE:
        pass
    else: print "unknown type %d payload %s" % (t, binascii.hexlify(packet))

def handleReceive(s, repeat, times):
    index = next((i for i, code in enumerate(CODES) if checkTimes(code, times)), -1)
    if index < 0:
        print "received repeat %d length %d times %s" % (repeat, len(times), str(times))
        return
    
    print NAMES[index]
    if index in TRANSLATION:
        print "translate to %d" % TRANSLATION[index]
        time.sleep(1)
        writeSendRequest(s, 20, CODES[TRANSLATION[index]])


s = serial.Serial("/dev/ttyUSB0", BAUD, timeout=TIMEOUT)

while True:
    handleSerial(s)
