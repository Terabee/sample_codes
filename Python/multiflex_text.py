# -*- coding: utf-8 -*-
"""
Example TeraRanger MultiFlex configuration script
"""
import sys
import serial 
import binascii

if __name__ == "__main__":
    
    if len(sys.argv) < 2:
        print '\n[ERROR] Correct usage $ python multiflex_text.py port'
        sys.exit(1)
        
    port_name = sys.argv[1]
    multiflex = serial.Serial(port_name, 115200, timeout=10, writeTimeout=5)
             
    print 'Connected to TeraRanger MultiFlex'
    multiflex.flushInput()
    multiflex.flushOutput()
    multiflex.write(bytearray([0x00, 0x11, 0x01, 0x45]))
    
    response = multiflex.read(16)
    response = binascii.hexlify(response)
    
    if response.find("52451100d4") != -1:
        print 'ACK'
        
    if response.find("5245110027") != -1:
        print 'NACK'
        
    multiflex.close()
    sys.exit(0)
