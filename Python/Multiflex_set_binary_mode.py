# -*- coding: utf-8 -*-
"""
Example TeraRanger MultiFlex configuration script.
For more information about how to use this script, please refer to this document:
https://www.terabee.com/wp-content/uploads/2017/09/TR-MF-Python-ReadMe.pdf
"""
import sys
import binascii
import serial 


if __name__ == "__main__":
    
    if len(sys.argv) < 2:
        print '\n \n[ERROR] Correct usage $ python multiflex_binary.py port'
        sys.exit(1)
        
    port_name = sys.argv[1]
    multiflex = serial.Serial(port_name, 115200, timeout=5, writeTimeout=5)
             
    print 'Connected to TeraRanger MultiFlex'
    multiflex.flushInput()
    multiflex.flushOutput()
    multiflex.write(bytearray([0x00, 0x11, 0x02, 0x4C]))
    
    response = multiflex.read(16)
    response = binascii.hexlify(response)
    
    if response.find("52451100d4") != -1:
        print 'ACK'
        
    if response.find("524511ff27") != -1:
        print 'NACK'
        
    multiflex.close()
    sys.exit(0)
