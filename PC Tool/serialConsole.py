# *===========================================================================*
# * Name:       serialConsole.py                                              *
# * Version:    1.0                                                           *
# * Created_by: David Dudas - david.dudas@outlook.com                         *
# * Copyright:  David Dudas - david.dudas@outlook.com                         *
# *---------------------------------------------------------------------------*
# * Content:                                                                  *
# *---------------------------------------------------------------------------*
# * Language: Python                                                          *
# * Compiler:                                                                 *
# * Target:   python 2.7                                                      *
# *===========================================================================*

import time
import serial

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

ser.close()
ser.open()
ser.isOpen()

print('Enter your commands below.\r\nInsert "exit" to leave the application.')

inp=1
while 1 :
    # get keyboard input
    inp = raw_input(">> ")
        # Python 3 users
        # input = input(">> ")
    if inp == 'exit':
        ser.close()
        exit()
    elif inp == 'a':
        print("send OK\n")
        # ser.write("%b"'\xa1\xf1\x92\x90\x80')
        # ser.write(b"%c%c%c%c%c" % (0xA1, 0xF1, 0x92, 0x90, 0x80))
        ser.write([0xA1, 0xF1, 0x00, 0xFF, 0x01])

    else:
        # send the character to the device
        # (note that I happend a \r\n carriage return and line feed to the characters - this is requested by my device)
        ser.write(b"%s\r\n" % inp.encode('ascii','ignore'))
        out = b''
        # let's wait one second before reading output (let's give device time to answer)
        time.sleep(1)
        while ser.inWaiting() > 0:
            out += ser.read(1)

        if out != '':
            print(">> %s" % out)