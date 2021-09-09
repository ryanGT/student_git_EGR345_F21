import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
fft = np.fft.fft
import time
from numpy import sin, cos, pi, arange, zeros, zeros_like

import os, glob
figdir = 'figs'


import serial
import serial_utils

portname = "/dev/ttyACM0"#<--- change me, get this from Arduino IDE
assert os.path.exists(portname), "could not find port %s" % portname
ser = serial.Serial(portname, 115200, timeout=5)

debug_line = serial_utils.Read_Line(ser)
line_str = ''.join(debug_line)
print(line_str)


N = 100

v_in = np.zeros(N)
# your code here
    
v_in_echo = np.zeros(N)
v_out = np.zeros(N)
nvect = np.zeros(N)
t_ard = np.zeros(N)

ser.flushInput()
ser.flushOutput()

serial_utils.WriteByte(ser, 2)#start new test
check_byte = serial_utils.Read_Two_Bytes(ser)
print('check_byte = %s' % check_byte)

t0 = time.time()

for i in range(N):
    # your code here
    
    nl_check = serial_utils.Read_Byte(ser)#<-- the last byte that is read should be a newline or '10'
    assert nl_check == 10, "newline problem"
    

t1 = time.time()
print('Total time = %0.4g' % (t1-t0))
dt_ave = (t1-t0)/N
print('average dt = %0.4g' % (dt_ave))

time.sleep(0.1)
end_byte = serial_utils.WriteByte(ser, 3)#stop test
print("end_byte = %s" % end_byte)
ser.close()

print("ser is closed")

# your code here
