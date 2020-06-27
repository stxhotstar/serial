'''
hserial.py
串口操作通用api，线程安全
'''
import serial
import threading

class hserial(object):
    #default 9600\8N1
    def __init__(self, dev = True, baudrate = False, data_bit = False, parity = False, stop_bit = False):
        print('dev = ', dev)
        data_bits = {5: serial.FIVEBITS, 6: serial.SIXBITS, 7: serial.SEVENBITS, 8: serial.EIGHTBITS}
        paritys = {'N': serial.PARITY_NONE, 'n': serial.PARITY_NONE, 'O': serial.PARITY_ODD, 'o': serial.PARITY_ODD, 'E': serial.PARITY_EVEN, 'e': serial.PARITY_EVEN}
        stop_bits = {1: serial.STOPBITS_ONE, 1.5: serial.STOPBITS_ONE_POINT_FIVE, 2: serial.STOPBITS_TWO}
        ibaudrate = (baudrate if (baudrate) else 9600)
        idata_bit = (data_bits[data_bit] if (data_bit) else serial.EIGHTBITS)
        iparity = (paritys[parity] if (parity) else serial.PARITY_NONE)
        istop_bit = (stop_bit[stop_bit] if (stop_bit) else serial.STOPBITS_ONE)
        try:
            self.__serial_fd = serial.Serial(dev, ibaudrate, idata_bit, iparity, istop_bit, timeout = None)
            self.__mutex = threading.Lock()
            print(self.__serial_fd)
        except:
            print('hserial __init__ error')

    def write(self, data):
        if not data:
            return 0

        return self.__serial_fd.write(data.encode('utf-8'))

    def read(self, size, timeout):
        self.__mutex.acquire()
        self.__serial_fd.timeout = timeout
        ret = self.__serial_fd.read(size)
        self.__mutex.release()
        return ret

    def __del__(self): 
        pass

