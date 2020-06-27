嵌入式软件工程师养成记-基本功能篇之485串口通信
## 什么是485协议
 在嵌入式设备中，经常会听到“这设备支持232、485接口"，这里面说到232、485就是一种串口协议，物理上有本质的不同（不像TCP和UDP协议的不同只是应用层上的不同），他们用于设备之间的通信，单片机设备中很常用到。其实我们编程不需要太过深入理解两种协议上的详细细节，因为在驱动层都帮我们屏蔽了物理细节，只需要在上层进行设置就行了。两种协议在编程上基本上是通用的都是一样的，因为驱动封装成一样的接口，我们调用没有区别。我们只需要大概知道协议的一些特性即可。
## 485/232 串口的区别 
RS232是一种全双工通信，不能实现多机通信，通信距离比较短，速率比较低。通常 RS-232 接口以9个引脚 （DB-9） 或是25个引脚 （DB-25） 的型态出现 。RS232最常用的连接方式是三根线：一条发送线、一条接收线及一条地线。
RS485为半双工通信方式，可以多机通信，通信距离长，理论上1km没问题，速率高，最高传输速率为10Mbps，一般采用两线制传输，即分时实现收和发。
## linux系统下485 应用编程
linux系统下面，一切皆是文件。由于我们是应用编程，这个驱动已经帮我们做了封装，我们只需要像操作（open、write）文件一样就可以实现通信。485、232的 驱动会帮我们生成一个驱动文件描述符，ubuntu系统下一般是在/dev/ttyUSB0、/dev/ttyUSB1、、、，嵌入式系统就要看驱动怎么设置的，一般是/dev/ttyS0、或者是/dev/ttyUSB0、、、。操作分为四步：
1、打开串口。
2、配置串口：对串口的波特率、数据位、停止位、校验码、等进行设置。
3、读写串口
4、关闭串口
### C语言编程
1、头文件定义

```c
#ifndef __SERIAL_H__
#define __SERIAL_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct serial_s serial_t;

/*
*打开串口，返回串口句柄serial
*@dev:串口路径
*@baud:波特率：1200\2400\4800\9600\14400\19200\38400\57600\115200
*@data:数据位：5\6\7\8
*@stop:停止位：1\2
*@parity:校验位：O:o:N:n:E:e

*/
serial_t* serial_open(const char* dev, int baud, int data, int stop, char parity);
/*
*串口读取函数，返回读取的字节数
*@serial:由函数serial_open生成的串口句柄
*@buf:用于存储读取数据的指针
*@size:想要读取的长度
*@us:读取超时时间，us<0:阻塞直至读取size个字；us=0:不等待，直接返回；us>0:当读取到size个字节或者读取us微妙后返回
*/
int serial_read(serial_t* serial, char* buf, int size, int us);
/*
*串口发送函数，返回发送的字节数
*@serial:由函数serial_open生成的串口句柄
*@buf:发送数据的指针
*@size:想要发送的长度
*/
int serial_write(serial_t* serial, const char* buf, int size);
/*
*串口释放函数，句柄serial内存未释放，需要外部free（serial）
*/
void serial_close(serial_t* serial);

#ifdef __cplusplus
}
#endif

#endif

```
2、打开串口

```c
serial->fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
```
O_RDWR：表示以可读可写的方式打开。
O_NOCTTY：表示打开的是一个终端设备，程序不会成为该端口的控制终端。如果不使用此标志，任何一个输入(eg:键盘中止信号等)都将影响进程。
O_NDELAY：表示不关心DCD信号线所处的状态（端口的另一端是否激活或者停止）。没有这个标志的话，该程序就会在DCD信号线为低电平时停止。
O_NONBLOCK：表示以非阻塞的方式打开。
3、串口设置

```c
serial_t* serial_open(const char* dev, int baud, int data, int stop, char parity) {
    int flags;
    serial_t* serial;

    if (!dev)
        return NULL;

    serial = (serial_t*)malloc(sizeof(serial_t));
    if (serial) {
        serial->fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
        if (serial->fd > 0) {
            flags  = fcntl(serial->fd, F_GETFL, 0);
            if ((flags >= 0) && (!(flags & O_NONBLOCK)))
               fcntl(serial->fd, F_SETFL, flags | O_NONBLOCK);

            /* 设置串口各项参数:波特率、校验位、停止位、数据位 */
            memset(&serial->newm, 0, sizeof(struct termios));
            memset(&serial->oldm, 0, sizeof(struct termios));
            if (tcgetattr(serial->fd, &serial->oldm) != 0) {
                goto out;
            }

            /* 使用 CLOCAL 用于忽略所有 MODEM 状态信号线，CREAD
             * 标志用于使能接收。CSIZE 为数据位掩码 */
            serial->newm.c_cflag |= CLOCAL | CREAD;
            serial->newm.c_cflag &= ~CSIZE;

            /*设置数据位数*/
            switch (data) {
            case 5:
                serial->newm.c_cflag |= CS5;
                break;
            case 6:
                serial->newm.c_cflag |= CS6;
                break;
            case 7:
                serial->newm.c_cflag |= CS7;
                break;
            case 8:
                serial->newm.c_cflag |= CS8;
                break;
            default:
                goto out;
            }

            /* 设置停止位 */
            switch (stop) {
            case 1:
                break;
            case 2:
                serial->newm.c_cflag |= CSTOPB;
                break;
            default:
                goto out;
            }
            /* 设置校验位 */
            switch (parity) {
            case 'O':
            case 'o':
                serial->newm.c_cflag |= (PARENB | PARODD);
                serial->newm.c_iflag |= INPCK;
                break;
            case 'E':
            case 'e':
                serial->newm.c_cflag |= PARENB;
                serial->newm.c_iflag |= INPCK;
                break;
            case 'N':
            case 'n':
                break;
            default:
                goto out;
            }

            /* 设置波特率 */
            switch (baud) {
            case 1200:
                cfsetispeed(&serial->newm, B1200);
                cfsetospeed(&serial->newm, B1200);
                break;
            case 2400:
                cfsetispeed(&serial->newm, B2400);
                cfsetospeed(&serial->newm, B2400);
                break;
            case 4800:
                cfsetispeed(&serial->newm, B4800);
                cfsetospeed(&serial->newm, B4800);
                break;
            case 9600:
                cfsetispeed(&serial->newm, B9600);
                cfsetospeed(&serial->newm, B9600);
                break;
            case 19200:
                cfsetispeed(&serial->newm, B19200);
                cfsetospeed(&serial->newm, B19200);
                break;
            case 38400:
                cfsetispeed(&serial->newm, B38400);
                cfsetospeed(&serial->newm, B38400);
                break;
            case 57600:
                cfsetispeed(&serial->newm, B57600);
                cfsetospeed(&serial->newm, B57600);
                break;
            case 115200:
                cfsetispeed(&serial->newm, B115200);
                cfsetospeed(&serial->newm, B115200);
                break;
            default:
                goto out;
            }


            /*
            VMIN定义了要读取的最小字节数，read()只有在读取了VMIN个字节数据或收到一个信号才会返回
            watch out，如果VMIM为0,则read函数会以非阻塞形式读取。
            */
            serial->newm.c_cc[VTIME] = 0;
            serial->newm.c_cc[VMIN] = 0;

            /* 通过 tcflush清空输入和输出缓冲区,最后通过 tcsetattr
             * 函数对将配置实际作用于串口 */
            tcflush(serial->fd, TCIOFLUSH);
            if (!tcsetattr(serial->fd, TCSANOW, &serial->newm)) {
                return serial;
            }

        out:
            close(serial->fd);
        }

        free((void*)serial);
    }

    return NULL;
}
```
4、读串口

```c
int serial_read(serial_t* serial, char* buf, int size, int us) {
    fd_set rd;
    struct timeval tv;
    ssize_t total_size = 0;
    ssize_t read_size = 0;  
    unsigned int left_us = us;
    struct timeval start_tv;
    struct timeval current_tv;
    unsigned int time_interval_us = 0;
    
    if (serial && buf && size > 0) {
        FD_ZERO(&rd);
        FD_SET(serial->fd, &rd);

        if (us < 0)  {
            while (total_size < size) {
                if (select(serial->fd + 1, &rd, NULL, NULL, NULL) > 0) {
                    read_size = read(serial->fd, buf + total_size, size - total_size);
                    total_size += read_size;
                }
            }            
        } else {
            gettimeofday(&start_tv, NULL);
            while ((total_size < size) && (time_interval_us < us)) {
                left_us = us - time_interval_us;
                tv.tv_sec = left_us / 1000000;
                tv.tv_usec = left_us % 1000000;
                if (select(serial->fd + 1, &rd, NULL, NULL, &tv) > 0) {
                    read_size = read(serial->fd, buf + total_size, size - total_size);
                    total_size += read_size;
                }

                gettimeofday(&current_tv, NULL);
                time_interval_us = (unsigned int) ((long long)((long long)current_tv.tv_sec * 1000000 + current_tv.tv_usec) -
                                                        (long long)((long long)start_tv.tv_sec * 1000000 + start_tv.tv_usec));
            }            
        }

    }
    return total_size;
}
```
细节：这里使用select进行监听，设置超时时间。其实在open的时候设置为阻塞读取也是可以的，但是会有问题，看官可以自己思考一下如果是阻塞会存在什么问题。、
5、写串口

```c
int serial_write(serial_t* serial, const char* buf, int size) {
    ssize_t ret = -1;
    int flags;

    if (serial && buf  && size > 0) {
        flags  = fcntl(serial->fd, F_GETFL, 0);
        fcntl(serial->fd, F_SETFL, flags & ~O_NONBLOCK);
        ret = write(serial->fd, buf, size);
        fcntl(serial->fd, F_SETFL, flags);
    }

    return ret > 0 ? ret : 0;
}
```
细节：前面我们设置串口为非阻塞，所以在这里进行写的时候需要重新设置为阻塞的，因为读是异步的，我们不希望阻塞，但是写是主动的我们需要立即操作，所以必须是阻塞，不然理论上会出现我们写入了，然后也返回了，但是内核它直到某个时刻再写入硬件进行发送，这样的话就不受控，所以这里需要在发送时候设置阻塞，发送完毕之后再设置为非阻塞。
6、关闭串口

```c
void serial_close(serial_t* serial) {
    if (serial) {
        tcflush(serial->fd, TCIOFLUSH);
        tcsetattr(serial->fd, TCSANOW, &serial->oldm);
        close(serial->fd);
    }
}
```
细节：恢复原来的设置
### python编程
python对于串口也有相应的库[pyserial](https://pythonhosted.org/pyserial/)，操作起来非常简单，不需要自己封装，直接导入拿来用就行了。下面举个使用的例子，参考一下就行了，基本够用，具体的话可以去官网查看详细api（也没几个api）
```python
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
```
### shell命令行操作
shell命令也是可以操作串口发送接收数据，常用来进行简单的测试。
1、设置参数
```powershell
stty -F /dev/ttyS0 speed 115200  cs8 -parenb -cstopb
```
解析：上面的命令是设置115200波特率 8数据位 1停止位 无校验位
选项parenb使终端进行奇偶校验，-parenb则是禁止校验；
选项cs5、cs6、cs7和cs8分别将字符大小设为5、6、7和8比特；
选项300、600、1200、2400、4800、9600和19200设置波特率；
cstopb和-cstopb分别设置两个或一个停止位；
2、发送

```powershell
echo "some" > /dev/ttyS0
```
3、接收

```powershell
cat /dev/ttyS0
```
## 源码
