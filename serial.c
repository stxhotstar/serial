#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/time.h>

#include "serial.h"

struct serial_s {
    int fd;
    struct termios newm;
    struct termios oldm;
};

/*串口打開函數*/
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
                watch out ,如果VMIM为0,则read函数会以非阻塞形式读取。
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

/* 串口接收函数 */
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

/* 串口发送函数 */
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

/*串口关闭函数*/
void serial_close(serial_t* serial) {
    if (serial) {
        tcflush(serial->fd, TCIOFLUSH);
        tcsetattr(serial->fd, TCSANOW, &serial->oldm);
        close(serial->fd);
    }
}
