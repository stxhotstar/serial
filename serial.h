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
