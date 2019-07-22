/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <systemlib/mavlink_log.h>
#include <uORB/uORB.h>
#include <uORB/topics/air_probe_uart.h>
#include <px4_defines.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>

static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;

__EXPORT int air_probe_uart_main(int argc, char *argv[]);
int air_probe_uart_thread_main(int argc, char *argv[]);

static int uart_init(char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud);
static void usage(const char *reason);

int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud)
	{
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            warnx("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;

    /* 设置某个选项，那么就使用"|="运算，
       如果关闭某个选项就使用"&="和"~"运算*/

    /* fill the struct for the new configuration */
    tcgetattr(fd, &uart_config); // 获取终端参数

    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;// 将NL转换成CR(回车)-NL后输出。

    /* no parity, one stop bit; 无偶校验，一个停止位 */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);// CSTOPB 使用两个停止位，PARENB 表示偶校验

     /* set baud rate; 设置波特率 */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0)
	{
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0)
	{
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }
    // 设置与终端相关的参数，TCSANOW 立即改变参数
    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0)
	{
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}


int uart_init(char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);
    /*Linux中，万物皆文件，打开串口设备和打开普通文件一样，使用的是open（）系统调用*/
    // 选项 O_NOCTTY 表示不能把本串口当成控制终端，否则用户的键盘输入信息将影响程序的执行
    if (serial_fd < 0)
	{
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    printf("Open the %s\n",serial_fd);
    return serial_fd;
}

static void usage(const char *reason)
{
    if (reason) {
        fprintf(stderr, "%s\n", reason);
    }

    fprintf(stderr, "usage: position_estimator_inav {start|stop|status} [param]\n\n");
}


int air_probe_uart_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("[YCM]missing command");
		return 1;
    }

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            warnx("[YCM]already running\n");
            return 0;
        }

        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("air_probe_uart",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_MAX - 5,
                         2000,
                         air_probe_uart_thread_main,
                         (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("[YCM]running");

        } else {
            warnx("[YCM]stopped");
        }

        return 0;
    }

    usage("unrecognized command");
    return 1;
}

int air_probe_uart_thread_main(int argc, char *argv[])
{

    char data = '0';
    char buffer[78] = "";
    int flag = 0;

    /*
     * TELEM1 : /dev/ttyS1
     * TELEM2 : /dev/ttyS2
     * GPS    : /dev/ttyS3
     * NSH    : /dev/ttyS5
     * SERIAL4: /dev/ttyS6
     * N/A    : /dev/ttyS4
     * IO DEBUG (RX only):/dev/ttyS0
     */
    char uart_read = uart_init("/dev/ttyS6");
    if(false == uart_read)
        return -1;
    if(false == set_uart_baudrate(uart_read,115200))
	{
        printf("[JXF]set_uart_baudrate is failed\n");
        return -1;
    }
    printf("[JXF]uart init is successful\n");

	thread_running = true;

	struct air_probe_uart_s probedata;//定义消息结构体
    memset(&probedata, 0, sizeof(probedata));//结构体清零
    orb_advert_t air_probe_uart_pub = orb_advertise(ORB_ID(air_probe_uart), &probedata);//公告主题
    warnx("[daemon] starting\n");

	float alpha = 0.0;
	float beta = 0.0;
	float airspeed = 0.0;

    while(true)
	{

	    read(uart_read,&data,1);

		if(data == 0x48 && flag == 0)
		{
			flag=1;
			read(uart_read,&data,1);
			if(data == 0x52 && flag == 1)
			{
				for(int i=0;i<78;++i)
				{
					read(uart_read,&data,1);
					buffer[i]=data;
					data='0';
					flag=0;
				}
			}
			/*
			char aa[4]={buffer[29],buffer[30],buffer[31],buffer[32]};
			float pf = *(float*)&aa;*/

			char aoa[4]={buffer[57],buffer[58],buffer[59],buffer[60]};
			char aos[4]={buffer[61],buffer[62],buffer[63],buffer[64]};
			char as[4]={buffer[65],buffer[66],buffer[67],buffer[68]};

			memcpy(&alpha,aoa,sizeof(float));
			memcpy(&beta,aos,sizeof(float));
			memcpy(&airspeed,as,sizeof(float));

			probedata.probe_alpha=alpha;
			probedata.probe_beta=beta;
			probedata.probe_airspeed=airspeed;
			orb_publish(ORB_ID(air_probe_uart), air_probe_uart_pub, &probedata);

		}
	}

    warnx("[YCM]exiting");
    thread_running = false;
    close(uart_read);

    fflush(stdout);

	return 0;
}
