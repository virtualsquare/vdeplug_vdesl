/*
 * libvdeplug - A library to connect to a VDE Switch.
 * Copyright (C) 2020 Renzo Davoli, University of Bologna
 *
 * Stream vde to a serial line
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation version 2.1 of the License, or (at
 * your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301, USA
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <libvdeplug.h>
#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <termios.h>
#include <poll.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/uio.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <libvdeplug_mod.h>

// #define SIMULATE_NOISY_LINE

#define VDESL_DEFAULT_SPEED B38400

static VDECONN *vde_sl_open(char *given_vde_url, char *descr,int interface_version,
		struct vde_open_args *open_args);
static ssize_t vde_sl_recv(VDECONN *conn,void *buf,size_t len,int flags);
static ssize_t vde_sl_send(VDECONN *conn,const void *buf,size_t len,int flags);
static int vde_sl_datafd(VDECONN *conn);
static int vde_sl_ctlfd(VDECONN *conn);
static int vde_sl_close(VDECONN *conn);

struct vdeplug_module vdeplug_ops={
	.vde_open_real=vde_sl_open,
	.vde_recv=vde_sl_recv,
	.vde_send=vde_sl_send,
	.vde_datafd=vde_sl_datafd,
	.vde_ctlfd=vde_sl_ctlfd,
	.vde_close=vde_sl_close};

struct vde_sl_conn {
	void *handle;
	struct vdeplug_module *module;
	int fd;
};

static speed_t itospeed(int ispeed) {
	switch (ispeed) {
		case 9600: return B9600; // POSIX values
		case 19200: return B19200;
		case 38400: return B38400;
#ifdef B57600
		case 57600: return B57600;
#endif
#ifdef B115200
		case 115200: return B115200;
#endif
#ifdef B230400
		case 230400: return B230400;
#endif
#ifdef B460800
		case 460800: return B460800;
#endif
#ifdef B500000
		case 500000: return B500000;
#endif
#ifdef B576000
		case 576000: return B576000;
#endif
#ifdef B921600
		case 921600: return B921600;
#endif
#ifdef B1000000
		case 1000000: return B1000000;
#endif
#ifdef B1152000
		case 1152000: return B1152000;
#endif
#ifdef B1500000
		case 1500000: return B1500000;
#endif
#ifdef B2000000
		case 2000000: return B2000000;
#endif
#ifdef B2500000
		case 2500000: return B2500000;
#endif
#ifdef B3000000
		case 3000000: return B3000000;
#endif
#ifdef B3500000
		case 3500000: return B3500000;
#endif
#ifdef B4000000
		case 4000000: return B4000000;
#endif
		default: return B0;
	}
}

static VDECONN *vde_sl_open(char *vde_url, char *descr,int interface_version,
		struct vde_open_args *open_args)
{
	struct vde_sl_conn *newconn;
	struct termios tty;
	char *speedstr = NULL;
	speed_t speed = VDESL_DEFAULT_SPEED;
	struct vdeparms parms[] = {
		{"speed", &speedstr},
		{NULL, NULL}};

#ifdef SIMULATE_NOISY_LINE
	srandom(time(NULL));
#endif

	if (vde_parsepathparms(vde_url, parms) != 0)
    return NULL;

	if (speedstr) {
		if ((speed = itospeed(atoi(speedstr))) == B0) {
			errno = EINVAL;
			return NULL;
		}
	}

	if ((newconn=calloc(1,sizeof(struct vde_sl_conn)))==NULL)
	{
		errno=ENOMEM;
		goto abort;
	}
	// options
	// newconn->fd = open args 
	// goto free_abort in case of error
	newconn->fd = open(vde_url,  O_RDWR);
	if (newconn->fd < 0)
		goto free_abort;

	memset(&tty, 0, sizeof tty);
	if(tcgetattr(newconn->fd, &tty) != 0)
		goto close_abort;

	/* cflags turn off: PARENB (no parity) CSTOPB (one stop) CRTSCTS(no hw RTS/CTS)
		 cflags turn on:  CS8 (8bit per char) CREAD (enable recv) CLOCAL (ignore ctl lines) */
	tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
	tty.c_cflag |= CS8 | CREAD | CLOCAL;

	/* lflags turn off ICANON, all echoes,  special chars(ISIG) */
	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);

	/* iflags turn off sw flow control, special processing of received bytes */
	tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); 

	/* oflags no processing of CR/NL */
	tty.c_oflag &= ~(OPOST | ONLCR); 

	tty.c_cc[VTIME] = 1;    // 100ms next byte max delay
	tty.c_cc[VMIN] = 0;

	if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0 ||
			tcsetattr(newconn->fd, TCSANOW, &tty) != 0)
		goto close_abort;

	return (VDECONN *)newconn;

close_abort:
	close(newconn->fd);
free_abort:
	free(newconn);
abort:
	return NULL;
}

static ssize_t vde_sl_recv(VDECONN *conn,void *buf,size_t len,int flags)
{
	struct vde_sl_conn *vde_conn = (struct vde_sl_conn *)conn;
	unsigned char header[2];
	unsigned int pktlen;
	unsigned int taillen;
	ssize_t rv;
	int pos;
	if ((rv = read(vde_conn->fd, header, 2)) <= 0)
		goto error_or_skip;
	else if (rv == 1) {
		if ((rv = read(vde_conn->fd, header + 1, 1)) <= 0)
			goto error_or_skip;
	}
	pktlen = (header[0]<<8) + header[1];
	if (pktlen > VDE_ETHBUFSIZE) {
		rv = 0;
		goto error_or_skip;
	}
	if (pktlen <= len)
		taillen = 0;
	else {
		taillen = pktlen - len;
		pktlen = len;
	}
	for (pos = 0; pos < pktlen; pos += rv) {
		if ((rv = read(vde_conn->fd, buf + pos, pktlen - pos)) <= 0)
			goto error_or_skip;
	}
	for (pos = 0; pos < taillen; pos += rv) {
		char fakebuf[taillen];
		if ((rv = read(vde_conn->fd, fakebuf, taillen - pos)) < 0)
			goto error_or_skip;
	}
	return pktlen;
error_or_skip:
	if (rv == 0) {
#ifdef SIMULATE_NOISY_LINE
		printf("skip\n");
#endif
		errno = EAGAIN;
		return 1;
	} else {
#ifdef SIMULATE_NOISY_LINE
		printf("err\n");
#endif
		return rv;
	}
}

static ssize_t vde_sl_send(VDECONN *conn,const void *buf,size_t len,int flags)
{
	struct vde_sl_conn *vde_conn = (struct vde_sl_conn *)conn;
#ifdef SIMULATE_NOISY_LINE
	if (random() % 10 == 0) {
		char dirty = random();
		write(vde_conn->fd, &dirty, 1);
		printf("Fault!\n");
	}
#endif
	if (len <= VDE_ETHBUFSIZE) {
		unsigned char header[2];
		struct iovec iov[2]={{header,2},{(void *)buf,len}};
		header[0]=len >> 8;
		header[1]=len & 0xff;
		return writev(vde_conn->fd,iov,2);
	} else
		return 0;
}

static int vde_sl_datafd(VDECONN *conn)
{
	struct vde_sl_conn *vde_conn = (struct vde_sl_conn *)conn;
	return vde_conn->fd;
}

static int vde_sl_ctlfd(VDECONN *conn)
{
	return -1;
}

static int vde_sl_close(VDECONN *conn)
{
	struct vde_sl_conn *vde_conn = (struct vde_sl_conn *)conn;
	int status;
	close(vde_conn->fd);
	free(vde_conn);
	return 0;
}
