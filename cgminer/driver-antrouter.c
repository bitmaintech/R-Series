/*
 * Copyright 2012-2013 Andrew Smith
 * Copyright 2013 Con Kolivas <kernel@kolivas.org>
 * Copyright 2013 Lingchao Xu <lingchao.xu@bitmaintech.com>
 * Copyright 2013 jinlong xue <jinlong.xue@bitmaintech.com>
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

/*
 * Those code should be works fine with AntMiner R1.
 * Operation:
 *   No detection implement.
 *   Input: 64B = 32B midstate + 20B fill bytes + last 12 bytes of block head.
 *   Return: send back 40bits immediately when antrouter found a valid nonce.
 *           no query protocol implemented here, if no data send back in ~11.3
 *           seconds (full cover time on 32bit nonce range by 380MH/s speed)
 *           just send another work.
 * Notice:
 *   1. antrouter will start calculate when you push a work to them, even they
 *      are busy.
 *   2. antrouter will stop work when: a valid nonce has been found or 40 bits
 *      nonce range is completely calculated.
 */


#include <float.h>
#include <limits.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <strings.h>
#include <sys/time.h>
#include <unistd.h>
#ifdef unix
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/select.h>
#include <fcntl.h>
#endif
#include "config.h"

#ifdef WIN32
#include <windows.h>
#endif

#include "compat.h"
#include "miner.h"
#include "driver-antrouter.h"

// The serial I/O speed - Linux uses a define 'B115200' in bits/termios.h
#define ANTROUTER_IO_SPEED 115200

#define ANTROUTER_NONCE_ARRAY_SIZE 6

// The size of a successful nonce read
#define ANTROUTER_READ_SIZE 5

// Ensure the sizes are correct for the Serial read
#if (ANTROUTER_READ_SIZE != 5)
#error ANTROUTER_READ_SIZE must be 5
#endif
#define ASSERT1(condition) __maybe_unused static char sizeof_uint32_t_must_be_4[(condition)?1:-1]
ASSERT1(sizeof(uint32_t) == 4);

// TODO: USB? Different calculation? - see usbstats to work it out e.g. 1/2 of normal send time
//  or even use that number? 1/2
// #define ANTROUTER_READ_TIME(baud) ((double)ANTROUTER_READ_SIZE * (double)8.0 / (double)(baud))
// maybe 1ms?
#define ANTROUTER_READ_TIME(baud) (0.001)

// USB ms timeout to wait - user specified timeouts are multiples of this
#define ANTROUTER_WAIT_TIMEOUT 100
#define ANTROUTER_CMR2_TIMEOUT 1
#define ANTROUTER_READ_BUF_LEN 8192

#ifndef O_CLOEXEC
#define O_CLOEXEC 0
#endif

// Defined in multiples of ANTROUTER_WAIT_TIMEOUT
// Must of course be greater than ANTROUTER_READ_COUNT_TIMING/ANTROUTER_WAIT_TIMEOUT
// There's no need to have this bigger, since the overhead/latency of extra work
// is pretty small once you get beyond a 10s nonce range time and 10s also
// means that nothing slower than 429MH/s can go idle so most antrouter devices
// will always mine without idling
#define ANTROUTER_READ_TIME_LIMIT_MAX 100

// In timing mode: Default starting value until an estimate can be obtained
// 5000 ms allows for up to a ~840MH/s device
#define ANTROUTER_READ_COUNT_TIMING	5000
#define ANTROUTER_READ_COUNT_MIN		ANTROUTER_WAIT_TIMEOUT
#define SECTOMS(s)	((int)((s) * 1000))
// How many ms below the expected completion time to abort work
// extra in case the last read is delayed
#define ANTROUTER_READ_REDUCE	((int)(ANTROUTER_WAIT_TIMEOUT * 1.5))

// For a standard antrouter (to 5 places)
// Since this rounds up a the last digit - it is a slight overestimate
// Thus the hash rate will be a VERY slight underestimate
// (by a lot less than the displayed accuracy)
// Minor inaccuracy of these numbers doesn't affect the work done,
// only the displayed MH/s
#define ANTROUTER_REV3_HASH_TIME 0.0000000026316
#define LANCELOT_HASH_TIME 0.0000000025000
#define ASICMINERUSB_HASH_TIME 0.0000000029761
// TODO: What is it?
#define CAIRNSMORE1_HASH_TIME 0.0000000027000
// Per FPGA
#define CAIRNSMORE2_HASH_TIME 0.0000000066600
#define NANOSEC 1000000000.0

#define CAIRNSMORE2_INTS 4

// antrouter doesn't send a completion message when it finishes
// the full nonce range, so to avoid being idle we must abort the
// work (by starting a new work item) shortly before it finishes
//
// Thus we need to estimate 2 things:
//	1) How many hashes were done if the work was aborted
//	2) How high can the timeout be before the antrouter is idle,
//		to minimise the number of work items started
//	We set 2) to 'the calculated estimate' - ANTROUTER_READ_REDUCE
//	to ensure the estimate ends before idle
//
// The simple calculation used is:
//	Tn = Total time in seconds to calculate n hashes
//	Hs = seconds per hash
//	Xn = number of hashes
//	W  = code/usb overhead per work
//
// Rough but reasonable estimate:
//	Tn = Hs * Xn + W	(of the form y = mx + b)
//
// Thus:
//	Line of best fit (using least squares)
//
//	Hs = (n*Sum(XiTi)-Sum(Xi)*Sum(Ti))/(n*Sum(Xi^2)-Sum(Xi)^2)
//	W = Sum(Ti)/n - (Hs*Sum(Xi))/n
//
// N.B. W is less when aborting work since we aren't waiting for the reply
//	to be transferred back (ANTROUTER_READ_TIME)
//	Calculating the hashes aborted at n seconds is thus just n/Hs
//	(though this is still a slight overestimate due to code delays)
//

// Both below must be exceeded to complete a set of data
// Minimum how long after the first, the last data point must be
#define HISTORY_SEC 60
// Minimum how many points a single ANTROUTER_HISTORY should have
#define MIN_DATA_COUNT 5
// The value MIN_DATA_COUNT used is doubled each history until it exceeds:
#define MAX_MIN_DATA_COUNT 100

static struct timeval history_sec = { HISTORY_SEC, 0 };

// Store the last INFO_HISTORY data sets
// [0] = current data, not yet ready to be included as an estimate
// Each new data set throws the last old set off the end thus
// keeping a ongoing average of recent data
#define INFO_HISTORY 10

#define ANTROUTER_WORK_QUEUE_NUM 36

struct ANTROUTER_HISTORY {
	struct timeval finish;
	double sumXiTi;
	double sumXi;
	double sumTi;
	double sumXi2;
	uint32_t values;
	uint32_t hash_count_min;
	uint32_t hash_count_max;
};

enum timing_mode { MODE_DEFAULT, MODE_SHORT, MODE_LONG, MODE_VALUE };

static const char *MODE_DEFAULT_STR = "default";
static const char *MODE_SHORT_STR = "short";
static const char *MODE_SHORT_STREQ = "short=";
static const char *MODE_LONG_STR = "long";
static const char *MODE_LONG_STREQ = "long=";
static const char *MODE_VALUE_STR = "value";
static const char *MODE_UNKNOWN_STR = "unknown";

struct ANTROUTER_INFO {
	enum sub_ident ident;
	int intinfo;

	// time to calculate the golden_ob
	uint64_t golden_hashes;
	struct timeval golden_tv;

	struct ANTROUTER_HISTORY history[INFO_HISTORY+1];
	uint32_t min_data_count;

	int timeout;

	// seconds per Hash
	double Hs;
	// ms til we abort
	int read_time;
	// ms limit for (short=/long=) read_time
	int read_time_limit;

	enum timing_mode timing_mode;
	bool do_antrouter_timing;

	bool start;

	double fullnonce;
	int count;
	double W;
	uint32_t values;
	uint64_t hash_count_range;

	// Determine the cost of history processing
	// (which will only affect W)
	uint64_t history_count;
	struct timeval history_time;

	// antrouter-options
	int baud;
	int work_division;
	int fpga_count;
	int frequency;
	char frequency_t[64];
	uint32_t nonce_mask;

	uint8_t cmr2_speed;
	bool speed_next_work;
	bool flash_next_work;

	struct work * work_queue[ANTROUTER_WORK_QUEUE_NUM];
	int work_queue_index;

	unsigned char nonce_bin[ANTROUTER_NONCE_ARRAY_SIZE][ANTROUTER_READ_SIZE+1];
	int nonce_index;
};


#define ANTROUTER_MIDSTATE_SIZE 32
#define ANTROUTER_UNUSED_SIZE 15
#define ANTROUTER_WORK_SIZE 12

#define ANTROUTER_WORK_DATA_OFFSET 64

#define ANTROUTER_CMR2_SPEED_FACTOR 2.5
#define ANTROUTER_CMR2_SPEED_MIN_INT 100
#define ANTROUTER_CMR2_SPEED_DEF_INT 180
#define ANTROUTER_CMR2_SPEED_MAX_INT 220
#define CMR2_INT_TO_SPEED(_speed) ((uint8_t)((float)_speed / ANTROUTER_CMR2_SPEED_FACTOR))
#define ANTROUTER_CMR2_SPEED_MIN CMR2_INT_TO_SPEED(ANTROUTER_CMR2_SPEED_MIN_INT)
#define ANTROUTER_CMR2_SPEED_DEF CMR2_INT_TO_SPEED(ANTROUTER_CMR2_SPEED_DEF_INT)
#define ANTROUTER_CMR2_SPEED_MAX CMR2_INT_TO_SPEED(ANTROUTER_CMR2_SPEED_MAX_INT)
#define ANTROUTER_CMR2_SPEED_INC 1
#define ANTROUTER_CMR2_SPEED_DEC -1
#define ANTROUTER_CMR2_SPEED_FAIL -10

#define ANTROUTER_CMR2_PREFIX ((uint8_t)0xB7)
#define ANTROUTER_CMR2_CMD_SPEED ((uint8_t)0)
#define ANTROUTER_CMR2_CMD_FLASH ((uint8_t)1)
#define ANTROUTER_CMR2_DATA_FLASH_OFF ((uint8_t)0)
#define ANTROUTER_CMR2_DATA_FLASH_ON ((uint8_t)1)
#define ANTROUTER_CMR2_CHECK ((uint8_t)0x6D)

struct ANTROUTER_WORK {
	uint8_t midstate[ANTROUTER_MIDSTATE_SIZE];
	// These 4 bytes are for CMR2 bitstreams that handle MHz adjustment
	uint8_t check;
	uint8_t data;
	uint8_t cmd;
	uint8_t prefix;
	uint8_t unused[ANTROUTER_UNUSED_SIZE];
	uint8_t workid;
	uint8_t work[ANTROUTER_WORK_SIZE];
};

#define END_CONDITION 0x0000ffff

// Looking for options in --antrouter-timing and --antrouter-options:
//
// Code increments this each time we start to look at a device
// However, this means that if other devices are checked by
// the antrouter code (e.g. Avalon only as at 20130517)
// they will count in the option offset
//
// This, however, is deterministic so that's OK
//
// If we were to increment after successfully finding an antrouter
// that would be random since an antrouter may fail and thus we'd
// not be able to predict the option order
//
// Devices are checked in the order libusb finds them which is ?
//
static int option_offset = -1;

unsigned char CRC5(unsigned char *ptr, unsigned char len)
{
    unsigned char i, j, k;
    unsigned char crc = 0x1f;

    unsigned char crcin[5] = {1, 1, 1, 1, 1};
    unsigned char crcout[5] = {1, 1, 1, 1, 1};
    unsigned char din = 0;

    j = 0x80;
    k = 0;
    for (i = 0; i < len; i++)
    {
    	if (*ptr & j) {
    		din = 1;
    	} else {
    		din = 0;
    	}
    	crcout[0] = crcin[4] ^ din;
    	crcout[1] = crcin[0];
    	crcout[2] = crcin[1] ^ crcin[4] ^ din;
    	crcout[3] = crcin[2];
    	crcout[4] = crcin[3];

        j = j >> 1;
        k++;
        if (k == 8)
        {
            j = 0x80;
            k = 0;
            ptr++;
        }
        memcpy(crcin, crcout, 5);
    }
    crc = 0;
    if(crcin[4]) {
    	crc |= 0x10;
    }
    if(crcin[3]) {
    	crc |= 0x08;
    }
    if(crcin[2]) {
    	crc |= 0x04;
    }
    if(crcin[1]) {
    	crc |= 0x02;
    }
    if(crcin[0]) {
    	crc |= 0x01;
    }
    return crc;
}

speed_t tiospeed_t(int baud)
{
	switch(baud){
		case 115200:return B115200;
	
	default:
		return B0;
	}
}

/* NOTE: Linux only supports uint8_t (decisecond) timeouts; limiting it in
 *       this interface buys us warnings when bad constants are passed in.
 */
int serial_open(const char *devpath, unsigned long baud, float timeout, bool purge)
{
#ifdef WIN32
	HANDLE hSerial = CreateFile(devpath, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (unlikely(hSerial == INVALID_HANDLE_VALUE))
	{
		DWORD e = GetLastError();
		switch (e) {
		case ERROR_ACCESS_DENIED:
			applog(LOG_ERR, "Do not have user privileges required to open %s", devpath);
			break;
		case ERROR_SHARING_VIOLATION:
			applog(LOG_ERR, "%s is already in use by another process", devpath);
			break;
		default:
			applog(LOG_DEBUG, "Open %s failed, GetLastError:%u", devpath, (unsigned)e);
			break;
		}
		return -1;
	}

	if (baud)
	{
		COMMCONFIG comCfg = {0};
		comCfg.dwSize = sizeof(COMMCONFIG);
		comCfg.wVersion = 1;
		comCfg.dcb.DCBlength = sizeof(DCB);
		comCfg.dcb.BaudRate = baud;
		comCfg.dcb.fBinary = 1;
		comCfg.dcb.fDtrControl = DTR_CONTROL_ENABLE;
		comCfg.dcb.fRtsControl = RTS_CONTROL_ENABLE;
		comCfg.dcb.ByteSize = 8;

		SetCommConfig(hSerial, &comCfg, sizeof(comCfg));
	}

	// Code must specify a valid timeout value (0 means don't timeout)
	const DWORD ctoms = ((DWORD)timeout * 100);
	COMMTIMEOUTS cto = {ctoms, 0, ctoms, 0, ctoms};
	SetCommTimeouts(hSerial, &cto);

	if (purge) {
		PurgeComm(hSerial, PURGE_RXABORT);
		PurgeComm(hSerial, PURGE_TXABORT);
		PurgeComm(hSerial, PURGE_RXCLEAR);
		PurgeComm(hSerial, PURGE_TXCLEAR);
	}

	return _open_osfhandle((intptr_t)hSerial, 0);
#else
	int fdDev = open(devpath, O_RDWR | O_CLOEXEC | O_NOCTTY );

	if (unlikely(fdDev == -1))
	{
		if (errno == EACCES)
			applog(LOG_ERR, "Do not have user privileges required to open %s", devpath);
		else
			applog(LOG_ERR, "Can not open %s", devpath);
		return -1;
	}
	
#if defined(LOCK_EX) && defined(LOCK_NB)
	if (likely(!flock(fdDev, LOCK_EX | LOCK_NB)))
		applog(LOG_DEBUG, "Acquired exclusive advisory lock on %s", devpath);
	else
	if (errno == EWOULDBLOCK)
	{
		applog(LOG_ERR, "%s is already in use by another process", devpath);
		close(fdDev);
		return -1;
	}
	else
		applog(LOG_WARNING, "Failed to acquire exclusive lock on %s:  (ignoring)", devpath);
#endif

	struct termios my_termios;
	tcgetattr(fdDev, &my_termios);

	if (baud)
	{
		speed_t speed = tiospeed_t(baud);
		if (speed == B0){
			applog(LOG_WARNING, "Unrecognized baud rate: %lu load default baud B115200", baud);
			cfsetispeed(&my_termios,B115200);
			cfsetospeed(&my_termios,B115200);
		}
		else
		{
			cfsetispeed(&my_termios, speed);
			cfsetospeed(&my_termios, speed);
		}
	}
	/* Set c_cflag options.*/
	my_termios.c_cflag &= ~(CSIZE | PARENB);
	my_termios.c_cflag |= CS8;
	my_termios.c_cflag |= CREAD;	
	
	/* Set c_iflag input options */
	my_termios.c_cflag |= CLOCAL;

	my_termios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK |
				ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	my_termios.c_oflag &= ~OPOST;
	my_termios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	my_termios.c_cflag &= ~CRTSCTS;

	
	// Code must specify a valid timeout value (0 means don't timeout)
	my_termios.c_cc[VTIME] = (cc_t)timeout;
	my_termios.c_cc[VMIN] = 0;

	tcsetattr(fdDev, TCSANOW, &my_termios);

	if (purge)
		tcflush(fdDev, TCIOFLUSH);
	return fdDev;
#endif
}

#define ANTROUTER_READ_FAULT_DECISECONDS 1
#define antrouter_open2(devpath, baud,timeout, purge)  serial_open(devpath, baud, ANTROUTER_READ_FAULT_DECISECONDS, purge)
#define antrouter_open(devpath, baud,timeout)  antrouter_open2(devpath, baud,timeout, false)

static bool antrouter_initialise(struct cgpu_info *antrouter, int baud,float timeout)
{
	struct ANTROUTER_INFO *info = (struct ANTROUTER_INFO *)(antrouter->device_data);
	static bool firstrun = true;
	if(!firstrun)
		return false;
	antrouter->device_path = "/dev/ttyATH0";
/*	int fp = antrouter_open2(antrouter->device_path, baud, timeout, true);
	if (unlikely(-1 == fp)) {
		applog(LOG_ERR, " Failed to open %s",antrouter->device_path);
		return false;
	}
	
	*/
	struct termios options;	  
	int fp = open(antrouter->device_path,O_RDWR|O_NOCTTY); 
	if(fp < 0) {
		quithere(1, "open ttyATH0 failed");
	}
	tcgetattr(fp,&options);
	cfsetispeed(&options,B115200);
	cfsetospeed(&options,B115200);

	options.c_cflag &= ~(CSIZE | PARENB);
	options.c_cflag |= CS8;
	options.c_cflag |= CREAD;
	options.c_cflag |= CLOCAL;

	options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK |
				ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	options.c_oflag &= ~OPOST;
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	options.c_cc[VTIME] = (cc_t)(timeout);
	options.c_cc[VMIN] = 0;
	tcsetattr(fp,TCSANOW,&options);
	if (firstrun)
		tcflush(fp, TCIOFLUSH);
	
	antrouter->device_fd = fp;
	firstrun = false;
	applog(LOG_INFO,"open devices success %d set timeout %f",fp,timeout);
	return true;
}
#define BTM_DEV_ERROR -2
#define BTM_NONCE_ERROR -1
#define BTM_NONCE_OK 0
#define BTM_NONCE_RESTART 1
#define BTM_NONCE_TIMEOUT 2
#if 0
static int antrouter_get_nonce(struct cgpu_info *antrouter, unsigned char *buf,struct timeval *tv_start,struct timeval *tv_finish,int read_time,int *reallen)
{
	struct ANTROUTER_INFO *info = (struct ANTROUTER_INFO *)(antrouter->device_data);
	int err, amt, rc;
	//int reallen = 0;

	if (antrouter->device_fd < 1)
		return BTM_DEV_ERROR;
	int fd = antrouter->device_fd;
	struct timeval tv_now, tv_timeout;
	
	timer_set_now(&tv_now);
	timer_set_delay(&tv_timeout, &tv_now,read_time);
	long sec = tv_timeout.tv_sec * 1000 + tv_timeout.tv_usec/1000;
	cgtime(tv_start);
	
	err = antrouter_read((char *)buf, fd, &tv_finish, NULL, &tv_timeout, &tv_now, ANTROUTER_READ_SIZE,&reallen);
	applog(LOG_ERR," antrouter_read result %d %l",err,sec);
	
	if (err < 0 && err != ANT_GETS_TIMEOUT) {
		applog(LOG_ERR, "%s%i: Comms error (rerr=%d )", antrouter->drv->name,
		       antrouter->device_fd, err );
		dev_error(antrouter, REASON_DEV_COMMS_ERROR);
		return BTM_NONCE_ERROR;
	}

	if (err == 0){
		return BTM_NONCE_OK;
	}		

	if(err ==1)
		return BTM_NONCE_RESTART;
	return BTM_NONCE_TIMEOUT;
}
#else
static int antrouter_get_nonce(struct cgpu_info *antrouter, unsigned char *buf, struct timeval *tv_start,
			    struct timeval *tv_finish, struct thr_info *thr, int read_time)
{
	struct ANTROUTER_INFO *info = (struct ANTROUTER_INFO *)(antrouter->device_data);
	int ret, rc,readlen = 0,totallen = 0 ;
	fd_set readfds;
	if (antrouter->device_fd < 0)
		return BTM_DEV_ERROR;
	FD_ZERO(&readfds);
	FD_SET(antrouter->device_fd, &readfds);
	

	struct timeval timeout;
	timeout.tv_sec=0;
	long t = timeout.tv_usec = read_time * 1000;
	applog(LOG_DEBUG,"readtime %d timeout %ld",read_time,t);
	cgtime(tv_start);
		
	ret = select(antrouter->device_fd+1, &readfds, NULL, NULL, &timeout);
	if(ret < 0)  
	{  
		applog(LOG_ERR,"select error!");  
		dev_error(antrouter, REASON_DEV_COMMS_ERROR);
		return BTM_NONCE_ERROR;
	} 
	if(ret == 0){
		applog(LOG_DEBUG, "Antrouter recv  nonce timesout");
		return BTM_NONCE_TIMEOUT;
	}
	ret = FD_ISSET(antrouter->device_fd, &readfds);
	if(ret > 0 ){
		uint8_t cnt = 0;
		while(1){			
			readlen = read(antrouter->device_fd,buf+totallen,ANTROUTER_READ_SIZE);
			totallen += readlen;
			if(cnt++ == 100)
			{
				applog(LOG_ERR,"get nonce %d times",cnt);
				break;
			}
			if(totallen == ANTROUTER_READ_SIZE)
				break;					
		}				
	}			
	cgtime(tv_finish);


	if (totallen = ANTROUTER_READ_SIZE)
		return BTM_NONCE_OK;

	rc = SECTOMS(tdiff(tv_finish, tv_start));
	if (thr && thr->work_restart) {
		applog(LOG_DEBUG, "antrouter Read: Work restart at %d ms", rc);
		return BTM_NONCE_RESTART;
	}

	if (readlen > 0)
		applog(LOG_DEBUG, "antrouter Read: Timeout reading for %d ms", rc);
	else
		applog(LOG_DEBUG, "antrouter Read: No data for %d ms", rc);
	
}
#endif 
static const char *timing_mode_str(enum timing_mode timing_mode)
{
	switch(timing_mode) {
	case MODE_DEFAULT:
		return MODE_DEFAULT_STR;
	case MODE_SHORT:
		return MODE_SHORT_STR;
	case MODE_LONG:
		return MODE_LONG_STR;
	case MODE_VALUE:
		return MODE_VALUE_STR;
	default:
		return MODE_UNKNOWN_STR;
	}
}


static void ant_init_time();
static void _now_is_not_set(__maybe_unused struct timeval *tv)
{
	ant_init_time();
	timer_set_now(tv);
}

void (*timer_set_now)(struct timeval *tv) = _now_is_not_set;
static void _now_gettimeofday(struct timeval *);

static
void ant_init_time()
{
	if (timer_set_now != _now_is_not_set)
		return;
#ifdef WIN32
	if (QueryPerformanceFrequency(&_perffreq) && _perffreq.QuadPart)
	{
		timer_set_now = _now_queryperformancecounter;
		applog(LOG_DEBUG, "Timers: Using QueryPerformanceCounter");
	}
	else
#endif
	{
		timer_set_now = _now_gettimeofday;
		applog(LOG_DEBUG, "Timers: Using gettimeofday");
	}
}

#ifndef WIN32
static
void _now_gettimeofday(struct timeval *tv)
{
	gettimeofday(tv, NULL);
}
#else
// Windows start time is since 1601 lol so convert it to unix epoch 1970. 
#define EPOCHFILETIME (116444736000000000LL)

void _now_gettimeofday(struct timeval *tv)
{
	FILETIME ft;
	LARGE_INTEGER li;

	GetSystemTimeAsFileTime(&ft);
	li.LowPart  = ft.dwLowDateTime;
	li.HighPart = ft.dwHighDateTime;
	li.QuadPart -= EPOCHFILETIME;

	///SystemTime is in decimicroseconds so divide by an unusual number *
	tv->tv_sec  = li.QuadPart / 10000000;
	tv->tv_usec = li.QuadPart % 10000000;
}
#endif


static void bmsc_set_timing_mode(int this_option_offset, struct cgpu_info *antrouter, float readtimeout)
{
	struct ANTROUTER_INFO *info = (struct ANTROUTER_INFO *)(antrouter->device_data);
	enum sub_ident ident;
	double Hs;
	char buf[BUFSIZ+1];
	char *ptr, *comma, *eq;
	size_t max;
	int i;

	ident = usb_ident(antrouter);
	switch (ident) {
		case IDENT_ICA:
			info->Hs = ANTROUTER_REV3_HASH_TIME;
			break;
		case IDENT_BLT:
		case IDENT_LLT:
			info->Hs = LANCELOT_HASH_TIME;
			break;
		case IDENT_AMU:
			info->Hs = ASICMINERUSB_HASH_TIME;
			break;
		case IDENT_CMR1:
			info->Hs = CAIRNSMORE1_HASH_TIME;
			break;
		case IDENT_CMR2:
			info->Hs = CAIRNSMORE2_HASH_TIME;
			break;
		default:
			quit(1, "Bmsc get_options() called with invalid %s ident=%d",
				antrouter->drv->name, ident);
	}

	info->read_time = 0;
	info->read_time_limit = 0; // 0 = no limit

	info->fullnonce = info->Hs * (((double) 0xffffffff) + 1);
	info->read_time = (int)(readtimeout * ANTROUTER_WAIT_TIMEOUT);

	if(info->read_time < 0)
		info->read_time = 1;

	info->timing_mode = MODE_DEFAULT;
	info->do_antrouter_timing = false;

	info->min_data_count = MIN_DATA_COUNT;

	// All values are in multiples of BMSC_WAIT_TIMEOUT
	info->read_time_limit *= ANTROUTER_WAIT_TIMEOUT;

	applog(LOG_ERR, "%s%d Init: mode=%s read_time=%dms limit=%dms Hs=%e",
			antrouter->drv->name, antrouter->cgminer_id,
			timing_mode_str(info->timing_mode),
			info->read_time, info->read_time_limit, info->Hs);
}

static void set_timing_mode(int this_option_offset, struct cgpu_info *antrouter, float readtimeout)
{
	struct ANTROUTER_INFO *info = (struct ANTROUTER_INFO *)(antrouter->device_data);
	double Hs;
	char buf[BUFSIZ+1];
	char *ptr, *comma, *eq;
	size_t max;
	int i;	
	info->Hs = ASICMINERUSB_HASH_TIME;	
	//applog(LOG_ERR, "antrouter get_options() called with invalid %s ",antrouter->drv->name);

	info->read_time = 0;
	info->read_time_limit = 0; // 0 = no limit

	info->fullnonce = info->Hs * (((double) 0xffffffff) + 1);
	info->read_time = (int)(readtimeout * ANTROUTER_WAIT_TIMEOUT);

	if(info->read_time < 0)
		info->read_time = 1;

	info->timing_mode = MODE_DEFAULT;
	info->do_antrouter_timing = false;

	info->min_data_count = MIN_DATA_COUNT;

	// All values are in multiples of ANTROUTER_WAIT_TIMEOUT
	info->read_time_limit *= ANTROUTER_WAIT_TIMEOUT;

	applog(LOG_ERR, "%s%d Init: mode=%s read_time=%dms limit=%dms Hs=%e",
			antrouter->drv->name, antrouter->cgminer_id,
			timing_mode_str(info->timing_mode),
			info->read_time, info->read_time_limit, info->Hs);
}

static uint32_t mask(int work_division)
{
	uint32_t nonce_mask = 0x7fffffff;

	// yes we can calculate these, but this way it's easy to see what they are
	switch (work_division) {
	case 1:
		nonce_mask = 0xffffffff;
		break;
	case 2:
		nonce_mask = 0x7fffffff;
		break;
	case 4:
		nonce_mask = 0x3fffffff;
		break;
	case 8:
		nonce_mask = 0x1fffffff;
		break;
	default:
		quit(1, "Invalid2 antrouter-options for work_division (%d) must be 1, 2, 4 or 8", work_division);
	}

	return nonce_mask;
}

static struct cgpu_info *com_alloc_cgpu(struct device_drv *drv, int threads)
{
	struct cgpu_info *cgpu = calloc(1, sizeof(*cgpu));
	if (unlikely(!cgpu))
		quit(1, "Failed to calloc cgpu for %s in usb_alloc_cgpu", drv->dname);

	cgpu->drv = drv;
	cgpu->deven = DEV_ENABLED;
	cgpu->threads = threads;

	return cgpu;
}

static struct cgpu_info *com_free_cgpu(struct cgpu_info *cgpu)
{
	if (cgpu->drv->copy)
		free(cgpu->drv);

	free(cgpu->device_path);

	free(cgpu);

	return NULL;
}

static void get_options(int this_option_offset, struct cgpu_info *antrouter, int *baud, float *readtimeout,int *frequency, char *frequency_t, uint8_t * reg_data)
{
	char buf[BUFSIZ+1];
	char *ptr, *comma, *colon, *colon2, *colon3;
	enum sub_ident ident;
	size_t max;
	int i, tmp;
	float tmpf;

	if (opt_antrouter_options == NULL)
		buf[0] = '\0';
	else {
		ptr = opt_antrouter_options;
		for (i = 0; i < this_option_offset; i++) {
			comma = strchr(ptr, ',');
			if (comma == NULL)
				break;
			ptr = comma + 1;
		}

		comma = strchr(ptr, ',');
		if (comma == NULL)
			max = strlen(ptr);
		else
			max = comma - ptr;

		if (max > BUFSIZ)
			max = BUFSIZ;
		strncpy(buf, ptr, max);
		buf[max] = '\0';
	}

	*baud = ANTROUTER_IO_SPEED;
	
	if (*buf) {
		colon = strchr(buf, ':');
		if (colon)
			*(colon++) = '\0';

		if (*buf) {
			tmp = atoi(buf);
			switch (tmp) {
			case 115200:
				*baud = 115200;
				break;
			case 57600:
				*baud = 57600;
				break;
			default:
				quit(1, "Invalid antrouter-options for baud (%s) must be 115200 or 57600", buf);
			}
		}

		if (colon && *colon) {
			colon2 = strchr(colon, ':');
			if (colon2)
				*(colon2++) = '\0';
			
			tmpf = atof(colon);
			if (tmpf > 0) {
				*readtimeout = tmpf;
			} else {
				quit(1, "Invalid antrouter-options for timeout (%s) must be > 0", colon);
			}
			applog(LOG_DEBUG,"get baud=%d,timeout=%f",*baud,*readtimeout);

			if (colon2 && *colon2){
				colon3 = strchr(colon2, ':');
				if (colon3)
					*(colon3++) = '\0';
				tmp = atoi(colon2);
				if (tmp < BITMAIN_MIN_FREQUENCY || tmp > BITMAIN_MAX_FREQUENCY) {
					quit(1, "Invalid antrouter-freq for frequency, must be %d <= frequency <= %d",
							BITMAIN_MIN_FREQUENCY, BITMAIN_MAX_FREQUENCY);
				} else {
					*frequency = tmp;
					strcpy(frequency_t, colon2);
				}

				if (colon3 && *colon3) {
					if(strlen(colon3) > 8 || strlen(colon3)%2 != 0 || strlen(colon3)/2 == 0) {
						quit(1, "Invalid antrouter-freq for reg data, must be hex now: %s",
								colon3);
					}
					memset(reg_data, 0, 4);
					if(!hex2bin(reg_data, colon3, strlen(colon3)/2)) {
						quit(1, "Invalid antrouter-freq for reg data, hex2bin error now: %s",
								colon3);
					}
				}
			}
		}
	}	
}

bool vcom_set_timeout_ms(const int fdDev, const unsigned timeout_ms)
{
#ifdef WIN32
	const HANDLE hSerial = (HANDLE)_get_osfhandle(fdDev);
	// Code must specify a valid timeout value (0 means don't timeout)
	const DWORD ctoms = timeout_ms;
	COMMTIMEOUTS cto = {ctoms, 0, ctoms, 0, ctoms};
	return (SetCommTimeouts(hSerial, &cto) != 0);
#else
	struct termios my_termios;
	
	tcgetattr(fdDev, &my_termios);
	my_termios.c_cc[VTIME] = (cc_t)((timeout_ms + 99) / 100);
	return (tcsetattr(fdDev, TCSANOW, &my_termios) == 0);
#endif
}

int antrouter_write(int fd, const void *buf, size_t bufLen)
{
	size_t ret;
	if (unlikely(fd == -1))
		return -1;	
	ret = write(fd, buf, bufLen);
	if (unlikely(ret != bufLen))
		return 1;
	return 0;
}

int antrouter_read(uint8_t *buf, const int fd, struct timeval * const tvp_finish, struct thr_info * const thr, const struct timeval * const tvp_timeout, struct timeval * const tvp_now, int read_size,int *reallen)
{
	int rv;
	long remaining_ms;
	ssize_t ret;
	struct timeval tv_start = *tvp_now;
	bool first = true;
	int i = 0;

	while (true) {
		remaining_ms = timer_remaining_us(tvp_timeout, tvp_now) / 1000;
			if (remaining_ms > 100)
				remaining_ms = 100;
			vcom_set_timeout_ms(fd, remaining_ms);
			// Read first byte alone to get earliest tv_finish
			ret = read(fd, buf, first ? 1 : read_size);
			if(ret < 0)
				perror("read");
			if(ret > 0)
				for(i = 0;i < ret;i ++){
					applog(LOG_DEBUG,"in real_read %02x",buf[i]);
				}
			timer_set_now(tvp_now);
		if (first)
			*tvp_finish = *tvp_now;
		if (ret)
		{
			if (unlikely(ret < 0))
			return rv = ANT_GETS_ERROR;
			
			first = false;
			
			if (ret >= read_size){
				return rv = ANT_GETS_OK;
			}
			read_size -= ret;
			buf += ret;
			reallen +=ret;
			continue;
		}
		if (thr && thr->work_restart)
			applog(LOG_ERR, " Interrupted by work restart");
			return rv = ANT_GETS_RESTART;
		
		if (timer_passed(tvp_timeout, tvp_now))
			applog(LOG_ERR, " No data in %.3f seconds",timer_elapsed_us(&tv_start, tvp_now) / 1e6);
			return rv = ANT_GETS_TIMEOUT;
	}
out:
	return rv;
}

int serial_close(const int fd)
{
#if defined(LOCK_EX) && defined(LOCK_NB) && defined(LOCK_UN)
	flock(fd, LOCK_UN);
#endif
	return close(fd);
}

void do_antrouter_close(struct thr_info *thr)
{
	struct cgpu_info *antrouter = thr->cgpu;
	const int fd = antrouter->device_fd;
	if (fd == -1)
		return;
	serial_close(fd);
	antrouter->device_fd = -1;
}

static int bmsc_get_nonce(struct cgpu_info *antrouter, unsigned char *buf, struct timeval *tv_start,
			    struct timeval *tv_finish, struct thr_info *thr, int read_time)
{
	struct ANTROUTER_INFO *info = (struct ANTROUTER_INFO *)(antrouter->device_data);
	int err, amt, rc;

	if (antrouter->usbinfo.nodev)
		return BTM_DEV_ERROR;

	cgtime(tv_start);
	err = usb_read_ii_timeout_cancellable(antrouter, info->intinfo, (char *)buf,
					      ANTROUTER_READ_SIZE, &amt, read_time,
					      C_GETRESULTS);
	cgtime(tv_finish);

	if (err < 0 && err != LIBUSB_ERROR_TIMEOUT) {
		applog(LOG_ERR, "%s%i: Comms error (rerr=%d amt=%d)", antrouter->drv->name,
		       antrouter->device_id, err, amt);
		dev_error(antrouter, REASON_DEV_COMMS_ERROR);
		return BTM_NONCE_ERROR;
	}

	if (amt >= ANTROUTER_READ_SIZE)
		return BTM_NONCE_OK;

	rc = SECTOMS(tdiff(tv_finish, tv_start));
	if (thr && thr->work_restart) {
		applog(LOG_DEBUG, "Bmsc Read: Work restart at %d ms", rc);
		return BTM_NONCE_RESTART;
	}

	if (amt > 0)
		applog(LOG_DEBUG, "Bmsc Read: Timeout reading for %d ms", rc);
	else
		applog(LOG_DEBUG, "Bmsc Read: No data for %d ms", rc);
	return BTM_NONCE_TIMEOUT;
}

static void _transfer(struct cgpu_info *antrouter, uint8_t request_type, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint32_t *data, int siz, enum usb_cmds cmd)
{
	int err;

	err = usb_transfer_data(antrouter, request_type, bRequest, wValue, wIndex, data, siz, cmd);

	applog(LOG_DEBUG, "%s: cgid %d %s got err %d",
			antrouter->drv->name, antrouter->cgminer_id,
			usb_cmdname(cmd), err);
}

#define transfer(antrouter, request_type, bRequest, wValue, wIndex, cmd) \
		_transfer(antrouter, request_type, bRequest, wValue, wIndex, NULL, 0, cmd)

static void bmsc_initialise(struct cgpu_info *antrouter, int baud)
{
	struct ANTROUTER_INFO *info = (struct ANTROUTER_INFO *)(antrouter->device_data);
	uint16_t wValue, wIndex;
	enum sub_ident ident;
	int interface;

	if (antrouter->usbinfo.nodev)
		return;

	interface = _usb_interface(antrouter, info->intinfo);
	ident = usb_ident(antrouter);

	switch (ident) {
		case IDENT_BLT:
		case IDENT_LLT:
		case IDENT_CMR1:
		case IDENT_CMR2:
			// Reset
			transfer(antrouter, FTDI_TYPE_OUT, FTDI_REQUEST_RESET, FTDI_VALUE_RESET,
				 interface, C_RESET);

			if (antrouter->usbinfo.nodev)
				return;

			// Latency
			_usb_ftdi_set_latency(antrouter, info->intinfo);

			if (antrouter->usbinfo.nodev)
				return;

			// Set data control
			transfer(antrouter, FTDI_TYPE_OUT, FTDI_REQUEST_DATA, FTDI_VALUE_DATA_BLT,
				 interface, C_SETDATA);

			if (antrouter->usbinfo.nodev)
				return;

			// default to BLT/LLT 115200
			wValue = FTDI_VALUE_BAUD_BLT;
			wIndex = FTDI_INDEX_BAUD_BLT;

			if (ident == IDENT_CMR1 || ident == IDENT_CMR2) {
				switch (baud) {
					case 115200:
						wValue = FTDI_VALUE_BAUD_CMR_115;
						wIndex = FTDI_INDEX_BAUD_CMR_115;
						break;
					case 57600:
						wValue = FTDI_VALUE_BAUD_CMR_57;
						wIndex = FTDI_INDEX_BAUD_CMR_57;
						break;
					default:
						quit(1, "bmsc_intialise() invalid baud (%d) for Cairnsmore1", baud);
						break;
				}
			}

			// Set the baud
			transfer(antrouter, FTDI_TYPE_OUT, FTDI_REQUEST_BAUD, wValue,
				 (wIndex & 0xff00) | interface, C_SETBAUD);

			if (antrouter->usbinfo.nodev)
				return;

			// Set Modem Control
			transfer(antrouter, FTDI_TYPE_OUT, FTDI_REQUEST_MODEM, FTDI_VALUE_MODEM,
				 interface, C_SETMODEM);

			if (antrouter->usbinfo.nodev)
				return;

			// Set Flow Control
			transfer(antrouter, FTDI_TYPE_OUT, FTDI_REQUEST_FLOW, FTDI_VALUE_FLOW,
				 interface, C_SETFLOW);

			if (antrouter->usbinfo.nodev)
				return;

			// Clear any sent data
			transfer(antrouter, FTDI_TYPE_OUT, FTDI_REQUEST_RESET, FTDI_VALUE_PURGE_TX,
				 interface, C_PURGETX);

			if (antrouter->usbinfo.nodev)
				return;

			// Clear any received data
			transfer(antrouter, FTDI_TYPE_OUT, FTDI_REQUEST_RESET, FTDI_VALUE_PURGE_RX,
				 interface, C_PURGERX);
			break;
		case IDENT_ICA:
			// Set Data Control
			transfer(antrouter, PL2303_CTRL_OUT, PL2303_REQUEST_CTRL, PL2303_VALUE_CTRL,
				 interface, C_SETDATA);

			if (antrouter->usbinfo.nodev)
				return;

			// Set Line Control
			uint32_t ica_data[2] = { PL2303_VALUE_LINE0, PL2303_VALUE_LINE1 };
			_transfer(antrouter, PL2303_CTRL_OUT, PL2303_REQUEST_LINE, PL2303_VALUE_LINE,
				 interface, &ica_data[0], PL2303_VALUE_LINE_SIZE, C_SETLINE);

			if (antrouter->usbinfo.nodev)
				return;

			// Vendor
			transfer(antrouter, PL2303_VENDOR_OUT, PL2303_REQUEST_VENDOR, PL2303_VALUE_VENDOR,
				 interface, C_VENDOR);
			break;
		case IDENT_AMU:
			// Enable the UART
			transfer(antrouter, CP210X_TYPE_OUT, CP210X_REQUEST_IFC_ENABLE,
				 CP210X_VALUE_UART_ENABLE,
				 interface, C_ENABLE_UART);

			if (antrouter->usbinfo.nodev)
				return;

			// Set data control
			transfer(antrouter, CP210X_TYPE_OUT, CP210X_REQUEST_DATA, CP210X_VALUE_DATA,
				 interface, C_SETDATA);

			if (antrouter->usbinfo.nodev)
				return;

			// Set the baud
			uint32_t data = CP210X_DATA_BAUD;
			_transfer(antrouter, CP210X_TYPE_OUT, CP210X_REQUEST_BAUD, 0,
				 interface, &data, sizeof(data), C_SETBAUD);
			break;
		default:
			quit(1, "bmsc_intialise() called with invalid %s cgid %i ident=%d",
				antrouter->drv->name, antrouter->cgminer_id, ident);
	}
}

static int64_t bmsc_scanwork(struct thr_info *thr)
{
	struct cgpu_info *antrouter = thr->cgpu;
	struct ANTROUTER_INFO *info = (struct ANTROUTER_INFO *)(antrouter->device_data);
	int ret, err, amount;
	unsigned char nonce_bin[ANTROUTER_READ_SIZE];
	struct ANTROUTER_WORK workdata;

	uint32_t nonce;
	int64_t hash_count = 0;
	struct timeval tv_start, tv_finish, elapsed;
	struct timeval tv_history_start, tv_history_finish;
	double Ti, Xi;
	int curr_hw_errors, i;
	bool was_hw_error;
	struct work *work = NULL;
	struct work *worktmp = NULL;

//	static int count = 0;
	double Hs, W, fullnonce;
	int read_time;
	bool limited;
	int64_t estimate_hashes;
	uint32_t values;
	int64_t hash_count_range;
	unsigned char workid = 0;
	int submitfull = 0;
	bool submitnonceok = true;

	// Device is gone
	if (antrouter->usbinfo.nodev)
		return -1;

	elapsed.tv_sec = elapsed.tv_usec = 0;

	work = get_work(thr, thr->id);
	memset((void *)(&workdata), 0, sizeof(workdata));
	memcpy(&(workdata.midstate), work->midstate, ANTROUTER_MIDSTATE_SIZE);
	memcpy(&(workdata.work), work->data + ANTROUTER_WORK_DATA_OFFSET, ANTROUTER_WORK_SIZE);
	rev((void *)(&(workdata.midstate)), ANTROUTER_MIDSTATE_SIZE);
	rev((void *)(&(workdata.work)), ANTROUTER_WORK_SIZE);

	workdata.workid = work->id;
	workid = work->id;
	workid = workid & 0x1F;

	// We only want results for the work we are about to send
	usb_buffer_clear(antrouter);

	if(info->work_queue[workid]) {
		free_work(info->work_queue[workid]);
		info->work_queue[workid] = NULL;
	}
	info->work_queue[workid] = copy_work(work);

	err = usb_write_ii(antrouter, info->intinfo, (char *)(&workdata), sizeof(workdata), &amount, C_SENDWORK);
	if (err < 0 || amount != sizeof(workdata)) {
		applog(LOG_ERR, "%s%i: Comms error (werr=%d amt=%d)", antrouter->drv->name, antrouter->device_id, err, amount);
		dev_error(antrouter, REASON_DEV_COMMS_ERROR);
	//	antrouter->usbinfo.nodev = true;
		bmsc_initialise(antrouter, info->baud);
		goto out;
	}
	if (opt_debug) {
		char *ob_hex;
		ob_hex = bin2hex((void *)(&workdata), sizeof(workdata));
		applog(LOG_DEBUG, "%s%d: sent %s", antrouter->drv->name, antrouter->device_id, ob_hex);
		free(ob_hex);
	}

more_nonces:
	memset(nonce_bin, 0, sizeof(nonce_bin));
	ret = bmsc_get_nonce(antrouter, nonce_bin, &tv_start, &tv_finish, thr, info->read_time);
	if (ret == BTM_NONCE_ERROR)
		goto out;

	// aborted before becoming idle, get new work
	if (ret == BTM_NONCE_TIMEOUT || ret == BTM_NONCE_RESTART) {
		timersub(&tv_finish, &tv_start, &elapsed);

		// ONLY up to just when it aborted
		// We didn't read a reply so we don't subtract BMSC_READ_TIME
		estimate_hashes = ((double)(elapsed.tv_sec) + ((double)(elapsed.tv_usec))/((double)1000000)) / info->Hs;

		// If some Serial-USB delay allowed the full nonce range to
		// complete it can't have done more than a full nonce
		if (unlikely(estimate_hashes > 0xffffffff))
			estimate_hashes = 0xffffffff;

		applog(LOG_DEBUG, "%s%d: no nonce = 0x%08lX hashes (%ld.%06lds)", antrouter->drv->name, antrouter->device_id, (long unsigned int)estimate_hashes, elapsed.tv_sec, elapsed.tv_usec);

		hash_count = 0;
		goto out;
	}

	memcpy((char *)&nonce, nonce_bin, sizeof(nonce));
	nonce = htobe32(nonce);
	curr_hw_errors = antrouter->hw_errors;

	workid = nonce_bin[4];
	workid = workid & 0x1F;
	worktmp = info->work_queue[workid];
	if(info->start && workid == 0x1f){
		goto out;
	}else{
		info->start = false;
	}
	if(worktmp) {
		submitfull = 0;
		if(submit_nonce_1(thr, worktmp, nonce, &submitfull)) {
			submitnonceok = true;
			submit_nonce_2(worktmp);
		} else {
			if(submitfull) {
				submitnonceok = true;
			} else {
				submitnonceok = false;
			}
		}
		cg_logwork(worktmp, nonce_bin, submitnonceok);
	} else {
		applog(LOG_ERR, "%s%d: work %02x not find error", antrouter->drv->name, antrouter->device_id, workid);
	}

	was_hw_error = (curr_hw_errors > antrouter->hw_errors);
	/*if (usb_buffer_size(antrouter) >= BMSC_READ_SIZE)
		goto more_nonces;
	*/
	hash_count = (nonce & info->nonce_mask);
	hash_count++;
	hash_count *= info->fpga_count;

	hash_count = 0xffffffff;

	if (opt_debug || info->do_antrouter_timing)
		timersub(&tv_finish, &tv_start, &elapsed);

	applog(LOG_DEBUG, "%s%d: nonce = 0x%08x = 0x%08lX hashes (%ld.%06lds)", antrouter->drv->name, antrouter->device_id, nonce, (long unsigned int)hash_count, elapsed.tv_sec, elapsed.tv_usec);

out:
	free_work(work);
	return hash_count;
}

static void set_anu_volt(struct cgpu_info *antrouter)
{
	unsigned char cmd_buf[4];
	unsigned char voltage_data[2] = {0};
	int err, amount,flag;
	if(strlen(opt_antrouter_volt) > 4 || strlen(opt_antrouter_volt)%2 != 0 || strlen(opt_antrouter_volt)/2 == 0) {
		applog(LOG_ERR, "Invalid voltage data, must be hex now: %s, set 0800 as default", opt_antrouter_volt);
		voltage_data[0] = 0x08;
		voltage_data[1] = 0x00;
		goto set_volt;
	}
	if(!hex2bin(voltage_data, opt_antrouter_volt, strlen(opt_antrouter_volt)/2)) {
		applog(LOG_ERR, "Invalid voltage data, hex2bin error now: %s,set 0800 as default", opt_antrouter_volt);
		voltage_data[0] = 0x08;
		voltage_data[1] = 0x00;
		goto set_volt;
	}
set_volt:
	cmd_buf[0] = 0xaa;
	cmd_buf[1] = voltage_data[0];;
	cmd_buf[1] &=0x0f;
	cmd_buf[1] |=0xb0;
	cmd_buf[2] = voltage_data[1];
	cmd_buf[3] = 0x00;  //0-7
	cmd_buf[3] = CRC5(cmd_buf, 4*8 - 5);
	cmd_buf[3] |= 0xc0;
	applog(LOG_ERR, "---------------------start voltage----------------------");
	cgsleep_ms(500);
	applog(LOG_ERR, "Send voltage %02x%02x%02x%02x", cmd_buf[0], cmd_buf[1], cmd_buf[2], cmd_buf[3]);
	err = usb_write(antrouter, (char * )cmd_buf, 4, &amount, C_SENDTESTWORK);
	if (err != LIBUSB_SUCCESS || amount != 4) {
		applog(LOG_ERR, "Write voltage Comms error (werr=%d amount=%d)", err, amount);
	}
}
static void get_anu_addr(struct cgpu_info *antrouter,struct ANTROUTER_INFO *info)
{
	unsigned char cmd_buf[4], rdreg_buf[4],msg[1024];
	unsigned char rebuf[ANTROUTER_READ_BUF_LEN] = {0};
	int amount, err;
	int i,nodata,relen = 0;
	int realllen = 0;
	memset(cmd_buf, 0, 4);
	memset(rdreg_buf, 0, 4);
	rdreg_buf[0] = 4;
	rdreg_buf[0] |= 0x80;
	rdreg_buf[1] = 0; //16-23
	rdreg_buf[2] = 0x00;  //8-15
	rdreg_buf[3] = 0;
	rdreg_buf[3] = CRC5(rdreg_buf, 27);

	applog(LOG_ERR, "-----------------start rdreg------------------");
	applog(LOG_ERR, "Send getstatus %02x%02x%02x%02x", rdreg_buf[0], rdreg_buf[1], rdreg_buf[2], rdreg_buf[3]);
	for(i = 0; i < 10; i++) {
		usb_read_ii_timeout_cancellable(antrouter, info->intinfo, (char * )rebuf, ANTROUTER_READ_SIZE, &relen, 100, C_GETRESULTS);
	}

	err = usb_write_ii(antrouter, info->intinfo, (char * )rdreg_buf, 4, &amount, C_SENDWORK);
	if (err != LIBUSB_SUCCESS || amount != 4) {
		applog(LOG_ERR, "%s%i: Write rdreg Comms error (werr=%d amount=%d)", antrouter->drv->name, antrouter->device_id, err, amount);
	}
	applog(LOG_DEBUG, "Send getstatus ok");

	nodata = 0;
	realllen = 0;
	while (1) {
		relen = 0;
		err = usb_read_ii_timeout_cancellable(antrouter, info->intinfo, (char * )rebuf + realllen, ANTROUTER_READ_SIZE, &relen, 200, C_GETRESULTS);
		if (err < 0 && err != LIBUSB_ERROR_TIMEOUT) {
			applog(LOG_ERR, "%s%i: Read rdreg Comms error (rerr=%d relen=%d)", antrouter->drv->name, antrouter->device_id, err, relen);
			break;
		} else if (err == LIBUSB_ERROR_TIMEOUT) {
			applog(LOG_DEBUG, "%s%i: Read rdreg Comms timeout (rerr=%d relen=%d)", antrouter->drv->name, antrouter->device_id, err, relen);

			nodata++;
			if (nodata > 5) {
				applog(LOG_DEBUG, "Recv rdreg getstatus len=%d", realllen);
				for (i = 0; i < realllen; i += 5) {
					applog(LOG_ERR, "Recv %d rdreg getstatus=%02x%02x%02x%02x%02x", i / 5 + 1, rebuf[i], rebuf[i + 1], rebuf[i + 2], rebuf[i + 3], rebuf[i + 4]);
				}
				applog(LOG_ERR, "---------recv rdreg getstatus finish----------");
				break;
			}
			continue;
			} else {
				nodata = 0;
				realllen += relen;
				for (i = 0; i < relen; i++) {
					sprintf(msg + i * 2, "%02x", rebuf[i]);
				}
				applog(LOG_DEBUG, "Read data(%d):%s", relen, msg);
			}
	}	
}
static bool set_anu_freq(struct cgpu_info *antrouter, struct ANTROUTER_INFO *info, uint8_t * reg_data)
{
	unsigned char cmd_buf[4], rdreg_buf[4],msg[1024];
	unsigned char rebuf[ANTROUTER_READ_BUF_LEN] = {0};
	int amount, err;
	int i,nodata,relen = 0;
	int realllen = 0;
	int sendfreqstatus = 1;
	memset(cmd_buf, 0, 4);
	memset(rdreg_buf, 0, 4);
	cmd_buf[0] = 2;
	cmd_buf[0] |= 0x80;
	// default freq
	//cmd_buf[1] = 0x12; //16-23
	//cmd_buf[2] = 0x86;  //8-15
	cmd_buf[1] = reg_data[0];
	cmd_buf[2] = reg_data[1];
	cmd_buf[3] = 0;
	cmd_buf[3] = CRC5(cmd_buf, 27);

	rdreg_buf[0] = 4;
	rdreg_buf[0] |= 0x80;
	rdreg_buf[1] = 0; //16-23
	rdreg_buf[2] = 0x04;  //8-15
	rdreg_buf[3] = 0;
	rdreg_buf[3] = CRC5(rdreg_buf, 27);
	applog(LOG_ERR, "-----------------start freq-------------------");
	cgsleep_ms(100);

	applog(LOG_ERR, "Send frequency %02x%02x%02x%02x", cmd_buf[0], cmd_buf[1], cmd_buf[2], cmd_buf[3]);
	err = usb_write_ii(antrouter, info->intinfo, (char * )cmd_buf, 4, &amount, C_SENDWORK);
	if (err != LIBUSB_SUCCESS || amount != 4) {
		applog(LOG_ERR, "%s%i: Write freq Comms error (werr=%d amount=%d)", antrouter->drv->name, antrouter->device_id, err, amount);
		return false;
	}
	applog(LOG_DEBUG, "Send frequency ok");
	cgsleep_ms(100);
	applog(LOG_ERR, "Send freq getstatus %02x%02x%02x%02x", rdreg_buf[0], rdreg_buf[1], rdreg_buf[2], rdreg_buf[3]);
	if(opt_debug){
		for(i = 0; i < 10; i++) {
			usb_read_ii_timeout_cancellable(antrouter, info->intinfo, (char * )rebuf, ANTROUTER_READ_SIZE, &relen, 100, C_GETRESULTS);
		}
		err = usb_write_ii(antrouter, info->intinfo, (char * )rdreg_buf, 4, &amount, C_SENDWORK);
		if (err != LIBUSB_SUCCESS || amount != 4) {
			applog(LOG_ERR, "%s%i: Write freq getstatus Comms error (werr=%d amount=%d)", antrouter->drv->name, antrouter->device_id, err, amount);
			return false;
		}
		applog(LOG_DEBUG, "Send freq getstatus ok");
		/*read reg data*/

		nodata = 0;
		realllen = 0;
		while (1) {
			relen = 0;
			err = usb_read_ii_timeout_cancellable(antrouter, info->intinfo, (char * )rebuf + realllen, ANTROUTER_READ_SIZE, &relen, 200, C_GETRESULTS);
			if (err < 0 && err != LIBUSB_ERROR_TIMEOUT) {
				applog(LOG_ERR, "%s%i: Read freq Comms error (rerr=%d relen=%d)", antrouter->drv->name, antrouter->device_id, err, relen);
				break;
			} else if (err == LIBUSB_ERROR_TIMEOUT) {
				applog(LOG_DEBUG, "%s%i: Read freq Comms timeout (rerr=%d relen=%d)", antrouter->drv->name, antrouter->device_id, err, relen);
				nodata++;
				if (nodata > 5) {
					if (realllen <= 0) {
						if (sendfreqstatus) {
							sendfreqstatus = 0;
							applog(LOG_ERR, "Send freq getstatus %02x%02x%02x%02x", rdreg_buf[0], rdreg_buf[1], rdreg_buf[2], rdreg_buf[3]);
							usb_read_ii_timeout_cancellable(antrouter, info->intinfo, (char * )rebuf, ANTROUTER_READ_SIZE, &relen, 200, C_GETRESULTS);
							err = usb_write_ii(antrouter, info->intinfo, (char * )rdreg_buf, 4, &amount, C_SENDWORK);
							if (err != LIBUSB_SUCCESS || amount != 4) {
								applog(LOG_ERR, "%s%i: Write freq getstatus Comms error (werr=%d amount=%d)", antrouter->drv->name, antrouter->device_id, err, amount);
									continue;
							}
							applog(LOG_DEBUG, "Send freq getstatus ok");
						} else {
							applog(LOG_ERR, "------recv freq getstatus no data finish------");
							break;
						}
					} else {
						applog(LOG_DEBUG, "Recv freq getstatus len=%d", realllen);
						for (i = 0; i < realllen; i += 5) {
							applog(LOG_ERR, "Recv %d freq getstatus=%02x%02x%02x%02x%02x", i / 5 + 1, rebuf[i], rebuf[i + 1], rebuf[i + 2], rebuf[i + 3], rebuf[i + 4]);
						}
						applog(LOG_ERR, "--------recv freq getstatus ok finish---------");
						break;
					}
				}
				continue;
			} else {
				nodata = 0;
				realllen += relen;
				for (i = 0; i < relen; i++) {
					sprintf(msg + i * 2, "%02x", rebuf[i]);
				}
				applog(LOG_DEBUG, "Read data(%d):%s", relen, msg);
			}
		}
	}
	return true;
}

static struct cgpu_info *bmsc_detect_one(struct libusb_device *dev, struct usb_find_devices *found)
{
	struct ANTROUTER_INFO *info;
	struct timeval tv_start, tv_finish;

	// Block 171874 nonce = (0xa2870100) = 0x000187a2
	// N.B. golden_ob MUST take less time to calculate
	//	than the timeout set in bmsc_open()
	//	This one takes ~0.53ms on Rev3 Bmsc
	const char golden_ob[] =
		"4679ba4ec99876bf4bfe086082b40025"
		"4df6c356451471139a3afa71e48f544a"
		"00000000000000000000000000000000"
		"0000001f87320b1a1426674f2fa722ce";
	const char golden_nonce[] = "000187a2";
	const uint32_t golden_nonce_val = 0x000187a2;
	unsigned char nonce_bin[ANTROUTER_READ_SIZE];
	struct ANTROUTER_WORK workdata;
	char *nonce_hex;
	int baud = 115200,frequency = 0, work_division = 1, fpga_count = 1;
	float readtimeout = 0.5;
	struct cgpu_info *antrouter;
	int ret, err, amount, tries, i;
	bool ok;
	bool cmr2_ok[CAIRNSMORE2_INTS];
	int cmr2_count;

	unsigned char cmd_buf[4] = {0};
	unsigned char rdreg_buf[4] = {0};
	char frequency_t[64] = {0};
	unsigned char reg_data[4] = {0};

	unsigned char rebuf[ANTROUTER_READ_BUF_LEN] = {0};
	int relen = 0;
	int realllen = 0;
	int nodata = 0;
	char msg[10240] = {0};
	int sendfreqstatus = 1;
	int k = 0;

	unsigned char core_cmd[4] = {0};
	int corenum = 0;
	char coreenable[256] = {0};
	int coresleep = 0;

	if ((sizeof(workdata) << 1) != (sizeof(golden_ob) - 1))
		quithere(1, "Data and golden_ob sizes don't match");

	antrouter = usb_alloc_cgpu(&antrouter_drv, 1);

	if (!usb_init(antrouter, dev, found))
		goto shin;

	//get_options(this_option_offset, antrouter, &baud, &readtimeout);
	//get_bandops(core_cmd, &corenum, coreenable, &coresleep);
	get_options(0, antrouter, &baud, &readtimeout,&frequency, frequency_t,reg_data);
	applog(LOG_ERR,"reg_data=%02x%02x",reg_data[0],reg_data[1]);

	info = (struct ANTROUTER_INFO *)calloc(1, sizeof(struct ANTROUTER_INFO));
	if (unlikely(!info))
		quit(1, "Failed to malloc ANTROUTER_INFO in bmsc_detect_one");
	antrouter->device_data = (void *)info;

	info->ident = usb_ident(antrouter);
	info->start = true;
	switch (info->ident) {
		case IDENT_ICA:
		case IDENT_BLT:
		case IDENT_LLT:
		case IDENT_AMU:
		case IDENT_CMR1:
			info->timeout = ANTROUTER_WAIT_TIMEOUT;
			break;
		case IDENT_CMR2:
			if (found->intinfo_count != CAIRNSMORE2_INTS) {
				quithere(1, "CMR2 Interface count (%d) isn't expected: %d",
						found->intinfo_count,
						CAIRNSMORE2_INTS);
			}
			info->timeout = ANTROUTER_CMR2_TIMEOUT;
			cmr2_count = 0;
			for (i = 0; i < CAIRNSMORE2_INTS; i++)
				cmr2_ok[i] = false;
			break;
		default:
			quit(1, "%s bmsc_detect_one() invalid %s ident=%d",
				antrouter->drv->dname, antrouter->drv->dname, info->ident);
	}
// For CMR2 test each USB Interface
cmr2_retry:
	tries = 2;
	ok = false;
	while (!ok && tries-- > 0) {
		bmsc_initialise(antrouter, baud);
		if(opt_antrouter_volt && tries == 1)
			set_anu_volt(antrouter);
		if(!set_anu_freq(antrouter,info,reg_data))
			continue;
		if (opt_debug) {
			get_anu_addr(antrouter,info);
		}

		applog(LOG_ERR, "-----------------start nonce------------------");
		hex2bin((void *)(&workdata), golden_ob, sizeof(workdata));
		err = usb_write_ii(antrouter, info->intinfo, (char *)(&workdata), sizeof(workdata), &amount, C_SENDWORK);
		if (err != LIBUSB_SUCCESS || amount != sizeof(workdata))
			continue;

		memset(nonce_bin, 0, sizeof(nonce_bin));
		ret = bmsc_get_nonce(antrouter, nonce_bin, &tv_start, &tv_finish, NULL, 500);
		if (ret != BTM_NONCE_OK) {
			applog(LOG_ERR, "Bmsc recv golden nonce timeout");
			continue;
		}

		nonce_hex = bin2hex(nonce_bin, sizeof(nonce_bin));
		if (strncmp(nonce_hex, golden_nonce, 8) == 0)
			ok = true;
		else {
			applog(LOG_ERR, "Bmsc recv golden nonce %s != %s and retry", nonce_hex, golden_nonce);
			if (tries < 0 && info->ident != IDENT_CMR2) {
				applog(LOG_ERR, "Bmsc Detect: Test failed at %s: get %s, should: %s",
					antrouter->device_path, nonce_hex, golden_nonce);
			}
		}
		free(nonce_hex);
	}

	if (!ok) {
		if (info->ident != IDENT_CMR2)
			goto unshin;

		if (info->intinfo < CAIRNSMORE2_INTS-1) {
			info->intinfo++;
			goto cmr2_retry;
		}
	} else {
		if (info->ident == IDENT_CMR2) {
			applog(LOG_DEBUG,
				"Bmsc Detect: "
				"Test succeeded at %s i%d: got %s",
					antrouter->device_path, info->intinfo, golden_nonce);

			cmr2_ok[info->intinfo] = true;
			cmr2_count++;
			if (info->intinfo < CAIRNSMORE2_INTS-1) {
				info->intinfo++;
				goto cmr2_retry;
			}
		}
	}

	/* We have a real Bmsc! */
	if (!add_cgpu(antrouter))
		goto unshin;

	update_usb_stats(antrouter);

	applog(LOG_INFO, "%s%d: Found at %s",
		antrouter->drv->name, antrouter->device_id, antrouter->device_path);

	info->baud = baud;
	info->work_division = work_division;
	info->fpga_count = fpga_count;
	info->nonce_mask = mask(work_division);
	info->work_queue_index = 0;
	for(k = 0; k < ANTROUTER_WORK_QUEUE_NUM; k++) {
		info->work_queue[k] = NULL;
	}

	info->golden_hashes = (golden_nonce_val & info->nonce_mask) * fpga_count;
	
	timersub(&tv_finish, &tv_start, &(info->golden_tv));

	bmsc_set_timing_mode(0, antrouter, readtimeout);

	antrouter->drv->scanwork = bmsc_scanwork;
	antrouter->drv->dname = "AntU3";
	return antrouter;

unshin:

	usb_uninit(antrouter);
	free(info);
	antrouter->device_data = NULL;

shin:

	antrouter = usb_free_cgpu(antrouter);

	return NULL;
}

static bool antrouter_detect_one(const char *devpath)
{
	int this_option_offset = ++option_offset;
	struct ANTROUTER_INFO *info;
	struct timeval tv_start, tv_finish,timeout;
	// Block 171874 nonce = (0xa2870100) = 0x000187a2
	// N.B. golden_ob MUST take less time to calculate
	//	than the timeout set in antrouter_open()
	//	This one takes ~0.53ms on Rev3 antrouter
	const char golden_ob[] =
		"4679ba4ec99876bf4bfe086082b40025"
		"4df6c356451471139a3afa71e48f544a"
		"00000000000000000000000000000000"
		"0000000087320b1a1426674f2fa722ce";
	const char golden_nonce[] = "000187a2";
	const uint32_t golden_nonce_val = 0x000187a2;
	unsigned char nonce_bin[ANTROUTER_READ_SIZE];
	struct ANTROUTER_WORK workdata;
	char *nonce_hex;
	int baud = 115200, work_division = 1, fpga_count = 1,frequency = 0,miner_count = 1;
	float readtimeout = 1.0;
	struct cgpu_info *antrouter;
	fd_set readfds;
	int ret, err, amount, tries, i = 0;
	bool ok,get_golden_nonce = false;

	char frequency_t[64] = {0};
	unsigned char cmd_buf[4] = {0};
	unsigned char rdreg_buf[4] = {0};
	unsigned char reg_data[4] = {0};
	unsigned char rebuf[ANTROUTER_READ_BUF_LEN] = {0};
	int nodata = 0;
	char msg[10240] = {0};
	int k = 0;

	if (opt_antrouter_options == NULL)
		return false;

	if ((sizeof(workdata) << 1) != (sizeof(golden_ob) - 1))
		quithere(1, "Data and golden_ob sizes don't match");
	antrouter = com_alloc_cgpu(&antrouter_drv, 1);
	get_options(this_option_offset, antrouter, &baud, &readtimeout,&frequency, frequency_t,reg_data);
	applog(LOG_ERR,"reg_data=%02x%02x",reg_data[0],reg_data[1]);
	antrouter_initialise(antrouter, baud,readtimeout);
	info = (struct ANTROUTER_INFO *)calloc(1, sizeof(struct ANTROUTER_INFO));
	if (unlikely(!info))
		quit(1, "Failed to malloc ANTROUTER_INFO");
	antrouter->device_data = (void *)info;
	info->timeout = ANTROUTER_WAIT_TIMEOUT * readtimeout;
	info->frequency = frequency;
	strcpy(info->frequency_t, frequency_t);;

	applog(LOG_DEBUG, "%s antrouter_detect_one() invalid %s",
		antrouter->drv->dname, antrouter->drv->dname);

cmr2_retry:
	tries = 2;
	ok = false;
	while (!ok && tries-- > 0)
	{
		int readlen = 0;
		int totallen = 0;
	/*	if(!opt_debug)
			ok = true;
	*/
		if(opt_debug || TEST_MODE){
			timeout.tv_sec = 0;
			timeout.tv_usec = 800;
			FD_ZERO(&readfds);
			FD_SET(antrouter->device_fd, &readfds);
			
			rdreg_buf[0] = 4;
			rdreg_buf[0] |= 0x80;
			rdreg_buf[1] = 0; //16-23
			rdreg_buf[2] = 0x00;  //8-15
			rdreg_buf[3] = 0;
			rdreg_buf[3] = CRC5(rdreg_buf, 27);
			applog(LOG_ERR, "Send addr getstatus %02x%02x%02x%02x", rdreg_buf[0], rdreg_buf[1], rdreg_buf[2], rdreg_buf[3]);
			
			antrouter_write(antrouter->device_fd, (char *)(rdreg_buf), sizeof(rdreg_buf));
  
		//		antrouter_get_nonce(antrouter, rebuf, &tv_start, &tv_finish,500,&realllen);
			ret = select(antrouter->device_fd+1, &readfds, NULL, NULL, &timeout);
			if(ret < 0)  
			{  
				applog(LOG_ERR,"select error!\n");  
				continue;  
			} 
			ret = FD_ISSET(antrouter->device_fd, &readfds);
			if(ret > 0 ){
				while(1){
					readlen = read(antrouter->device_fd,rebuf+totallen,ANTROUTER_READ_SIZE);
					if(readlen == 0)
						nodata++;
					totallen += readlen;
					if(nodata > 5){
						applog(LOG_DEBUG, "Recv rdreg getstatus len=%d", totallen);
						for (i = 0; i < totallen; i += 5) {
							applog(LOG_ERR, "Recv %d rdreg getstatus=%02x%02x%02x%02x%02x", i / 5 + 1, rebuf[i], rebuf[i + 1], rebuf[i + 2], rebuf[i + 3], rebuf[i + 4]);
						}
						if(TEST_MODE)
							system("logger getstats ok");
						break;
					}
					continue;
				}
					
			}
			cgsleep_ms(200);
		}		

		if (strlen(reg_data) == 2) {
			applog(LOG_ERR, "-----------------start freq-------------------");
			cmd_buf[0] = 2;
			cmd_buf[0] |= 0x80;
			cmd_buf[1] = reg_data[0]; //16-23
			cmd_buf[2] = reg_data[1];  //8-15
			cmd_buf[3] = 0;
			cmd_buf[3] = CRC5(cmd_buf, 27);		
			cgsleep_ms(100);

			applog(LOG_ERR, "Send frequency %02x%02x%02x%02x", cmd_buf[0], cmd_buf[1], cmd_buf[2], cmd_buf[3]);
			err = antrouter_write(antrouter->device_fd, (char *)(cmd_buf), sizeof(cmd_buf));
			if(err != 0)
				continue;
			applog(LOG_DEBUG, "Send frequency ok");	
			cgsleep_ms(500);
		}

		if(opt_debug || TEST_MODE){
			applog(LOG_ERR, "-----------------start nonce------------------");
			applog(LOG_ERR, "AntRouter send golden nonce");
			FD_ZERO(&readfds);
			FD_SET(antrouter->device_fd, &readfds);
			hex2bin((void *)(&workdata), golden_ob, sizeof(workdata));
			ret = antrouter_write(antrouter->device_fd, (char *)(&workdata), sizeof(workdata));
			if(ret == 1)
				continue;
			cgsleep_ms(100);
			memset(nonce_bin, 0, sizeof(nonce_bin));

			/*ret = antrouter_get_nonce(antrouter, nonce_bin,&tv_start,&tv_finish,1000,&realllen);
			if (ret != BTM_NONCE_OK) {
				applog(LOG_ERR, "antrouter recv golden nonce timeout");
				continue;
			}*/
			timeout.tv_sec = 0;
			timeout.tv_usec = 600;
			ret = select(antrouter->device_fd+1, &readfds, NULL, NULL, &timeout);
			if(ret < 0)  
			{  
				applog(LOG_ERR,"select error!\n");  
				continue;  
			} 
			if(ret == 0){
				applog(LOG_ERR, "antrouter recv golden nonce timesout");
				continue;
			}
			ret = FD_ISSET(antrouter->device_fd, &readfds);
			if(ret > 0 ){
				readlen = totallen = 0;
				while(1){				
					readlen = read(antrouter->device_fd,nonce_bin+totallen,ANTROUTER_READ_SIZE);
					totallen += readlen;
					if(totallen == ANTROUTER_READ_SIZE)
						break;					
				}				
			}			
			nonce_hex = bin2hex(nonce_bin, sizeof(nonce_bin));
			if (strncmp(nonce_hex, golden_nonce, 8) == 0){
				ok = true;
				if(TEST_MODE){
					system("logger asic test ok");
					exit(0);			
				}
			}else {
				applog(LOG_ERR, "antrouter recv golden nonce %s != %s and retry", nonce_hex, golden_nonce);
				if (tries < 0 && info->ident != IDENT_CMR2) {
					applog(LOG_ERR, "antrouter Detect: Test failed at %s: get %s, should: %s",
						antrouter->device_path, nonce_hex, golden_nonce);
				}
				if(TEST_MODE){
					system("logger asic test failed");
					exit(1);					
				}
			}
			free(nonce_hex);
		}
	}

	if (!ok) {
		if (info->intinfo < CAIRNSMORE2_INTS-1) {
			info->intinfo++;
			goto cmr2_retry;
		}
	} 
	/* We have a real antrouter! */
	if (!add_cgpu(antrouter))
		goto unshin;

	applog(LOG_DEBUG, "%s%d: Init baud=%d work_division=%d fpga_count=%d readtimeout=%f",
		antrouter->drv->name, antrouter->device_fd, baud, work_division, fpga_count, readtimeout);

	info->baud = baud;
	info->work_division = work_division;
	info->fpga_count = fpga_count;
	info->count = miner_count;
	info->nonce_mask = mask(work_division);
	info->work_queue_index = 0;
	for(k = 0; k < ANTROUTER_WORK_QUEUE_NUM; k++) {
		info->work_queue[k] = NULL;
	}

	info->golden_hashes = (golden_nonce_val & info->nonce_mask) * fpga_count;
	timersub(&tv_finish, &tv_start, &(info->golden_tv));

	set_timing_mode(this_option_offset, antrouter, readtimeout);

	return true;

unshin:
	free(info);
	antrouter->device_data = NULL;
shin:
	antrouter = com_free_cgpu(antrouter);
	return false;
}

void ant_detect(struct device_drv *drv, bool (*device_detect)(const char*))
{
	applog(LOG_ERR, "ANT scan devices: checking for %s devices", drv->name);
	device_detect("/dev/ttyATH0");
}

void antrouter_detect(bool __maybe_unused hotplug)
{
	static bool firstrun = true;
	if(opt_api_port != 4028)
		usb_detect(&antrouter_drv, bmsc_detect_one);
	else{
		if(firstrun){
			ant_detect(&antrouter_drv, antrouter_detect_one);
			firstrun = false;
		}
	}	
}

static bool antrouter_prepare(__maybe_unused struct thr_info *thr)
{
//	struct cgpu_info *antrouter = thr->cgpu;
	return true;
}

static void cmr2_command(struct cgpu_info *bmsc, uint8_t cmd, uint8_t data)
{
	struct ANTROUTER_INFO *info = (struct ANTROUTER_INFO *)(bmsc->device_data);
	struct ANTROUTER_WORK workdata;
	int amount;

	memset((void *)(&workdata), 0, sizeof(workdata));

	workdata.prefix = ANTROUTER_CMR2_PREFIX;
	workdata.cmd = cmd;
	workdata.data = data;
	workdata.check = workdata.data ^ workdata.cmd ^ workdata.prefix ^ ANTROUTER_CMR2_CHECK;

	usb_write_ii(bmsc, info->intinfo, (char *)(&workdata), sizeof(workdata), &amount, C_SENDWORK);
}

static void cmr2_commands(struct cgpu_info *bmsc)
{
	struct ANTROUTER_INFO *info = (struct ANTROUTER_INFO *)(bmsc->device_data);

	if (info->speed_next_work) {
		info->speed_next_work = false;
		cmr2_command(bmsc, ANTROUTER_CMR2_CMD_SPEED, info->cmr2_speed);
		return;
	}

	if (info->flash_next_work) {
		info->flash_next_work = false;
		cmr2_command(bmsc, ANTROUTER_CMR2_CMD_FLASH, ANTROUTER_CMR2_DATA_FLASH_ON);
		cgsleep_ms(250);
		cmr2_command(bmsc, ANTROUTER_CMR2_CMD_FLASH, ANTROUTER_CMR2_DATA_FLASH_OFF);
		cgsleep_ms(250);
		cmr2_command(bmsc, ANTROUTER_CMR2_CMD_FLASH, ANTROUTER_CMR2_DATA_FLASH_ON);
		cgsleep_ms(250);
		cmr2_command(bmsc, ANTROUTER_CMR2_CMD_FLASH, ANTROUTER_CMR2_DATA_FLASH_OFF);
		return;
	}
}

static int64_t antrouter_scanwork(struct thr_info *thr)
{
	struct cgpu_info *antrouter = thr->cgpu;
	struct ANTROUTER_INFO *info = (struct ANTROUTER_INFO *)(antrouter->device_data);
	int ret, err, amount,fd;
	unsigned char nonce_bin[ANTROUTER_READ_SIZE];
	struct ANTROUTER_WORK workdata;
	char *ob_hex;
	uint32_t nonce;
	int64_t hash_count = 0;
	struct timeval tv_start, tv_finish, elapsed;
	struct timeval tv_history_start, tv_history_finish,tv_now;
	double Ti, Xi;
	int curr_hw_errors, i;
	bool was_hw_error;
	struct work *work = NULL;
	struct work *worktmp = NULL;

	int count;
	double Hs, W, fullnonce;
	int read_time;
	bool limited;
	int64_t estimate_hashes;
	uint32_t values;
	int64_t hash_count_range;
	unsigned char workid = 0;
	int submitfull = 0;
	bool submitnonceok = true;
	int realllen = 0,nbytes = 0;
	static int write_times = 0;
	// Device is gone
	if (antrouter->device_fd  <  1)
		return -1;
	fd = antrouter->device_fd;
	elapsed.tv_sec = elapsed.tv_usec = 0;

	work = get_work(thr, thr->id);
	memset((void *)(&workdata), 0, sizeof(workdata));
	memcpy(&(workdata.midstate), work->midstate, ANTROUTER_MIDSTATE_SIZE);
	memcpy(&(workdata.work), work->data + ANTROUTER_WORK_DATA_OFFSET, ANTROUTER_WORK_SIZE);
	rev((void *)(&(workdata.midstate)), ANTROUTER_MIDSTATE_SIZE);
	rev((void *)(&(workdata.work)), ANTROUTER_WORK_SIZE);

	workdata.workid = work->id;
	workid = work->id;
	workid = workid & 0x1F;

	if(info->work_queue[workid]) {
		free_work(info->work_queue[workid]);
		info->work_queue[workid] = NULL;
	}
	info->work_queue[workid] = copy_work(work);

	ret = antrouter_write(antrouter->device_fd, (char *)(&workdata), sizeof(workdata));
	applog(LOG_DEBUG,"write_times = %d",write_times++);
	if (ret) {
		do_antrouter_close(thr);
		applog(LOG_ERR, "%s: Comms error (werr=%d)", antrouter->device_path, ret);
		dev_error(antrouter, REASON_DEV_COMMS_ERROR);
		antrouter_initialise(antrouter,info->baud,info->read_time);
		goto out;	/* This should never happen */
	} 
	if (opt_debug) {
		ob_hex = bin2hex((void *)(&workdata), sizeof(workdata));
		applog(LOG_DEBUG, "%s%d: sent %s", antrouter->drv->name, antrouter->device_fd, ob_hex);
		free(ob_hex);
	}
more_nonces:
	memset(nonce_bin, 0, sizeof(nonce_bin));
	
	//ret = antrouter_get_nonce(antrouter, nonce_bin, &tv_start, &tv_finish,info->read_time,&realllen);
	ret = antrouter_get_nonce(antrouter, nonce_bin, &tv_start, &tv_finish, thr, info->read_time);
	if (ret == BTM_NONCE_ERROR)
		goto out;

	// aborted before becoming idle, get new work
	if (ret == BTM_NONCE_TIMEOUT || ret == BTM_NONCE_RESTART) {
		timersub(&tv_finish, &tv_start, &elapsed);
		// ONLY up to just when it aborted
		// We didn't read a reply so we don't subtract ANTROUTER_READ_TIME
		estimate_hashes = ((double)(elapsed.tv_sec) + ((double)(elapsed.tv_usec))/((double)1000000)) / info->Hs;

		// If some Serial-USB delay allowed the full nonce range to
		// complete it can't have done more than a full nonce
		if (unlikely(estimate_hashes > 0xffffffff))
			estimate_hashes = 0xffffffff;

		applog(LOG_DEBUG, "%s%d: no nonce = 0x%08lX hashes (%ld.%06lds)", antrouter->drv->name, antrouter->device_fd, (long unsigned int)estimate_hashes, elapsed.tv_sec, elapsed.tv_usec);

		hash_count = 0;
		goto out;
	}

	memcpy((char *)&nonce, nonce_bin, sizeof(nonce_bin));
	nonce = htobe32(nonce);
	curr_hw_errors = antrouter->hw_errors;

	workid = nonce_bin[4];
	workid = workid & 0x1F;
	worktmp = info->work_queue[workid];
	
	if(info->start && workid == 0x1f){
		goto out;
	}else{
		info->start = false;
	}
	if(worktmp) {
		submitfull = 0;
		if(submit_nonce_1(thr, worktmp, nonce, &submitfull)) {
			submitnonceok = true;
			submit_nonce_2(worktmp);
		} else {
			if(submitfull) {
				submitnonceok = true;
			} else {
				submitnonceok = false;
			}
		}
		cg_logwork(worktmp, nonce_bin, submitnonceok);
	} else {
		applog(LOG_ERR, "%s%d: work %02x not find error", antrouter->drv->name, antrouter->device_fd, workid);
	}

	was_hw_error = (curr_hw_errors > antrouter->hw_errors);

	if(ioctl(antrouter->device_fd, FIONREAD, &nbytes) == 0){
		applog(LOG_ERR,"read buffer %d",nbytes);
		if(nbytes >= ANTROUTER_READ_SIZE)		
			goto more_nonces;
	}
	hash_count = (nonce & info->nonce_mask);
	hash_count++;
	hash_count *= info->fpga_count;

	hash_count = 0xffffffff;

	if (opt_debug || info->do_antrouter_timing)
		timersub(&tv_finish, &tv_start, &elapsed);

	applog(LOG_DEBUG, "%s%d: nonce = 0x%08x = 0x%08lX hashes (%ld.%06lds)", antrouter->drv->name, antrouter->device_fd, nonce, (long unsigned int)hash_count, elapsed.tv_sec, elapsed.tv_usec);

out:
	free_work(work);
	return hash_count;
}

static struct api_data *antrouter_api_stats(struct cgpu_info *cgpu)
{
	struct api_data *root = NULL;
	struct ANTROUTER_INFO *info = (struct ANTROUTER_INFO *)(cgpu->device_data);

	// Warning, access to these is not locked - but we don't really
	// care since hashing performance is way more important than
	// locking access to displaying API debug 'stats'
	// If locking becomes an issue for any of them, use copy_data=true also
	root = api_add_int(root, "read_time", &(info->read_time), false);
	root = api_add_int(root, "read_time_limit", &(info->read_time_limit), false);
	root = api_add_double(root, "fullnonce", &(info->fullnonce), false);
	root = api_add_int(root, "miner_count", &(info->count), false);
	root = api_add_hs(root, "Hs", &(info->Hs), false);
	root = api_add_double(root, "W", &(info->W), false);
	root = api_add_uint(root, "total_values", &(info->values), false);
	root = api_add_uint64(root, "range", &(info->hash_count_range), false);
	root = api_add_uint64(root, "history_count", &(info->history_count), false);
	root = api_add_timeval(root, "history_time", &(info->history_time), false);
	root = api_add_uint(root, "min_data_count", &(info->min_data_count), false);
	root = api_add_uint(root, "timing_values", &(info->history[0].values), false);
	root = api_add_const(root, "timing_mode", timing_mode_str(info->timing_mode), false);
	root = api_add_bool(root, "is_timing", &(info->do_antrouter_timing), false);
	root = api_add_int(root, "baud", &(info->baud), false);
	root = api_add_int(root, "work_division", &(info->work_division), false);
	root = api_add_int(root, "fpga_count", &(info->fpga_count), false);

	return root;
}

static void antrouter_statline_before(char *buf, size_t bufsiz, struct cgpu_info *cgpu)
{
	struct ANTROUTER_INFO *info = (struct ANTROUTER_INFO *)(cgpu->device_data);

	if (info->cmr2_speed > 0)
		tailsprintf(buf, bufsiz, "%5.1fMhz", (float)(info->cmr2_speed) * ANTROUTER_CMR2_SPEED_FACTOR);
	else
		tailsprintf(buf, bufsiz, "       ");

	tailsprintf(buf, bufsiz, "        | ");
}

static void antrouter_shutdown(__maybe_unused struct thr_info *thr)
{
	// TODO: ?
	do_antrouter_close(thr);
	applog(LOG_ERR,"closed device");
	free(thr->cgpu_data);
}

static void antrouter_identify(struct cgpu_info *cgpu)
{
	struct ANTROUTER_INFO *info = (struct ANTROUTER_INFO *)(cgpu->device_data);

//	if (info->ident == IDENT_CMR2)
		info->flash_next_work = true;
}

static char *antrouter_set(struct cgpu_info *cgpu, char *option, char *setting, char *replybuf)
{
	struct ANTROUTER_INFO *info = (struct ANTROUTER_INFO *)(cgpu->device_data);
	int val;

	if (strcasecmp(option, "help") == 0) {
		sprintf(replybuf, "clock: range %d-%d",
				  ANTROUTER_CMR2_SPEED_MIN_INT, ANTROUTER_CMR2_SPEED_MAX_INT);
		return replybuf;
	}

	if (strcasecmp(option, "clock") == 0) {
		if (!setting || !*setting) {
			sprintf(replybuf, "missing clock setting");
			return replybuf;
		}

		val = atoi(setting);
		if (val < ANTROUTER_CMR2_SPEED_MIN_INT || val > ANTROUTER_CMR2_SPEED_MAX_INT) {
			sprintf(replybuf, "invalid clock: '%s' valid range %d-%d",
					  setting,
					  ANTROUTER_CMR2_SPEED_MIN_INT,
					  ANTROUTER_CMR2_SPEED_MAX_INT);
		}

		info->cmr2_speed = CMR2_INT_TO_SPEED(val);
		info->speed_next_work = true;

		return NULL;
	}

	sprintf(replybuf, "Unknown option: %s", option);
	return replybuf;
}

struct device_drv antrouter_drv = {
	.drv_id = DRIVER_antrouter,
	.dname = "antrouter",
	.name = "ANTR1",
	.drv_detect = antrouter_detect,
	.hash_work = &hash_driver_work,
	.get_api_stats = antrouter_api_stats,
	.get_statline_before = antrouter_statline_before,
	.set_device = antrouter_set,
	.identify_device = antrouter_identify,
	.thread_prepare = antrouter_prepare,
	.scanwork = antrouter_scanwork,
	.thread_shutdown = antrouter_shutdown,
};
