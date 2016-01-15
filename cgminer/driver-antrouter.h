#ifndef __DRIVER_ANTROUTER_H__
#define __DRIVER_ANTROUTER_H__


#ifndef BUFSIZ
#define BUFSIZ 512
#endif
#define TEST_MODE 0
#define ANTROUTER_POOL "stratum+tcp://solo.antpool.com:443"

#define BITMAIN_MIN_FREQUENCY 100
#define BITMAIN_MAX_FREQUENCY 500
#define return_via(label, stmt)  do {  \
	stmt;  \
	goto label;  \
} while (0)

extern void (*timer_set_now)(struct timeval *);
#define cgtime(tvp)  timer_set_now(tvp)

#define timer_set_delay(tvp_timer, tvp_now, usecs)  do {  \
	struct timeval tv_add = TIMEVAL_USECS(usecs);  \
	timeradd(&tv_add, tvp_now, tvp_timer);  \
} while(0)

#define TIMEVAL_USECS(usecs)  (  \
	(struct timeval){  \
		.tv_sec = (usecs) / 1000000,  \
		.tv_usec = (usecs) % 1000000,  \
	}  \
)

#define ANT_GETS_ERROR -1
#define ANT_GETS_OK 0
#define ANT_GETS_RESTART 1
#define ANT_GETS_TIMEOUT 2
static inline
const struct timeval *_ant_nullisnow(const struct timeval *tvp, struct timeval *tvp_buf)
{
	if (tvp)
		return tvp;
	cgtime(tvp_buf);
	return tvp_buf;
}

static inline
long timeval_to_us(const struct timeval *tvp)
{
	return ((long)tvp->tv_sec * 1000000) + tvp->tv_usec;
}

static inline
long timer_elapsed_us(const struct timeval *tvp_timer, const struct timeval *tvp_now)
{
	struct timeval tv;
	const struct timeval *_tvp_now = _ant_nullisnow(tvp_now, &tv);
	timersub(_tvp_now, tvp_timer, &tv);
	return timeval_to_us(&tv);
}

#ifndef timersub
    #define timersub(a, b, result)                     \
    do {                                               \
      (result)->tv_sec = (a)->tv_sec - (b)->tv_sec;    \
      (result)->tv_usec = (a)->tv_usec - (b)->tv_usec; \
      if ((result)->tv_usec < 0) {                     \
        --(result)->tv_sec;                            \
        (result)->tv_usec += 1000000;                  \
      }                                                \
    } while (0)
#endif

ssize_t _serial_read(int fd, char *buf, size_t buflen, char *eol);
#define serial_read(fd, buf, count)  \
	_serial_read(fd, (char*)(buf), count, NULL)
#define antrouter_close(fd) serial_close(fd)

extern void antrouter_detect(bool);

static inline
long timer_remaining_us(const struct timeval *tvp_timer, const struct timeval *tvp_now)
{
	struct timeval tv;
	const struct timeval *_tvp_now = _ant_nullisnow(tvp_now, &tv);
	timersub(tvp_timer, _tvp_now, &tv);
	return timeval_to_us(&tv);
}

#endif
