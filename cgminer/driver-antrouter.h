#ifndef __DRIVER_ANTROUTER_H__
#define __DRIVER_ANTROUTER_H__


#ifndef BUFSIZ
#define BUFSIZ 512
#endif
#define TEST_MODE 0
#define ANTROUTER_POOL "stratum+tcp://solo.antpool.com:443"

#define BITMAIN_MIN_FREQUENCY 100
#define BITMAIN_MAX_FREQUENCY 800
/*
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
*/
//Command Description
#define CMD_ALL				(0x01 << 4)
#define SET_ADDR 			0x1
#define SET_PLL_DIV2		0x2
#define PATTERN_CTRL		0x3
#define GET_STATUS			0x4
#define CHAIN_INACTIVE		0x5
#define SET_BAUDOPS			0x6
#define SET_PLL_DIV1		0x7
#define	SET_CONFIG			0x8

#define CMD_LENTH			0x5
#define CONFIG_LENTH		0x9
//Register description

#define  CHIP_ADDR			0x00
#define  HASHRATE			0x08
#define  PLL_PARAMETER		0x0C
#define  SNO				0x10
#define  HCN				0x14
#define  TICKET_MASK		0x18
#define  MISC_CONTROL		0x1C
#define  GENERAL_IIC		0x20
#define  SECURITY_IIC		0x24
#define  SIG_INPUT			0x28
#define  SIG_NONCE			0x2C
#define  SIG_ID				0x30
#define  SEC_CTRL_STATUS  	0x34
#define  JOB_INFO			0x38

//Register bits value
#define CMD_TYPE			(0x2 << 5)
#define HASHRATE_TYPE		(0x0 << 7)
#define	HASHRATE_CTRL1(X)	(X & 0x07)
#define HASHRATE_CTRL2(X)	((X & 0x07) << 4)
#define PAT					(0x0 << 7)
#define INV_CLKO			(0x1 << 5)
#define GATEBCLK			0x1 << 7
#define RFS					(0x0 << 6)
#define BT8D				0x1A
#define MMEN				(0x1 << 7)
#define TFS(X)				((X & 0x03) << 5)
#define LCM(X)				(X & 0x0F)
#define IIC_BUSY			(0x1 << 7)
#define IIC_RW_FAIL			(0x1 << 6)
#define REGADDR_VALID		0x1
#define DEVICE_ADDR(X)		(X & 0x7F) << 1)
#define IIC_READ			0x0  				//0: READ  1: WRITE
#define EPROM_ADDR 			0x50 
#define SIG_CNT(X)			(X & 0x0F) << 4)
#define ECC_CLKEN			(0x1 << 3)
#define SIG_PASS			(0x1 << 1)
#define DISA_CHIP			0x0

#define NONCE_RESPOND		(0x1 << 7)
#define SIG_BIT				(0x1 << 6)
#define NONCE_BIT			(0x1 << 7)

struct freq_pll
{
	const char *freq;
	unsigned int fildiv1;
	unsigned int fildiv2;
	unsigned int vilpll;
};
static struct freq_pll freq_pll_1387[] = {
	{"100",0x020040, 0x0420, 0x200241},
	{"125",0x028040, 0x0420, 0x280241},
	{"150",0x030040, 0x0420, 0x300241},
	{"175",0x038040, 0x0420, 0x380241},
	{"200",0x040040, 0x0420, 0x400241},
	{"225",0x048040, 0x0420, 0x480241},
	{"250",0x050040, 0x0420, 0x500241},
	{"275",0x058040, 0x0420, 0x580241},
	{"300",0x060040, 0x0420, 0x600241},
	{"325",0x068040, 0x0420, 0x680241},
	{"350",0x070040, 0x0420, 0x700241},
	{"375",0x078040, 0x0420, 0x780241},
	{"400",0x080040, 0x0420, 0x800241},
	{"404",0x061040, 0x0320, 0x610231},
	{"406",0x041040, 0x0220, 0x410221},
	{"408",0x062040, 0x0320, 0x620231},
	{"412",0x042040, 0x0220, 0x420221},
	{"416",0x064040, 0x0320, 0x640231},
	{"418",0x043040, 0x0220, 0x430221},
	{"420",0x065040, 0x0320, 0x650231},
	{"425",0x044040, 0x0220, 0x440221},
	{"429",0x067040, 0x0320, 0x670231},
	{"431",0x045040, 0x0220, 0x450221},
	{"433",0x068040, 0x0320, 0x680231},
	{"437",0x046040, 0x0220, 0x460221},
	{"441",0x06a040, 0x0320, 0x6a0231},
	{"443",0x047040, 0x0220, 0x470221},
	{"445",0x06b040, 0x0320, 0x6b0231},
	{"450",0x048040, 0x0220, 0x480221},
	{"454",0x06d040, 0x0320, 0x6d0231},
	{"456",0x049040, 0x0220, 0x490221},
	{"458",0x06e040, 0x0320, 0x6e0231},
	{"462",0x04a040, 0x0220, 0x4a0221},
	{"466",0x070040, 0x0320, 0x700231},
	{"468",0x04b040, 0x0220, 0x4b0221},
	{"470",0x071040, 0x0320, 0x710231},
	{"475",0x04c040, 0x0220, 0x4c0221},
	{"479",0x073040, 0x0320, 0x730231},
	{"481",0x04d040, 0x0220, 0x4d0221},
	{"483",0x074040, 0x0320, 0x740231},
	{"487",0x04e040, 0x0220, 0x4e0221},
	{"491",0x076040, 0x0320, 0x760231},
	{"493",0x04f040, 0x0220, 0x4f0221},
	{"495",0x077040, 0x0320, 0x770231},
	{"500",0x050040, 0x0220, 0x500221},
	{"504",0x079040, 0x0320, 0x790231},
	{"506",0x051040, 0x0220, 0x510221},
	{"508",0x07a040, 0x0320, 0x7a0231},
	{"512",0x052040, 0x0220, 0x520221},
	{"516",0x07c040, 0x0320, 0x7c0231},
	{"518",0x053040, 0x0220, 0x530221},
	{"520",0x07d040, 0x0320, 0x7d0231},
	{"525",0x054040, 0x0220, 0x540221},
	{"529",0x07f040, 0x0320, 0x7f0231},
	{"531",0x055040, 0x0220, 0x550221},
	{"533",0x080040, 0x0320, 0x800231},
	{"537",0x056040, 0x0220, 0x560221},
	{"543",0x057040, 0x0220, 0x570221},
	{"550",0x058040, 0x0220, 0x580221},
	{"556",0x059040, 0x0220, 0x590221},
	{"562",0x05a040, 0x0220, 0x5a0221},
	{"568",0x05b040, 0x0220, 0x5b0221},
	{"575",0x05c040, 0x0220, 0x5c0221},
	{"581",0x05d040, 0x0220, 0x5d0221},
	{"587",0x05e040, 0x0220, 0x5e0221},
	{"593",0x05f040, 0x0220, 0x5f0221},
	{"600",0x060040, 0x0220, 0x600221},
	{"606",0x061040, 0x0220, 0x610221},
	{"612",0x062040, 0x0220, 0x620221},
	{"618",0x063040, 0x0220, 0x630221},
	{"625",0x064040, 0x0220, 0x640221},
	{"631",0x065040, 0x0220, 0x650221},
	{"637",0x066040, 0x0220, 0x660221},
	{"643",0x067040, 0x0220, 0x670221},
	{"650",0x068040, 0x0220, 0x680221},
	{"656",0x069040, 0x0220, 0x690221},
	{"662",0x06a040, 0x0220, 0x6a0221},
	{"668",0x06b040, 0x0220, 0x6b0221},
	{"675",0x06c040, 0x0220, 0x6c0221},
	{"681",0x06d040, 0x0220, 0x6d0221},
	{"687",0x06e040, 0x0220, 0x6e0221},
	{"693",0x06f040, 0x0220, 0x6f0221},
	{"700",0x070040, 0x0220, 0x700221},
	{"706",0x071040, 0x0220, 0x710221},
	{"712",0x072040, 0x0220, 0x720221},
	{"718",0x073040, 0x0220, 0x730221},
	{"725",0x074040, 0x0220, 0x740221},
	{"731",0x075040, 0x0220, 0x750221},
	{"737",0x076040, 0x0220, 0x760221},
	{"743",0x077040, 0x0220, 0x770221},
	{"750",0x078040, 0x0220, 0x780221},
	{"756",0x079040, 0x0220, 0x790221},
	{"762",0x07a040, 0x0220, 0x7a0221},
	{"768",0x07b040, 0x0220, 0x7b0221},
	{"775",0x07c040, 0x0220, 0x7c0221},
	{"781",0x07d040, 0x0220, 0x7d0221},
	{"787",0x07e040, 0x0220, 0x7e0221},
	{"793",0x07f040, 0x0220, 0x7f0221},
	{"800",0x080040, 0x0220, 0x800221},
};
static void get_plldata(int type,int freq,uint8_t * reg_data_pll,uint8_t * reg_data_pll2,uint8_t * reg_data_vil)
{
	uint32_t i;
	char freq_str[10],origin_pll_data[10];
	char plldivider1[10], plldivider2[10];
	
	sprintf(freq_str,"%d", freq);
	applog(LOG_ERR, "freq_str: %s", freq_str);
	if(type == 1387)
	{
		for(i=0; i < sizeof(freq_pll_1387)/sizeof(freq_pll_1387[0]); i++)
		{
			if( memcmp(freq_pll_1387[i].freq, freq_str, sizeof(freq_pll_1387[i].freq)) == 0)
				break;
		}
	}
	if( i == sizeof(freq_pll_1387)/sizeof(freq_pll_1387[0]))
	{
		applog(LOG_ERR, "freq set error");
		quit(1, "Failed to get_plldata");
	}

	sprintf(plldivider1, "%08x", freq_pll_1387[i].fildiv1);
	sprintf(plldivider2, "%04x", freq_pll_1387[i].fildiv2);
	applog(LOG_ERR, "plldivider1: %s, plldivider2: %s", plldivider1, plldivider2);
	if(!hex2bin(reg_data_pll, plldivider1, strlen(plldivider1)/2)) {
		quit(1, "Invalid fil plldata for reg data, hex2bin error now: %s",
							plldivider1);
	}

	if(!hex2bin(reg_data_pll2, plldivider2, 2)) {
		quit(1, "Invalid fil plldata for reg data, hex2bin error now: %s",
					plldivider2);
	}
	sprintf(origin_pll_data, "%08x", freq_pll_1387[i].vilpll);
	applog(LOG_ERR, "origin_pll_data:%s", origin_pll_data);
	if(!hex2bin(reg_data_vil, origin_pll_data, strlen(origin_pll_data)/2)) {
		quit(1, "Invalid vil plldata for reg data, hex2bin error now: %s",
					origin_pll_data);
	}
	return;
}

#endif
