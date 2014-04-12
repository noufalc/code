#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

// 13 MHz clock
#define PWM_FREQUENCY_13MHZ 13000000
// 32 kHz clock
#define PWM_FREQUENCY_32KHZ 32000

// Timer register
#define GPT_REG_TCLR 0x024  // in byte space
#define GPT_REG_TCRR 0x028
#define GPT_REG_TLDR 0x02c
#define GPT_REG_TMAR 0x038
#define GPT_REG_TCAR1 0x03c
#define GPT_REG_TCAR2 0x044
#define GPT_REG_TISR 0x018

// clock register of GPTIMER10
#define CM_CLOCK_BASE 0x48004000
#define CM_FCLKEN1_CORE 0xa00
#define CM_ICLKEN1_CORE 0xa10
#define CM_CLKSEL_CORE 0xa40

unsigned int diffrerence, time1, time2, time1old, time2old;
float timePeriod;

void pwm_config_timer(unsigned int *gpt)
{
    gpt[GPT_REG_TCLR/4] = 0; // Turn off
    gpt[GPT_REG_TCRR/4] = 0xf;
    gpt[GPT_REG_TLDR/4] = 0x1;
    gpt[GPT_REG_TCLR/4] = (
        (1 << 0)  | // ST -- enable counter
        (1 << 1)  | // AR -- autoreload on overflow
        (1 << 8) | // TRG -- overflow and match trigger
        (1 << 13) |   // PT -- toggle PWM mode
        (1 << 14)   // PT -- toggle PWM mode
    );
}

int start_counter11(){
	int dev_fd;
	unsigned int *PinConfig, *testValue;
	unsigned int CurValue;
	unsigned int *gpt11;
	unsigned long resolution;
	int i;

	dev_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (dev_fd == -1) {
		printf("Can not opem /dev/mem.");
		return -1;
	}
	// set the clock source to 13MHz
    // comparing to GPIO/gpio.c, (unsigned int *) is added here
    // so it then goes into int space and the offset (@char) must be devided by 4 (@int)
	PinConfig = (unsigned int *) mmap(NULL, 0x300, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, CM_CLOCK_BASE);
	testValue = *(volatile unsigned int*)(PinConfig+CM_CLKSEL_CORE/4);
	CurValue = *(volatile unsigned int*)(PinConfig+CM_CLKSEL_CORE/4);
	printf("The curvalue is CurValue %x\n", (unsigned int) testValue);
	CurValue &= 0xffffff7f;
	CurValue |= 0x80;	// set CLKSEL_GPT10 1,
	*(volatile unsigned int*)(PinConfig+CM_CLKSEL_CORE/4) = CurValue;
	printf("13MHz clock source enabled........\n");

	// enable the clock: FCLK and ICLK
	PinConfig = (unsigned int *) mmap(NULL, 0x300, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, CM_CLOCK_BASE);
	CurValue = *(volatile unsigned int*)(PinConfig+CM_FCLKEN1_CORE/4);
	CurValue &= 0xffffefff;
	CurValue |= 0x1000;	// set EN_GPT10 1, GPTIMER 10 functional clock is enabled
	*(volatile unsigned int*)(PinConfig+CM_FCLKEN1_CORE/4) = CurValue;
	printf("FCLOCK enabled........\n");

	CurValue = *(volatile unsigned int*)(PinConfig+CM_ICLKEN1_CORE/4);
	CurValue &= 0xffffefff;
	CurValue |= 0x1000;	// set EN_GPT10 1, GPTIMER 10 interface clock is enabled
	*(volatile unsigned int*)(PinConfig+CM_ICLKEN1_CORE/4) = CurValue;
	printf("ICLOCK enabled.........\n");
	munmap(PinConfig, 0x1000);

	// System control module: 0x4800 2000, found via devmem2
	PinConfig=(unsigned int *) mmap(NULL, 0x200, PROT_READ | PROT_WRITE, MAP_SHARED,dev_fd, 0x48002000);
	// Set PWM function on pin: GPIO_56, EMA-product-board GPMC_nCS5, DM3730 spec page 2428
	// division by 4 is necessary because the size of one element of "unsigned int" is 4 bytes, which corresponds to the size of control registers
	CurValue=*(volatile unsigned int*)(PinConfig+0x178/4); 
	CurValue &= 0xffff0000;
	/* Timer 10: mode 2 - gpt_10_pwm_evt, PullUp selected, PullUp/Down enabled, Input enable value for pad_x */
	CurValue |= 0x0000011A; 
	*(volatile unsigned int*)(PinConfig+0x178/4) = CurValue; //PIN CONFIGURED AS BIDIRECTIONAL
	munmap(PinConfig, 0x200);

	// Configure timer 10 and 11 to 13 MHz
	// Config=(unsigned int *) mmap(NULL, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED,dev_fd, 0x48004000);
	// CurValue=Config[0xA40/4];
	// CurValue=CurValue | (1<<6) | (1<<7);
	// CurValue |= (1<<6);//Configure 10 timer to 13 MHz
	// CurValue |= (1<<7);//Configure 11 timer to 13 MHz
	// Config[0xA40/4]=CurValue;

	// GPTIMER10 base address: 0x48086000(in byte space)
        // gpt10: in int space
	gpt11=(unsigned int *) mmap(NULL, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED,dev_fd, 0x48088000);
	pwm_config_timer(gpt11);
	munmap(gpt11, 0x10000);
	close(dev_fd);
	return 0;
}



int print_countervalue()
{
	int dev_fd, value;
	unsigned int *gpt11;
	dev_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (dev_fd == -1) {
		printf("Can not opem /dev/mem.");
		return -1;
	}
	gpt11=(unsigned int *) mmap(NULL, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED,dev_fd, 0x48088000);
	printf("The present value of the TCRR is %x\n", gpt11[GPT_REG_TCRR/4]);
	printf("The present value of the TCAR1 is %x\n", gpt11[GPT_REG_TCAR1/4]);
	printf("The present value of the TCAR2 is %x\n", gpt11[GPT_REG_TCAR2/4]);
	printf("The present value of the TISR is %x\n", gpt11[GPT_REG_TISR/4]);
	value = gpt11[GPT_REG_TISR/4];
	if (value & 0x4)
	{
		diffrerence = (unsigned int) (gpt11[GPT_REG_TCAR2/4] - gpt11[GPT_REG_TCAR1/4]);
		time1old = time1;
		time2old = time2;
		gpt11[GPT_REG_TISR/4] = 0x6;
		timePeriod =  (float) 13000000 / (float) diffrerence;
	}
	printf("Speed = %f rpm\n", timePeriod * 30);
	munmap(gpt11, 0x10000);
	close(dev_fd);
	return 0;
}

int main(int argc, char *argv[])
{
	int i;
	printf("Enter 1 to start and 0 to exit:-");
	scanf("%d", &i);
	if (i == 1)
	{
		start_counter11();
	}
	else
	{
		return 0;
	}
	while(1)
	{
		usleep(1000);
		print_countervalue();
	}
	return 0;
}
