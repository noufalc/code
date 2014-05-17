#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <time.h>
#include <sched.h>

#define PWM_FREQUENCY_13MHZ 13000000	// 13 MHz clock
#define PWM_FREQUENCY_32KHZ 32000		// 32 kHz clock
#define NANOSEC_PER_SEC 1000000000		// 1 sec = NANOSEC_PER_SEC nano seconds
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

typedef struct
{
	double dState; //Last position input
	double iState; //Integrator state
	double iMax, iMin; // Maximum and minimum allowable integrator state
	double iGain, pGain, dGain;
} SPid;

static inline void timeNormalize(struct timespec *ts) {
    while (ts->tv_nsec >= NANOSEC_PER_SEC) {
        ts->tv_nsec -= NANOSEC_PER_SEC;
        ts->tv_sec++;
    }
}


double final;
const double T = 0.005;
double t, y, yn1, err, errn1, errn2;
double updatePID(SPid * pid, double error, double position)
{
	double pTerm, dTerm, iTerm, pidSum;
	yn1 = y;
	errn2 = errn1;
	errn1 = err;
	err = error;
	y = yn1 + pid->pGain * err * T + ((pid->dGain / T) * (err + errn2 - 2 * errn1)); 
//	pTerm = pid->pGain * error;	// calculate the proportional term
//								// calculate the integral state with appropriate limiting
//	pid->iState += error;
//	//if (pid->iState > pid->iMax) pid->iState = pid->iMax;
//	//else if (pid->iState < pid->iMin) pid->iState = pid->iMin;
//	iTerm = pid->iGain * pid->iState; // calculate the integral term
//	dTerm = pid->dGain * (pid->dState - position);
//	pid->dState = position;
//	pidSum = pTerm + dTerm + iTerm;
	if (y > 95.0)
		y = 95.0;
	else if(y < 30.0)
		y = 30.0;
	return y;
}




unsigned int diffrerence, time1, time2, time1old, time2old;
float timePeriod;

void capture_config_timer(unsigned int *gpt)
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


int start_counter11()
{
	int dev_fd;
	unsigned int *PinConfig, *testValue;
	unsigned int CurValue;
	unsigned int *gpt11;
	unsigned long resolution;
	int i;

	dev_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (dev_fd == -1) 
	{
		printf("Can not opem /dev/mem.");
		return -1;
	}
	// set the clock source to 13MHz
    // comparing to GPIO/gpio.c, (unsigned int *) is added here
    // so it then goes into int space and the offset (@char) must be devided by 4 (@int)
	PinConfig = (unsigned int *) mmap(NULL, 0x300, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, CM_CLOCK_BASE);
	testValue = *(volatile unsigned int*)(PinConfig+CM_CLKSEL_CORE/4);
	CurValue = *(volatile unsigned int*)(PinConfig+CM_CLKSEL_CORE/4);
//	printf("The curvalue is CurValue %x\n", (unsigned int) testValue);
	CurValue &= 0xffffff7f;
	CurValue |= 0x80;	// set CLKSEL_GPT10 1,
	*(volatile unsigned int*)(PinConfig+CM_CLKSEL_CORE/4) = CurValue;
//	printf("13MHz clock source enabled........\n");

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
	gpt11=(unsigned int *) mmap(NULL, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED,dev_fd, 0x48088000);
	capture_config_timer(gpt11);
	munmap(gpt11, 0x10000);
	close(dev_fd);
	return 0;
}




double get_speed(double duty_cycle, double reftime)
{
	int dev_fd, value;
	FILE *fp;
	unsigned int *gpt11;
	struct timespec clock1;
	dev_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (dev_fd == -1) {
		printf("Can not opem /dev/mem.");
		return -1;
	}
	gpt11=(unsigned int *) mmap(NULL, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED,dev_fd, 0x48088000);
//	printf("The present value of the TCRR is %x\n", gpt11[GPT_REG_TCRR/4]);
//	printf("The present value of the TCAR1 is %x\n", gpt11[GPT_REG_TCAR1/4]);
//	printf("The present value of the TCAR2 is %x\n", gpt11[GPT_REG_TCAR2/4]);
//	printf("The present value of the TISR is %x\n", gpt11[GPT_REG_TISR/4]);
	value = gpt11[GPT_REG_TISR/4];
	if (value & 0x4)
	{
		diffrerence = (unsigned int) (gpt11[GPT_REG_TCAR2/4] - gpt11[GPT_REG_TCAR1/4]);
		time1old = time1;
		time2old = time2;
		gpt11[GPT_REG_TISR/4] = 0x6;
		timePeriod =  (float) 13000000 / (float) diffrerence;
	}
	fp = fopen("output.txt", "a");
//	gettimeofday(&clock1, NULL);
	clock_gettime(CLOCK_REALTIME, &clock1);
	fprintf(fp, "%lf,\t%lf,\t%lf,%lf;\n",((double)(clock1.tv_sec) - reftime) + (double)(clock1.tv_nsec) / 1000000000.0, duty_cycle, timePeriod * 30, final);
	fclose(fp);
//	printf("Frequency = %f rpm\n", timePeriod * 30);
	munmap(gpt11, 0x10000);
	close(dev_fd);
	return timePeriod * 30;
}




unsigned int pwm_calc_resolution(int pwm_frequency, int clock_frequency)
{
    float pwm_period = 1.0 / pwm_frequency;
    float clock_period = 1.0 / clock_frequency;
    return (unsigned int) (pwm_period / clock_period);
}

void pwm_config_timer(unsigned int *gpt, unsigned int resolution, float duty_cycle){
	duty_cycle = duty_cycle/100;
	unsigned long counter_start = 0xffffffff - resolution;
	unsigned long dc = 0xffffffff - ((unsigned long) (resolution * duty_cycle));
	
    // Edge condition: the duty cycle is set within two units of the overflow
    // value.  Loading the register with this value shouldn't be done (TRM 16.2.4.6).
    if (0xffffffff - dc <= 2) {
        dc = 0xffffffff - 2;
    }
	//printf("%x\n", dc);
    // Edge condition: TMAR will be set to within two units of the overflow
    // value.  This means that the resolution is extremely low, which doesn't
    // really make sense, but whatever.
    if (0xffffffff - counter_start <= 2) {
        counter_start = 0xffffffff - 2;
    }

    // GPT_REG_TCLR/4: in int space
    gpt[GPT_REG_TCLR/4] = 0; // Turn off
    gpt[GPT_REG_TCRR/4] = counter_start;
    gpt[GPT_REG_TLDR/4] = counter_start;
    gpt[GPT_REG_TMAR/4] = dc;
    gpt[GPT_REG_TCLR/4] = (
        (1 << 0)  | // ST -- enable counter
        (1 << 1)  | // AR -- autoreload on overflow
        (1 << 6)  | // CE -- compare enabled
        (0 << 7)  | // SCPWM -- invert pulse
        (2 << 10) | // TRG -- overflow and match trigger
        (1 << 12)   // PT -- toggle PWM mode
    );
}

int start_pwm(int frequency, int duty_cycle){
	int dev_fd;
	unsigned int *PinConfig, *testValue;
	unsigned int CurValue;
	unsigned int *gpt10;
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
	testValue = (volatile unsigned int*)(PinConfig+CM_CLKSEL_CORE/4);
	CurValue = *(volatile unsigned int*)(PinConfig+CM_CLKSEL_CORE/4);
//	printf("The curvalue is CurValue %x\n", testValue);
	CurValue &= 0xffffffbf;
	CurValue |= 0x40;	// set CLKSEL_GPT10 1,
	*(volatile unsigned int*)(PinConfig+CM_CLKSEL_CORE/4) = CurValue;
//	printf("13MHz clock source enabled........\n");

	// enable the clock: FCLK and ICLK
	PinConfig = (unsigned int *) mmap(NULL, 0x300, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, CM_CLOCK_BASE);
	CurValue = *(volatile unsigned int*)(PinConfig+CM_FCLKEN1_CORE/4);
	CurValue &= 0xfffff7ff;
	CurValue |= 0x800;	// set EN_GPT10 1, GPTIMER 10 functional clock is enabled
	*(volatile unsigned int*)(PinConfig+CM_FCLKEN1_CORE/4) = CurValue;
	//printf("FCLOCK enabled........\n");

	CurValue = *(volatile unsigned int*)(PinConfig+CM_ICLKEN1_CORE/4);
	CurValue &= 0xfffff7ff;
	CurValue |= 0x800;	// set EN_GPT10 1, GPTIMER 10 interface clock is enabled
	*(volatile unsigned int*)(PinConfig+CM_ICLKEN1_CORE/4) = CurValue;
	//printf("ICLOCK enabled.........\n");
	munmap(PinConfig, 0x1000);

	// System control module: 0x4800 2000, found via devmem2
	PinConfig=(unsigned int *) mmap(NULL, 0x200, PROT_READ | PROT_WRITE, MAP_SHARED,dev_fd, 0x48002000);
	// Set PWM function on pin: GPIO_56, EMA-product-board GPMC_nCS5, DM3730 spec page 2428
	// division by 4 is necessary because the size of one element of "unsigned int" is 4 bytes, which corresponds to the size of control registers
	CurValue=*(volatile unsigned int*)(PinConfig+0x174/4); 
	CurValue &= 0x0000ffff;
	/* Timer 10: mode 2 - gpt_10_pwm_evt, PullUp selected, PullUp/Down enabled, Input enable value for pad_x */
	CurValue |= 0x011A0000; 
	*(volatile unsigned int*)(PinConfig+0x174/4) = CurValue; //PIN CONFIGURED AS BIDIRECTIONAL

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
	gpt10=(unsigned int *) mmap(NULL, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED,dev_fd, 0x48086000);
	resolution = pwm_calc_resolution(frequency, PWM_FREQUENCY_13MHZ);
	pwm_config_timer(gpt10, resolution, duty_cycle);
	munmap(gpt10, 0x10000);
	close(dev_fd);
	return 0;
}
double output;
double present_referecne;


double lpf(double start, double final)
{
        output += 0.01 * (final - start);
        present_referecne = output;
        return output;
}



int main(int argc, char *argv[])
{
	int d = 193000, i, j, interval;
//	struct timeval clock;
	struct timespec t, clock;
	struct sched_param param;
	int priority = 90;
	SPid fanControl;
	fanControl.pGain = 0.3; //Kc, Kc/Ti 
	fanControl.iGain = 0.34;
	fanControl.dGain = 0.0;
	fanControl.iMax = 0.0;
	fanControl.iMin = 0.0;
	fanControl.iState  = 0;
	double command_speed = 0, present_speed = 0, new_duty = 50, initial;

   	printf("Using realtime, priority: %d.\n", priority);
   	param.sched_priority = priority;
   	// Enable realtime fifo scheduling.
   	if(sched_setscheduler(0, SCHED_FIFO, &param)==-1)
	{
   		perror("Error: sched_setscheduler failed.");
   	    exit(-1);
   	}
	printf("Enter 1 to start and 0 to exit:-");
	scanf("%d", &i);
	if (i == 1)
	{
		start_counter11();
	}
	start_pwm(25000, new_duty);
	while(d--)
		printf("%d\n", d);
//	gettimeofday(&clock, NULL);
	clock_gettime(CLOCK_REALTIME, &clock);
	initial = get_speed(new_duty, (double)(clock.tv_sec));
	printf("Kp = %lf, Ki = %lf, Kd  = %lf\n", fanControl.pGain, fanControl.iGain, fanControl.dGain);
	printf("start speed = %lf\n", initial);
	final = initial;
	final += 40;
	printf("Set speed = %lf\n", final);
//	output = initial;
//	present_referecne = initial;
	j = 15000;
	clock_gettime(0, &t);
	t.tv_sec++;
	interval = (int) NANOSEC_PER_SEC * T;
	while (j--)
	{
//		present_referecne  = lpf(present_referecne, final);
		clock_nanosleep(0, TIMER_ABSTIME, &t, NULL);	
		new_duty = updatePID(&fanControl, final - present_speed, present_speed);
		start_pwm(25000, new_duty);
		present_speed = get_speed(new_duty, (double)(clock.tv_sec));
		t.tv_nsec += interval;
		timeNormalize(&t);
	}
	return 0;
}
