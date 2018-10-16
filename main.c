// RTOS Framework - Fall 2018
// J Losh

// Student Name: YETISH KRISHNA REDDY
// TO DO: Add your name on this line.  Do not include your ID number.

// Submit only two .c files in an e-mail to me (not in a compressed file):
// 09_rtos.c   Single-file with your project code
// (xx is a unique number that will be issued in class)
// Please do not include .intvecs section in your code submissions
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 5 Pushbuttons and 5 LEDs, UART

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "tm4c123gh6pm.h"

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4))) // off-board red LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4))) // off-board orange LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4))) // off-board yellow LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4))) // off-board green LED
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED

#define PB1            (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))
#define PB2            (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))
#define PB3            (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))
#define PB4            (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))
#define PB5			   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 7*4)))

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
struct semaphore
{
	uint16_t count;
	uint16_t queueSize;
	uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;
char* name_sema[] = { "KeyPressed", "keyReleased", "flashReq", "resource" };
char* name_task[] = { "idle", "lengthyfn", "flash4hz", "oneshot", "readkeys",
						"debounce", "important", "uncoop", "shell" };

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_BLOCKED    3 // has run, but now blocked by semaphore
#define STATE_DELAYED    4 // has run, but now awaiting timer

#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks
uint32_t systemSP;
//uint32_t r0, r2;
//uint32_t *r1;
uint8_t light = 0;
bool flag = true;
uint8_t count = 0;
struct semaphore *pSemaphore;
char ch;
char str[81];
uint8_t MaxChar = 81;
uint8_t pi = 1;
uint8_t pre = 1;
char store[20];
uint32_t tas = 0;
uint32_t tas1;
uint32_t tbs = 0;
uint32_t totaltime = 0;
uint8_t prioinhen = 1;
uint8_t first = 0;
uint8_t first1 = 0;
uint32_t onesec = 0;
uint32_t one1sec = 0;
uint32_t onesecdemo = 0;
uint16_t tempid[9];
uint8_t inhen1;
uint64_t total;

struct _tcb
{
	uint8_t state;                 // see STATE_ values above
	void *pid;                     // used to uniquely identify thread
	void *sp;                      // location of stack pointer for thread
	uint8_t priority;              // 0=highest, 15=lowest
	uint8_t currentPriority;       // used for priority inheritance
	uint64_t ticks;                // ticks until sleep complete
	char name[16];                 // name of task used in ps command
	void *semaphore;     // pointer to the semaphore that is blocking the thread
	uint8_t skipcount;
	uint64_t processTime;
	uint64_t taskTime;
	uint64_t cpu_usage_percentage;
	uint64_t S;
} tcb[MAX_TASKS];

uint8_t ii;

uint32_t stack[MAX_TASKS][256];  // 1024 byte stack for each thread

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

void rtosInit()
{
	uint8_t i;
	// no tasks running
	taskCount = 0;
	// clear out tcb records
	for (i = 0; i < MAX_TASKS; i++)
	{
		tcb[i].state = STATE_INVALID;
		tcb[i].pid = 0;
	}
	// REQUIRED: initialize systick for 1ms system timer
	NVIC_ST_CTRL_R = NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN
			| NVIC_ST_CTRL_ENABLE;
	//turnonSystick();
}

void Setsp(uint32_t SPvalue)
{
	__asm("     ADD SP, #8");
	__asm("     MOV   SP, R0 ");
	__asm("     SUB SP, #8");
	//__asm("     BX    LR     ");
}

uint32_t Getsp()
{
	__asm("     MOV  R0, SP" );
	__asm("     BX    LR     ");
}

void rtosStart()
{
	// REQUIRED: add code to call the first task to be run
	turnonSystick();
	_fn fn;
	taskCurrent = rtosScheduler();
	systemSP = Getsp();
	Setsp((uint32_t) tcb[taskCurrent].sp);
	tcb[taskCurrent].state = STATE_READY;
	fn = (_fn) tcb[taskCurrent].pid;
	(*fn)();
	// Add code to initialize the SP with tcb[task_current].sp;
}

bool createThread(_fn fn, char name[], uint32_t priority)
{
	__asm (" 		SVC #0x06");
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{
	__asm(" 		SVC #0x05");
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
	uint8_t x;
	for (x = 0; x < MAX_TASKS; x++)
	{
		if (tcb[x].pid == fn)
		{
			tcb[x].priority = priority;
			tcb[x].currentPriority = priority;
		}
	}

}

struct semaphore* createSemaphore(uint8_t count)
{
	struct semaphore *pSemaphore = 0;
	if (semaphoreCount < MAX_SEMAPHORES)
	{
		pSemaphore = &semaphores[semaphoreCount++];
		pSemaphore->count = count;
	}
	return pSemaphore;
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
void yield()
{
	__asm("    SVC #0x01");
	// push registers, call scheduler, pop registers, return to new function

}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t tick)
{
	__asm("    SVC #0x02");

	// push registers, set state to delayed, store timeout, call scheduler, pop registers,
	// return to new function (separate unrun or ready processing)
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
	__asm("    SVC #0x03");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
	__asm("    SVC #0x04");
}

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{
	bool ok;
	static uint8_t task = 0xFF;
	ok = false;
	while (!ok)
	{
		task++;
		if (task >= MAX_TASKS)
			task = 0;
		if (pi == 0)
		{
			ok = (tcb[task].state == STATE_READY
					|| tcb[task].state == STATE_UNRUN);
		}
		if (pi == 1)
		{
			if (tcb[task].skipcount >= tcb[task].currentPriority)
			{
				ok = (tcb[task].state == STATE_READY
						|| tcb[task].state == STATE_UNRUN);
				tcb[task].skipcount = 0;
			}
			else
			{
				tcb[task].skipcount++;
			}
		}
	}
	return task;
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
	uint32_t i, alpha = 0.99;
	for (i = 0; i < MAX_TASKS; i++)
	{
		if ((tcb[i].ticks > 0) && (tcb[i].state == STATE_DELAYED))
		{
			tcb[i].ticks--;
			if (tcb[i].ticks == 0)
				tcb[i].state = STATE_READY;
		}
	}

	if (onesec == 100)
	{
		tas = 0;
		tbs = 0;
		first = 0;

		for (i = 0; i < MAX_TASKS; i++)
		{
			total = total + tcb[i].taskTime;
		}
		for (i = 0; i < MAX_TASKS; i++)
		{
			tcb[i].S = tcb[i].taskTime * 10000 / total;
			tcb[i].cpu_usage_percentage = tcb[i].cpu_usage_percentage * 0.9
					+ 0.1 * tcb[i].S;
		}
		onesec = 0;
		for (i = 0; i < MAX_TASKS; i++)
		{
			tcb[i].S = 0;
			tcb[i].taskTime;
			total = 0;
			first = 0;
			total = 0;
		}
		//totaltime = 0;
	}
	onesec++;
	if (pre == 1)
	{
		tcb[taskCurrent].state = STATE_READY;
		NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
	}
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
	__asm("     PUSH {R4}" );
	__asm("     PUSH {R5}" );
	__asm("     PUSH {R6}" );
	__asm("     PUSH {R7}" );
	__asm("     PUSH {R8}" );
	__asm("     PUSH {R9}" );
	__asm("     PUSH {R10}" );
	__asm("     PUSH {R11}" );

	tcb[taskCurrent].sp = (void*) Getsp();
	Setsp(systemSP);

	if (first == 0)
	{
		first++;
	}
	else
	{
		tbs = TIMER1_TAV_R;
		if (tbs > tas)
		{
			tcb[taskCurrent].processTime = tbs - tas;
			tcb[taskCurrent].taskTime = tcb[taskCurrent].taskTime
					+ tcb[taskCurrent].processTime;
			totaltime = totaltime + tcb[taskCurrent].processTime;
		}
	}

	taskCurrent = rtosScheduler();

	tas = TIMER1_TAV_R;
	tas1 = TIMER1_TAV_R;

	if (tcb[taskCurrent].state == STATE_UNRUN)
	{
		Setsp((uint32_t) tcb[taskCurrent].sp);
		stack[taskCurrent][255] = 0x01000000;
		stack[taskCurrent][254] = (uint32_t) tcb[taskCurrent].pid;
		stack[taskCurrent][253] = 15;
		stack[taskCurrent][252] = 14;
		stack[taskCurrent][251] = 13;
		stack[taskCurrent][250] = 12;
		stack[taskCurrent][249] = 11;
		stack[taskCurrent][248] = 10;
		stack[taskCurrent][247] = 0xfffffff9;
		stack[taskCurrent][246] = 9;
		stack[taskCurrent][245] = 8;
		stack[taskCurrent][244] = 7;
		stack[taskCurrent][243] = 6;
		stack[taskCurrent][242] = 5;
		stack[taskCurrent][241] = 4;
		stack[taskCurrent][240] = 3;
		stack[taskCurrent][239] = 2;
		stack[taskCurrent][238] = 1;
		//stack[taskCurrent][237] = 1;
		//stack[taskCurrent][236] = 1;

		tcb[taskCurrent].sp = &stack[taskCurrent][238];
		tcb[taskCurrent].state = STATE_READY;
	}
	if (tcb[taskCurrent].state == STATE_READY)
	{
		Setsp((uint32_t) tcb[taskCurrent].sp);
		__asm("     POP {R11}" );
		__asm("     POP {R10}" );
		__asm("     POP {R9}" );
		__asm("     POP {R8}" );
		__asm("     POP {R7}" );
		__asm("     POP {R6}" );
		__asm("     POP {R5}" );
		__asm("     POP {R4}" );
	}

}
// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives

uint8_t getsvval()
{

}

uint32_t getR0()
{
}

char* getR1()
{
	__asm("    MOV R0, R1");
}

uint32_t getR2()
{
	__asm("    MOV R0, R2");
}

uint8_t sv;
void svCallIsr()
{
	uint32_t r0 = getR0();
	char* r1 = getR1();
	uint32_t r2 = getR2();

	uint8_t p, j, x, i, o;

	__asm("    MOV R0, SP");
	__asm("    LDR R0, [R0, #64]");
	__asm("    LDRB R0, [R0, #-2]");
	sv = getsvval();

	switch (sv)
	{
	case 1:	//Idle------------------------------------------------------------------------------------------------
	{
		tcb[taskCurrent].state = STATE_READY;
		NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
		break;
	}
	case 2:	//sleep-----------------------------------------------------------------------------------------------
	{

		tcb[taskCurrent].ticks = r0;
		tcb[taskCurrent].state = STATE_DELAYED;
		NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
		break;
	}
	case 3:	//wait-----------------------------------------------------------------------------------------------
	{
		struct semaphore *pSemaphore;
		pSemaphore = r0;
		uint8_t k, inhen;
		if (pSemaphore->count == 0)
		{
			pSemaphore->processQueue[pSemaphore->queueSize] =
					(uint32_t) tcb[taskCurrent].pid;
			pSemaphore->queueSize++;
			tcb[taskCurrent].semaphore = pSemaphore;
			tcb[taskCurrent].state = STATE_BLOCKED;
			if (prioinhen == 1)
			{
				for (x = 0; x < MAX_TASKS; x++)
				{
					if (tcb[taskCurrent].semaphore == tcb[x].semaphore)
					{
						if (tcb[x].currentPriority
								> tcb[taskCurrent].currentPriority)
						{
							inhen = tcb[taskCurrent].currentPriority;
							tcb[x].currentPriority = inhen;

						}
					}
				}
			}
			NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
		}
		//if (pSemaphore->count != 0)
		else
		{
			pSemaphore->count = pSemaphore->count - 1;
		}
		break;
	}
	case 4:	//post-----------------------------------------------------------------------------------------------
	{
		struct semaphore *pSemaphore;
		pSemaphore = (uint32_t) r0;
		tcb[taskCurrent].currentPriority = tcb[taskCurrent].priority;
		pSemaphore->count++;
		if ((pSemaphore->count > 0) && (pSemaphore->queueSize > 0))
		{
			for (p = 0; p < MAX_TASKS; p++)
			{
				if (first1 < MAX_TASKS)
				{
					tempid[p] = tcb[p].pid;
					first1++;
				}

				for (j = 0; j < pSemaphore->queueSize; j++)
				{
					if (pSemaphore->processQueue[j] == (uint32_t) tcb[p].pid)
					{
						pSemaphore->processQueue[j] = 0;
						tcb[p].state = STATE_READY;
						pSemaphore->count--;
						pSemaphore->queueSize--;
						//break;
					}
				}
			}
		}
		tcb[taskCurrent].state = STATE_READY;
		NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
		break;
	}
	case 5:	//destroy--------------------------------------------------------------------------------------------
	{
		for (x = 0; x < MAX_TASKS; x++)
		{
			if (tcb[x].pid == (_fn) r0)
			{
				tcb[x].state = STATE_INVALID;
				pSemaphore = tcb[x].semaphore;
				for (j = 0; j < MAX_QUEUE_SIZE; j++)
				{
					if (pSemaphore->processQueue[j] == tcb[x].pid)
					{
						pSemaphore->processQueue[j] = 0;
						tcb[i].state = STATE_READY;
						pSemaphore->queueSize--;
					}
				}
				tcb[x].pid = 0;
				taskCount--;
				tcb[x].cpu_usage_percentage = 0;
				tcb[x].processTime = 0;
				tcb[x].taskTime = 0;
			}
			//totaltime = 0;
			first = 0;
		}
		break;
	}
	case 6:	//create thread--------------------------------------------------------------------------------------
	{
		bool ok = false;
		uint8_t i = 0;
		bool found = false;
		if (taskCount < MAX_TASKS)
		{
			// make sure fn not already in list (prevent reentrancy)
			while (!found && (i < MAX_TASKS))
			{
				found = (tcb[i++].pid == (_fn) r0);
			}
			if (!found)
			{
				// find first available tcb record
				i = 0;
				while (tcb[i].state != STATE_INVALID)
				{
					i++;
				}
				tcb[i].state = STATE_UNRUN;
				tcb[i].pid = (_fn) r0;
				tcb[i].sp = &stack[i][255];
				tcb[i].priority = r2;
				tcb[i].currentPriority = r2;
				strcpy(tcb[i].name, r1);
				// increment task count
				taskCount++;
				ok = true;
			}
		}
		return ok;
	}
	}

}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           5 pushbuttons, and uart
	SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN
			| SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

// Set GPIO ports to use APB (not needed since default configuration -- for clarity)
// Note UART on port A must use APB
	SYSCTL_GPIOHBCTL_R = 0;

// Enable GPIO port A,F and C peripherals
	SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB
			| SYSCTL_RCGC2_GPIOF;

// Configure LED and pushbutton pins
	GPIO_PORTA_DIR_R |= 0xE0; // bits 7,6,5 are outputs, other pins are inputs
	GPIO_PORTA_DR2R_R |= 0xE0; // set drive strength to 2mA (not needed since default configuration -- for clarity)
	GPIO_PORTA_DEN_R |= 0xFC; // enable LEDs and pushbuttons ( 7,6,5,4,3,2)
	GPIO_PORTA_PUR_R |= 0x1C;  // (4,3,2)

	GPIO_PORTB_DIR_R |= 0x10;  // bit 4, other pins are inputs
	GPIO_PORTB_DR2R_R |= 0x10; // set drive strength to 2mA (not needed since default configuration -- for clarity)
	GPIO_PORTB_DEN_R |= 0xD0; // enable LEDs and pushbuttons (7,6 and 4)
	GPIO_PORTB_PUR_R |= 0xC0;  // (7,6)

	GPIO_PORTF_DEN_R |= 0x04;  // enable LEDs and pushbuttons(blue)
//GPIO_PORTF_PUR_R |= 0x04;  // enable internal pull-up for push button
	GPIO_PORTF_DIR_R |= 0x04; // bits 1,2 and 3 are outputs, other pins are inputs
	GPIO_PORTF_DR2R_R |= 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)

// Configure GPIO pins for UART0
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // turn-on UART0, leave other uarts in same status
	GPIO_PORTA_DEN_R |= 3;                           // defaul
	GPIO_PORTA_AFSEL_R |= 3;                         // default
	GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

// Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
	UART0_CTL_R = 0;     // turn-off UART0 to allow safe programming
	UART0_CC_R = UART_CC_CS_SYSCLK;     // use system clock (40 MHz)
	UART0_IBRD_R = 21; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
	UART0_FBRD_R = 45;                      // round(fract(r)*64)=45
	UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
	UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module */

// Configure Systick timer for 1ms
	/*NVIC_ST_CTRL_R = 7;                //Enable SysTick during configuration
	 NVIC_ST_RELOAD_R = 0x00009C40;      //Reload value configured for 1ms
	 NVIC_ST_CURRENT_R = 0;              //Current value is reset to 0*/

// Configure Timer 1 as the time base
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;      // turn-on timer
	TIMER1_CTL_R &= ~TIMER_CTL_TAEN; // turn-off timer before reconfiguring
	TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER; // configure as 32-bit timer (A+B)
	TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR; // configure for periodic mode (count down) TIMER_TAMR_TACDIR
	TIMER1_TAV_R = 0x00000000;
	TIMER1_CTL_R |= TIMER_CTL_TAEN;
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
	while (UART0_FR_R & UART_FR_RXFE)
	{
		yield();
	}
	return UART0_DR_R & 0xFF;
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
	while (UART0_FR_R & UART_FR_TXFF)
		;
	UART0_DR_R = c;
}

void putnUart0(uint8_t n)
{

	while (UART0_FR_R & UART_FR_TXFF)
		;
	UART0_DR_R = n;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
	uint8_t i;
	for (i = 0; i < mystrlen(str); i++)
		putcUart0(str[i]);
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
// Approx clocks per us
	__asm("WMS_LOOP0:   MOV  R1, #6");
// 1
	__asm("WMS_LOOP1:   SUB  R1, #1");
// 6
	__asm("             CBZ  R1, WMS_DONE1");
// 5+1*3
	__asm("             NOP");
// 5
	__asm("             B    WMS_LOOP1");
// 5*3
	__asm("WMS_DONE1:   SUB  R0, #1");
// 1
	__asm("             CBZ  R0, WMS_DONE0");
// 1
	__asm("             B    WMS_LOOP0");
// 1*3
	__asm("WMS_DONE0:");
// ---
// 40 clocks/us + error
}

// REQUIRED: add code to return a value from 0-31 indicating which of 5 PBs are pressed
uint8_t readPbs()
{
	light = 0;
	if (PB1 == 0)
	{
		light = 1;
	}
	if (PB2 == 0)
	{
		light = light + 2;
	}
	if (PB3 == 0)
	{
		light = light + 4;
	}
	if (PB4 == 0)
	{
		light = light + 8;
	}
	if (PB5 == 0)
	{
		light = light + 16;
	}
	return light;
}

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
	while (true)
	{
		ORANGE_LED = 1;
		waitMicrosecond(1000);
		ORANGE_LED = 0;
		yield();
	}
}

void idle2()
{
	while (true)
	{
		YELLOW_LED = 1;
		waitMicrosecond(1000);
		YELLOW_LED = 0;
		yield();
	}
}

void flash4Hz()
{
	while (true)
	{
		GREEN_LED ^= 1;
		sleep(125);
	}
}

void oneshot()
{
	while (true)
	{
		wait(flashReq);
		YELLOW_LED = 1;
		sleep(1000);
		YELLOW_LED = 0;
	}
}

void partOfLengthyFn()
{
// represent some lengthy operation
	waitMicrosecond(1000);
// give another process a chance to run
	yield();
}

void lengthyFn()
{
	uint16_t i;
	while (true)
	{
		wait(resource);
		for (i = 0; i < 4000; i++)
		{
			partOfLengthyFn();
		}
		RED_LED ^= 1;
		post(resource);
	}
}

void readKeys()
{
	uint8_t buttons;
	while (true)
	{
		wait(keyReleased);
		buttons = 0;
		while (buttons == 0)
		{
			buttons = readPbs();
			yield();
		}
		post(keyPressed);
		if ((buttons & 1) != 0)
		{
			YELLOW_LED ^= 1;
			RED_LED = 1;
		}
		if ((buttons & 2) != 0)
		{
			post(flashReq);
			RED_LED = 0;
		}
		if ((buttons & 4) != 0)
		{
			createThread(flash4Hz, "Flash4Hz", 0);
		}
		if ((buttons & 8) != 0)
		{
			destroyThread(flash4Hz);
		}
		if ((buttons & 16) != 0)
		{
			setThreadPriority(lengthyFn, 4);
		}
		yield();
	}
}

void debounce()
{
	uint8_t count;
	while (true)
	{
		wait(keyPressed);
		count = 10;
		while (count != 0)
		{
			sleep(10);
			if (readPbs() == 0)
				count--;
			else
				count = 10;
		}
		post(keyReleased);
	}
}

void uncooperative()
{
	while (true)
	{
		while (readPbs() == 8)
		{
		}
		yield();
	}
}

void important()
{
	while (true)
	{
		wait(resource);
		BLUE_LED = 1;
		sleep(1000);
		BLUE_LED = 0;
		post(resource);
	}
}

//-------------------------------------------------------------------------------------------------------
uint8_t mystrlen(char *str)
{
	uint8_t i;
	for (i = 0; str[i] != '\0'; i++)
		;
	return i;
}

//-------------------------------------------------------------------------------------------------------
void int2char(uint16_t n)
{
	char d;
	uint16_t u = 0, y = 0;
	for (u = 4; u > 0; u--)
	{
		y = pow(10, (u - 1));
		d = ((n / y) % 10) + 48;
		putcUart0(d);
	}
}

//-------------------------------------------------------------------------------------------------------
void int2char1(uint8_t n)
{
	char d;
	uint8_t u = 0, y = 0, a = 0;
	for (u = 1; u > 0; u--)
	{
		y = pow(10, (u - 1));
		d = ((n / y) % 10) + 48;
		putcUart0(d);
	}
}

//------------------------------------------------------------------------------------------------------------------
void int2char2(uint8_t n)
{
	char d;
	uint8_t u = 0, y = 0, a = 0;
	for (u = 2; u > 0; u--)
	{
		y = pow(10, (u - 1));
		a = (n / y) % 10;
		if (a < 10)
		{
			d = a + 48;
		}
		else
		{
			d = a + 55;
		}
		putcUart0(d);
	}
}

//-----------------------------------------------------------------------------------------------------
void hex2char(uint32_t n)
{

	char d;
	uint32_t u = 0, y = 0, a = 0;
	for (u = 8; u > 0; u--)
	{
		y = pow(16, (u - 1));
		a = (n / y) % 16;
		if (a < 10)
		{
			d = a + 48;
		}
		else
		{
			d = a + 55;
		}
		putcUart0(d);
	}
}

//-----------------------------------------------------------------------------------------------------
void float2char(uint16_t n)
{
	char d;
	char percent = 37;
	uint16_t u, y;
	for (u = 4; u > 2; u--)
	{
		y = pow(10, (u - 1));
		d = ((n / y) % 10) + 48;
		putcUart0(d);
	}
	putsUart0(".");
	for (u = 2; u > 0; u--)
	{
		y = pow(10, (u));
		d = ((n / y) % 10) + 48;
		putcUart0(d);
	}
	putcUart0(percent);
}

//-----------------------------------------------------------------------------------------------------
void turnonSystick()
{
	NVIC_ST_CTRL_R = 7;        //Enable SysTick during configuration
	NVIC_ST_RELOAD_R = 0x00009C40; //Reload value configured for 1ms
	NVIC_ST_CURRENT_R = 0;             //Current value is reset to 0
}

//-----------------------------------------------------------------------------------------------------
uint32_t myatoi(char *pidn)
{
	int res = 0, i;
	for (i = 0; pidn[i] != '\0'; ++i)
		res = res * 10 + pidn[i] - 48;
	return res;
}

//-----------------------------------------------------------------------------------------------------
uint8_t mystrcmp(char *str1, char *str2)
{
	if (*str1 == '\0' || *str2 == '\0')
	{
		return 1;
	}
	else
	{
		while (*str1 != '\0' || *str2 != '\0')
		{
			if (*str1 == *str2)
			{
				str1++;
				str2++;
			}
			else
			{
				return (*str1 - *str2);
			}
		}
		return 0;
	}
}

//---------------------------------------------------------------------------------------------------------------
void shell()
{
	putsUart0("\n\r");
	putsUart0(
			"********************************************************************************\n\r");
	putsUart0("EE 6314 Advanced Embedded Microcontroller System Design\n\r");
	putsUart0("  Project 1 – Design of a Real-time Operating System\n\r");
	putsUart0("                     Fall 2018 \r\n");
	putsUart0("\n\r");
	putsUart0("Author: YETISH KRISHNA REDDY\r\n");
	putsUart0("UTA ID: 1001574820\r\n");
	putsUart0("\n\r");
	putsUart0("Commands:\n\r");
	putsUart0("---------\n\r");
	putsUart0("1. Set Priority on/off - Sets the Priority on/off\n\r");
	putsUart0("2. Set Preemption on/off - Sets the Preemption on/off\n\r");
	putsUart0("3. Set PI on - Sets the Priority Inheritance on/off\n\r");
	putsUart0("4. Kill Pidno - Kills the task\n\r");
	putsUart0("5. PS - Displays the Process Status\n\r");
	putsUart0("6. IPCS - Displays the Inter Process Communication Status\n\r");
	putsUart0("7. PidOf taskname - Displays the Process ID of the task\n\r");
	putsUart0("8. Processname& - Creates the Process Status\n\r");
	putsUart0("9. Reboot - Restarts the controller\n\r");
	putsUart0(
			"********************************************************************************\n\r");
	putsUart0("\n\r");
	while (true)
	{
		//putsUart0("\n\r");
		putsUart0("Enter commands\r\n");
		while (1)
		{
			count = 0;
			Jump: ch = getcUart0();
			ch = tolower(ch);
			if (ch == 8)
			{
				if (count > 0)
				{
					count--;
					goto Jump;
				}
				else
				{
					goto Jump;
				}
			}
			if ((ch == 13) || (ch == 20))
			{
				str[count] = 0;
				break;
			}
			else
			{
				if (ch < ' ')
				{
					goto Jump;
				}
				else
				{
					str[count++] = ch;
					if ((count > MaxChar) || (count == MaxChar))
					{
						str[count] = '\0';

					}
					else
					{
						goto Jump;
					}
				}
			}
		}
		//putsUart0("\r\nEntered string is:\n\r");
		//putsUart0(str);
		putsUart0("\n");

		if (*str == '\0')
		{
			*str = '0';
		}
		putsUart0("\r\n");
		char str1[5][11];
		uint16_t count = 0;
		uint16_t count1 = 0;
		uint16_t i = 0;
		for (i = 0; i < mystrlen(str) - 1; i++)
		{
			if ((str[i] != ' ' && str[i] != ',' && str[i] != ';'
					&& str[i] != ':' && str[i] != '!' && str[i] != '_'
					&& str[i] != '#' && str[i] != '@' && str[i] != '$'
					&& str[i] != '*' && str[i] != '/' && str[i] != ','
					&& str[i] != '^')
					&& (str[i + 1] == ' ' || str[i + 1] == ','
							|| str[i + 1] == ';' || str[i + 1] == ':'
							|| str[i + 1] == '!' || str[i + 1] == '_'
							|| str[i + 1] == '#' || str[i + 1] == '@'
							|| str[i + 1] == '$' || str[i + 1] == '*'
							|| str[i + 1] == '/' || str[i + 1] == ','
							|| str[i + 1] == '^'))
			{
				str1[count][count1] = str[i];
				str1[count][++count1] = '\0';
				count++;
				count1 = 0;
			}
			else if (str[i] != ' ' && str[i] != ',' && str[i] != ';'
					&& str[i] != ':' && str[i] != '!' && str[i] != '_'
					&& str[i] != '#' && str[i] != '@' && str[i] != '$'
					&& str[i] != '*' && str[i] != '/' && str[i] != ','
					&& str[i] != '^')
			{
				str1[count][count1] = str[i];
				count1++;
			}
			if (i == mystrlen(str) - 2)
			{
				if (((str[i] == ' ' || str[i] == ',' || str[i] == ';'
						|| str[i] == ':' || str[i] == '!' || str[i] == '_'
						|| str[i] == '#' || str[i] == '@' || str[i] == '$'
						|| str[i] == '*' || str[i] == '/' || str[i] == ','
						|| str[i] == '^')
						&& (str[i + 1] != ' ' && str[i + 1] != ','
								&& str[i + 1] != ';' && str[i + 1] != ':'
								&& str[i + 1] != '!' && str[i + 1] != '_'
								&& str[i + 1] != '#' && str[i + 1] != '@'
								&& str[i + 1] != '$' && str[i + 1] != '*'
								&& str[i + 1] != '/' && str[i + 1] != ','
								&& str[i + 1] != '^')))
				{
					str1[count++][count1] = str[i + 1];
					str1[count][++count1] = '\0';
				}
				else if ((str[i] != ' ' && str[i] != ',' && str[i] != ';'
						&& str[i] != ':' && str[i] != '!' && str[i] != '_'
						&& str[i] != '#' && str[i] != '@' && str[i] != '$'
						&& str[i] != '*' && str[i] != '/' && str[i] != ','
						&& str[i] != '^')
						&& (str[i + 1] != ' ' && str[i + 1] != ','
								&& str[i + 1] != ';' && str[i + 1] != ':'
								&& str[i + 1] != '!' && str[i + 1] != '_'
								&& str[i + 1] != '#' && str[i + 1] != '@'
								&& str[i + 1] != '$' && str[i + 1] != '*'
								&& str[i + 1] != '/' && str[i + 1] != ','
								&& str[i + 1] != '^'))
				{
					str1[count][count1] = str[i + 1];
					str1[count++][++count1] = '\0';
				}
			}
		}
		char* args0 = str1[0];
		char* args1 = str1[1];
		char* args2 = str1[2];
		uint8_t x;
// Priority on or off module ------------------------------------------------------------------------------------
		if (mystrcmp(args0, "set") == 0)
		{
			if (mystrcmp(args1, "priority") == 0)
			{
				if (mystrcmp(args2, "on") == 0)
				{
					putsUart0("Priority ON \n\r");
					putcUart0('\n');
					putcUart0('\r');
					pi = 1;
				}
				else if (mystrcmp(args2, "off") == 0)
				{
					putsUart0("Priority OFF \n\r");
					putcUart0('\n');
					putcUart0('\r');
					pi = 0;
				}
				else
				{
					putsUart0("Error in command");
					putcUart0('\n');
					putcUart0('\r');
				}
			}
// Priority inheritance on or off module ------------------------------------------------------------------------
			if (mystrcmp(args1, "pi") == 0)
			{
				if (mystrcmp(args2, "on") == 0)
				{
					putsUart0("Priority Inheritance ON \n\r");
					putcUart0('\n');
					putcUart0('\r');
					prioinhen = 1;
				}
				else if (mystrcmp(args2, "off") == 0)
				{
					putsUart0("Priority Inheritance OFF \n\r");
					putcUart0('\n');
					putcUart0('\r');
					prioinhen = 0;
				}
				else
				{
					putsUart0("Error in command");
					putcUart0('\n');
					putcUart0('\r');
				}
			}
// Pre-empt on off module----------------------------------------------------------------------------------------
			if (mystrcmp(args1, "preemption") == 0)
			{
				if (mystrcmp(args2, "on") == 0)
				{
					putsUart0("Preemption is ON \n\r");
					putcUart0('\n');
					putcUart0('\r');
					pre = 1;
				}
				else if (mystrcmp(args2, "off") == 0)
				{
					putsUart0("Preemption OFF \n\r");
					putcUart0('\n');
					putcUart0('\r');
					pre = 0;
				}
				else
				{
					putsUart0("Error in command");
					putcUart0('\n');
					putcUart0('\r');
				}
			}
		}
// reboot module-------------------------------------------------------------------------------------------------
		else if (mystrcmp(args0, "reboot") == 0)
		{
			putsUart0("\r\n rebooting \r\n\n");
			NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
		}
//pid module-----------------------------------------------------------------------------------------------------
		else if (mystrcmp(args0, "pidof") == 0)
		{
			uint8_t x = 0, j = 0;
			for (x = 0; x < MAX_TASKS; x++)
			{
				if (mystrcmp(args1, name_task[x]) == 0)
				{
					putsUart0("The PID of ");
					putsUart0(name_task[x]);
					putsUart0(" is = ");
					int2char(tcb[x].pid);
					putcUart0('\n');
					putcUart0('\n');
					putcUart0('\r');
					j = 1;
					//break;
				}
			}
			if (j == 0)
			{
				putsUart0("Task not found");
				putcUart0('\n');
				putcUart0('\r');
			}
		}
//ipcs module----------------------------------------------------------------------------------------------------
		else if (mystrcmp(args0, "ipcs") == 0)
		{
			uint8_t x, i, m, n, o, p;
			putsUart0(
					"--------------------------------------------------------------------------------\n\r");
			putsUart0(" ");
			putsUart0(
					"SEMAPHORE NAME     |   COUNT    |  QUEUE SIZE  |  PROCESS IN QUEUE\n\r");
			putsUart0(
					"--------------------------------------------------------------------------------\n\r");
			for (x = 0; x < MAX_SEMAPHORES - 1; x++)
			{
				putsUart0(" ");
				putsUart0(name_sema[x]);
				for (o = 0; o < 19 - mystrlen(name_sema[x]); o++)
				{
					putsUart0(" ");
				}
				putsUart0("|");
				putcUart0('\t');
				putsUart0(" ");
				putsUart0(" ");
				int2char1(semaphores[x].count);
				for (m = 0; m < 6; m++)
					putsUart0(" ");
				putsUart0("|");
				for (m = 0; m < 6; m++)
					putsUart0(" ");
				int2char1(semaphores[x].queueSize);
				for (m = 0; m < 7; m++)
					putsUart0(" ");
				putsUart0("|");
				for (m = 0; m < 2; m++)
					putsUart0(" ");
				int2char(semaphores[x].processQueue[0]);
				for (n = 0; n < 9; n++)
				{
					if (semaphores[x].processQueue[0] == tempid[n])
					{
						putsUart0(" - ");
						putsUart0(tcb[n].name);
					}
				}
				putcUart0('\n');
				putcUart0('\r');
				putsUart0(
						"--------------------------------------------------------------------------------\n\r");
			}
			putcUart0('\n');
		}
//ps module------------------------------------------------------------------------------------------------------
		else if (mystrcmp(args0, "ps") == 0)
		{
			putsUart0(
					"--------------------------------------------------------------------------------\n\r");
			putsUart0(
					" PID  | PROCESS NAME |   CPU%  | PROCESS STATE\t \t|STACK POINTER |PRIORITY\n\r");
			putsUart0(
					"--------------------------------------------------------------------------------\n\r");
			char hex[50];
			uint8_t m;
			for (x = 0; x < 9; x++)
			{
				if (tcb[x].pid != 0)
				{
					putsUart0(" ");
					int2char(tcb[x].pid);
					putsUart0(" ");
					putsUart0("|");
					putsUart0(" ");
					putsUart0(tcb[x].name);
					for (i = 0; i < 13 - mystrlen(tcb[x].name); i++)
						putsUart0(" ");
					putsUart0("|");
					putsUart0(" ");
					putsUart0(" ");
					float2char(tcb[x].cpu_usage_percentage);
					//putcUart0('\t');
					putsUart0(" ");
					putsUart0("|");
					putsUart0(" ");
					if (tcb[x].state == 0)
					{
						putsUart0("Invalid");
						for (i = 0; i < 17 - mystrlen(tcb[x].state); i++)
							putsUart0(" ");
					}
					else if (tcb[x].state == 1)
					{
						putsUart0("Unrun  ");
						for (i = 0; i < 17 - mystrlen(tcb[x].state); i++)
							putsUart0(" ");
					}
					else if (tcb[x].state == 2)
					{
						putsUart0("Ready  ");
						for (i = 0; i < 9 - mystrlen(tcb[x].state); i++)
							putsUart0(" ");
					}
					else if (tcb[x].state == 3)
					{
						putsUart0("Blocked");
						for (m = 0; m < MAX_SEMAPHORES - 1; m++)
							if (tcb[x].pid == semaphores[m].processQueue[0])
							{
								putsUart0(" by ");
								putsUart0(name_sema[m]);
							}
					}
					else if (tcb[x].state == 4)
					{
						putsUart0("Delayed");
						for (i = 0; i < 17 - mystrlen(tcb[x].state); i++)
							putsUart0(" ");
					}
					putcUart0('\t');
					putsUart0("|");
					putsUart0(" ");
					putsUart0(" ");
					hex2char(tcb[x].sp);
					putsUart0(" ");
					putsUart0(" ");
					putsUart0(" ");
					putsUart0(" ");
					putsUart0("|  ");
					int2char2(tcb[x].currentPriority);
					putcUart0('\n');
					putcUart0('\r');
					putsUart0(
							"--------------------------------------------------------------------------------\n\r");
				}
			}
			putcUart0('\n');
		}
//kill module ---------------------------------------------------------------------------------------------------
		else if (mystrcmp(args0, "kill") == 0)
		{
			uint8_t x, j = 0;
			for (x = 0; x < taskCount; x++)
			{
				if ((myatoi(args1)) == tcb[x].pid)
				{
					putsUart0(tcb[x].name);
					putsUart0(" thread is killed \n\r");
					putsUart0(" \n\r");
					destroyThread((_fn) tcb[x].pid);
					j = 1;
				}
			}
			if (j == 0)
			{
				putsUart0("Thread PID is not entered or wrong\n\r");
				putsUart0("\n\r");
			}
		}
//create module--------------------------------------------------------------------------------------------------
		else
		{
			uint8_t x, j, k, l = 0, z = 0;
			for (j = 0; j < mystrlen(args0); j++)
			{
				if (args0[j] == '&')
				{
					l = 1;
					args0[mystrlen(args0) - 1] = 0;
					for (x = 0; x < MAX_TASKS; x++)
					{
						if (mystrcmp(args0, name_task[x]) == 0)
						{
							z = 1;
							if (mystrcmp(args0, "lengthyfn") == 0)

							{
								createThread(lengthyFn, "LengthyFn", 12);
								putsUart0(args0);
								putsUart0(" thread Created \n\r");
								putsUart0("\n\r");
							}
							else if (mystrcmp(args0, "flash4hz") == 0)
							{
								createThread(flash4Hz, "Flash4Hz", 4);
								putsUart0(args0);
								putsUart0(" thread Created \n\r");
								putsUart0("\n\r");
							}
							else if (mystrcmp(args0, "oneshot") == 0)

							{
								createThread(oneshot, "OneShot", 4);
								putsUart0(args0);
								putsUart0(" thread Created \n\r");
								putsUart0("\n\r");
							}
							else if (mystrcmp(args0, "readkeys") == 0)

							{
								createThread(readKeys, "ReadKeys", 12);
								putsUart0(args0);
								putsUart0(" thread Created \n\r");
								putsUart0("\n\r");
							}
							else if (mystrcmp(args0, "debounce") == 0)

							{
								createThread(debounce, "Debounce", 12);
								putsUart0(args0);
								putsUart0(" thread Created \n\r");
								putsUart0("\n\r");
							}
							else if (mystrcmp(args0, "important") == 0)

							{
								createThread(important, "Important", 0);
								putsUart0(args0);
								putsUart0(" thread Created \n\r");
								putsUart0("\n\r");
							}
							else if (mystrcmp(args0, "uncoop") == 0)

							{
								createThread(uncooperative, "Uncoop", 10);
								putsUart0(args0);
								putsUart0(" thread Created \n\r");
								putsUart0("\n\r");
							}
							else if (mystrcmp(args0, "shell") == 0)

							{
								createThread(shell, "Shell", 8);
								putsUart0(args0);
								putsUart0(" thread Created \n\r");
								putsUart0("\n\r");

							}
							else if (mystrcmp(args0, "idle") == 0)

							{
								createThread(shell, "Idle", 15);
								putsUart0(args0);
								putsUart0(" thread Created \n\r");
								putsUart0("\n\r");

							}
							else
							{
								putsUart0("Thread already created\n\r");
								putsUart0("\n\r");
							}
						}
					}
					if (z == 0)
					{
						putsUart0("Thread name not found \n\r");
						putsUart0("\n\r");
					}
				}
			}
			if (l == 0)
			{
				putsUart0(
						"Error in command or suffix '&' to create a thread \n\r");
				putsUart0("\n\r");
			}
		}
	}
}
// Clearing the Strings------------------------------------------------------------------------------------------
//uint8_t xx, yy, zz;
/*for (xx = 0; xx < count; xx++)
 {
 for (yy = 0; yy < 20; yy++)
 {
 str1[xx][yy] = '\0';
 }
 }*/

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
int main(void)
{
	bool ok;

// Initialize hardware
	initHw();
	rtosInit();

// Power-up flash
	GREEN_LED = 1;
	waitMicrosecond(250000);
	GREEN_LED = 0;
	waitMicrosecond(250000);

// Initialize semaphores
	keyPressed = createSemaphore(1);
	keyReleased = createSemaphore(0);
	flashReq = createSemaphore(5);
	resource = createSemaphore(1);

// Add required idle process
	ok = createThread(idle, "Idle", 15);
//ok =  createThread(idle2, "Idle2", 15);

// Add other processes
	ok &= createThread(lengthyFn, "LengthyFn", 12);
	ok &= createThread(flash4Hz, "Flash4Hz", 4);
	ok &= createThread(oneshot, "OneShot", 4);
	ok &= createThread(readKeys, "ReadKeys", 12);
	ok &= createThread(debounce, "Debounce", 12);
	ok &= createThread(important, "Important", 0);
	ok &= createThread(uncooperative, "Uncoop", 10);
	ok &= createThread(shell, "Shell", 8);

// Start up RTOS
	if (ok)
		rtosStart(); // never returns
	else
		RED_LED = 1;

	return 0;
}
