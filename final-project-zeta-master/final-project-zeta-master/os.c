// Modified by Mustafa Hotaki 8/1/2018
// MODIFIED BY SILE SHU 2017.6
// os.c
// Runs on LM4F120/TM4C123
// A very simple real time operating system with minimal features.
// Daniel Valvano
// January 29, 2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
   Programs 4.4 through 4.12, section 4.2
 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#include <stdint.h>
#include "os.h"
#include "PLL.h"
#include "tm4c123gh6pm.h"
#include "LCD.h"
#include "UART.h"
#include "joystick.h"

// Functions implemented in assembly files
void OS_DisableInterrupts(void);	// Disable interrupts
void OS_EnableInterrupts(void);  	// Enable interrupts
long StartCritical(void);    			// previous I bit, disable interrupts
void EndCritical(long sr);    		// restore I bit to previous value
void WaitForInterrupt(void);  		// low power mode
void StartOS(void);

// Periodic task function pointers
void (*PeriodicTask1)(void);
void (*PeriodicTask2)(void);

// Button task function pointers
void (*ButtonOneTask)(void);
void (*ButtonTwoTask)(void);

#define NUMTHREADS	20					// Maximum number of threads
#define STACKSIZE		100      		// Number of 32-bit words in stack

// Macros
//#define blockSema								// Blocking sempahores
//#define prioritySched						// Fixed priority scheduler
//#define aging										// Dynamic priority scheculer with aging

// TCB Data Structure
struct tcb {
  int32_t *sp;           // Pointer to stack (valid for threads not running
  struct tcb *next;      // Linked list pointer
  uint32_t id;           // Thread #
  uint32_t available;    // Used to indicate if this tcb is available or not
	uint32_t sleepCt;	     // Sleep counter in MS
  uint32_t ArriveTime;   // First time thread is added to the system
  uint32_t WaitTime;     // Elapsed time since thread arrived till it starts execution
  uint32_t ExecCount;    // Number of times thread is executed (switched to)
#ifdef blockSema
  Sema4Type *blockPt;    // Pointer to resource thread is blocked on (0 if not)
#endif
#ifdef prioritySched
#ifdef aging
  uint32_t age;          // How long the thread has been active
  uint32_t FixedPriority;// Permanent priority
  uint32_t WorkPriority; // Temporary priority 
#else
	uint32_t priority;
#endif
#endif
};
typedef struct tcb tcbType;

tcbType *RunPt;														// Pointer to the currently running TCB
tcbType tcbs[NUMTHREADS]; 								// Statically allocated memory for TCBs
int32_t Stacks[NUMTHREADS][STACKSIZE];		// Statically allocated memory for Stacks

// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// initialize OS controlled I/O: systick, 80 MHz PLL
// input:  none
// output: none
void OS_Init(void){int i;
  OS_DisableInterrupts();
  PLL_Init(Bus80MHz);                 // set processor clock to 80 MHz
	for(i = 0; i < NUMTHREADS; i++){
		tcbs[i].available = 1; // initial available
	}  
	InitTimer2A(TIME_1MS);  // initialize Timer2A which is used for software timer and decrease the sleepCt
	InitTimer3A();
  OS_ClearMsTime();
  
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
															// lowest PRI so only foreground interrupted
  NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0x00FFFFFF)|0xE0000000; // priority 7
}

void SetInitialStack(int i){
  tcbs[i].sp = &Stacks[i][STACKSIZE-16]; // thread stack pointer
  Stacks[i][STACKSIZE-1] = 0x01000000;   // thumb bit
  Stacks[i][STACKSIZE-3] = 0x14141414;   // R14
  Stacks[i][STACKSIZE-4] = 0x12121212;   // R12
  Stacks[i][STACKSIZE-5] = 0x03030303;   // R3
  Stacks[i][STACKSIZE-6] = 0x02020202;   // R2
  Stacks[i][STACKSIZE-7] = 0x01010101;   // R1
  Stacks[i][STACKSIZE-8] = 0x00000000;   // R0
  Stacks[i][STACKSIZE-9] = 0x11111111;   // R11
  Stacks[i][STACKSIZE-10] = 0x10101010;  // R10
  Stacks[i][STACKSIZE-11] = 0x09090909;  // R9
  Stacks[i][STACKSIZE-12] = 0x08080808;  // R8
  Stacks[i][STACKSIZE-13] = 0x07070707;  // R7
  Stacks[i][STACKSIZE-14] = 0x06060606;  // R6
  Stacks[i][STACKSIZE-15] = 0x05050505;  // R5
  Stacks[i][STACKSIZE-16] = 0x04040404;  // R4
}

///******** OS_Launch ***************
// start the scheduler, enable interrupts
// Inputs: number of 20ns clock cycles for each time slice
//         (maximum of 24 bits)
// Outputs: none (does not return)
void OS_Launch(unsigned long theTimeSlice){
	NVIC_ST_RELOAD_R = theTimeSlice - 1; // reload value
  NVIC_ST_CTRL_R = 0x00000007; // enable, core clock and interrupt arm
  StartOS();                   // start on the first task
}

// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
void OS_Suspend(void) { 
	NVIC_ST_CURRENT_R  = 0; // reset counter
	NVIC_INT_CTRL_R = 0x04000000;		// trigger SysTick
}

//******** OS_AddThread *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
static uint32_t ThreadNum = 0;
int OS_AddThread(void(*task)(void), unsigned long stackSize, unsigned long priority){
	unsigned char i,j;	 
	int32_t status,thread;
  status = StartCritical();
  if (ThreadNum == NUMTHREADS){ // no available tcbs
	  return 0;
  }
  else{
	  	if (ThreadNum == 0){  // start add thread
			tcbs[0].available = 0;
			tcbs[0].next = &tcbs[0];  // first, create a single cycle
			RunPt = &tcbs[0]; //start from tcbs[0]
			thread = 0;
		}
		else{   // not the start
			for (i=0;i<NUMTHREADS;i++){
				if (tcbs[i].available) break;   // find an available tcb for the new thread
			}
			thread = i;
			tcbs[i].available = 0; // make this tcb no longer available
			for (j = (thread + NUMTHREADS - 1)%  NUMTHREADS; j != thread; j = (j + NUMTHREADS - 1) % NUMTHREADS){
				if (tcbs[j].available == 0) break; // find a previous tcb which has been used
			}
			// add this tcb into the link list cycle
			tcbs[thread].next = tcbs[j].next;  
			tcbs[j].next = &tcbs[thread];
		}
		tcbs[thread].id = thread;
		tcbs[thread].WaitTime = 0; // Initially 0
		tcbs[thread].ArriveTime = OS_MsTime();
		tcbs[thread].ExecCount = 0; // Initially 0

#ifdef prioritySched
#ifdef aging
		tcbs[thread].FixedPriority = priority;
		tcbs[thread].WorkPriority = priority;
#else
		tcbs[thread].priority = priority;
#endif
#endif
	
		SetInitialStack(thread); 
		Stacks[thread][STACKSIZE-2] = (int32_t)(task); // PC		
		ThreadNum++;
		EndCritical(status);
		return 1; 
	}            
}
	 
//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
unsigned long OS_Id(void) { 
	return RunPt->id;
}
	 
// ******** OS_Wait ************
// decrement semaphore 
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt){
#ifdef blockSema
	OS_DisableInterrupts();
	semaPt->Value -= 1;
	if (semaPt->Value < 0){
		RunPt->blockPt = semaPt;
		OS_EnableInterrupts();
		OS_Suspend();
	}
	OS_EnableInterrupts();
#else
	OS_DisableInterrupts();
	while (semaPt->Value == 0){
		OS_EnableInterrupts();
		OS_Suspend();
		OS_DisableInterrupts();
	}
	semaPt->Value -= 1;
	OS_EnableInterrupts();
#endif
}

// ******** OS_Signal ************
// increment semaphore 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt){
#ifdef blockSema
	tcbType *pt;
	OS_DisableInterrupts();
	semaPt->Value += 1;
	if (semaPt->Value <= 0){
		pt = RunPt->next;
		while (pt->blockPt != semaPt){
			pt = pt->next;
		}
		pt->blockPt = 0; // wake up this one
	}
	OS_EnableInterrupts();
#else
	OS_DisableInterrupts();
	semaPt->Value += 1;
	OS_EnableInterrupts();
#endif
}

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, long value){
	OS_DisableInterrupts();
	semaPt->Value = value;
	OS_EnableInterrupts();
}

// ******** OS_bWait ************
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt){
#ifdef blockSema
	OS_DisableInterrupts();
	semaPt->Value -= 1;
	if (semaPt->Value < 0){
		RunPt->blockPt = semaPt;
		OS_EnableInterrupts();
		OS_Suspend();
	}
	OS_EnableInterrupts();
#else
	OS_DisableInterrupts();
	while (semaPt->Value == 0){
		OS_EnableInterrupts();
		OS_Suspend();
		OS_DisableInterrupts();
	}
	semaPt->Value = 0;
	OS_EnableInterrupts();
#endif
}	

// ******** OS_bSignal ************
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4Type *semaPt){
#ifdef blockSema
	tcbType *pt;
	OS_DisableInterrupts();
	semaPt->Value++;
	if ((semaPt->Value) > 1){
		semaPt->Value = 1;
	}
	if (semaPt->Value <= 0){
		pt = RunPt->next;
		while (pt->blockPt != semaPt){
			pt = pt->next;
		}
		pt->blockPt = 0; // wake up this one
	}
	OS_EnableInterrupts();
#else
	OS_DisableInterrupts();
	semaPt->Value = 1;
	OS_EnableInterrupts();
#endif
}

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(unsigned long sleepTime){
	RunPt->sleepCt = sleepTime;
	OS_Suspend();
}

// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void){
	unsigned char i;
	int32_t thread;
	RunPt->available = 1;
	thread = OS_Id();
	for (i = (thread + NUMTHREADS - 1) % NUMTHREADS; i != thread; i = (i + NUMTHREADS - 1) % NUMTHREADS){
		if (tcbs[i].available == 0) 
			break;   // find the previous used tcb
	}
	ThreadNum--;
  tcbs[i].next = tcbs[thread].next;	
	OS_Suspend(); // switch the thread
}	

void Scheduler(void){
#ifdef blockSema 
#ifdef prioritySched
	uint32_t max = 255; // max priority
	tcbType *pt;
	tcbType *bestPt;  // threads that have highest priority
	RunPt = RunPt->next;
	pt = RunPt; // search for highest thread not blocked or sleeping
	do {
#ifdef aging // dynamic priority scheduling
		if ((pt->WorkPriority < max) && (pt->blockPt == 0) && (pt->sleepCt == 0)){
			 max = pt->WorkPriority;
			 bestPt = pt;
		}
		pt = pt->next; //  skips at least one
	} while(RunPt != pt);
	RunPt = bestPt;
	RunPt->WorkPriority = RunPt->FixedPriority;			 
#else // fixed priority scheduling
		if ((pt->priority < max) && (pt->blockPt == 0) && (pt->sleepCt == 0)){
			 max = pt->priority;
			 bestPt = pt;
		}
		pt = pt->next; //  skips at least one
	} while(RunPt != pt);
	RunPt = bestPt;
#endif
#else // round robin scheduling
	RunPt = RunPt->next;
	while ((RunPt->sleepCt) || (RunPt->blockPt)){
		RunPt = RunPt->next;
	}
#endif
#else // spinlock semaphores
	RunPt = RunPt->next;
	while(RunPt->sleepCt){
		RunPt = RunPt->next;
	}
#endif
	if (RunPt->ExecCount == 0) 
		RunPt->WaitTime = OS_MsTime() - RunPt->ArriveTime;
	RunPt->ExecCount += 1;
}

//******** OS_AddPeriodicThread *************** 
// add a background periodic task
// typically this function receives the highest priority
// Inputs: pointer to a void/void background function
//         period given in system time units (12.5ns)
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// You are free to select the time resolution for this function
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
int OS_AddPeriodicThread(void(*task)(void), 
   unsigned long period, unsigned long priority) { 
	static uint16_t PeriodTaskCt;
	if (PeriodTaskCt == 0){
		PeriodicTask1 = task;
		InitTimer1A(period,priority);
	}
	else {
		PeriodicTask2 = task;
		InitTimer4A(period,priority);
	}
	PeriodTaskCt++;
  return 1;
}


// Timing Functions ------------------------------------------------------------------------------

// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
unsigned long OS_Time(void) { 
	return TIMER3_TAILR_R - TIMER3_TAV_R;
}

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
unsigned long OS_TimeDifference(unsigned long start, unsigned long stop) {
	return stop-start;
}

// Ms time system
static uint32_t MSTime;
// ******** OS_ClearMsTime ************
// sets the system time to zero
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void) {
	MSTime = 0;
}

// ******** OS_MsTime ************
// reads the current time in msec
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// It is ok to make the resolution to match the first call to OS_AddPeriodicThread
unsigned long OS_MsTime(void) {	
	return MSTime;
}

// Timers ------------------------------------------------------------------------------

void InitTimer1A(unsigned long period, uint32_t priority) {
	long sr;
	volatile unsigned long delay;
	
	sr = StartCritical();
  SYSCTL_RCGCTIMER_R |= 0x02;
	
  while((SYSCTL_RCGCTIMER_R & 0x02) == 0){} // allow time for clock to stabilize
	
  TIMER1_CTL_R &= ~TIMER_CTL_TAEN; // 1) disable timer1A during setup
                                   // 2) configure for 32-bit timer mode
  TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
                                   // 3) configure for periodic mode, default down-count settings
  TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER1_TAILR_R = period - 1;     // 4) reload value
                                   // 5) clear timer1A timeout flag
  TIMER1_ICR_R = TIMER_ICR_TATOCINT;
  TIMER1_IMR_R |= TIMER_IMR_TATOIM;// 6) arm timeout interrupt
								   // 7) priority shifted to bits 15-13 for timer1A
  NVIC_PRI5_R = (NVIC_PRI5_R&0xFFFF00FF)|(priority << 13);	//3
  NVIC_EN0_R = NVIC_EN0_INT21;     // 8) enable interrupt 21 in NVIC
  TIMER1_TAPR_R = 0;
  TIMER1_CTL_R |= TIMER_CTL_TAEN;  // 9) enable timer1A
	
  EndCritical(sr);
}

void Timer1A_Handler(void){ 
  TIMER1_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer1A timeout
	(*PeriodicTask1)();
}

void InitTimer2A(unsigned long period) {
	long sr;
	volatile unsigned long delay;
	
	sr = StartCritical();
  SYSCTL_RCGCTIMER_R |= 0x04;
  while((SYSCTL_RCGCTIMER_R & 0x04) == 0){} // allow time for clock to stabilize
	
  TIMER2_CTL_R &= ~TIMER_CTL_TAEN; // 1) disable timer2A during setup
                                   // 2) configure for 32-bit timer mode
  TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;
                                   // 3) configure for periodic mode, default down-count settings
  TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER2_TAILR_R = period - 1;     // 4) reload value
                                   // 5) clear timer2A timeout flag
  TIMER2_ICR_R = TIMER_ICR_TATOCINT;
  TIMER2_IMR_R |= TIMER_IMR_TATOIM;// 6) arm timeout interrupt
								   // 7) priority shifted to bits 31-29 for timer2A
  NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|(2 << 29);	
  NVIC_EN0_R = NVIC_EN0_INT23;     // 8) enable interrupt 23 in NVIC
  TIMER2_TAPR_R = 0;
  TIMER2_CTL_R |= TIMER_CTL_TAEN;  // 9) enable timer2A
	
  EndCritical(sr);
}

void Timer2A_Handler(void){ 
	int i;
	
	TIMER2_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer2A timeout
	MSTime++;
	
	for(i = 0; i < NUMTHREADS; i++) {
#ifdef aging
		if (!tcbs[i].available) { // find threads that is in using
			if (tcbs[i].sleepCt){  // sleeping threads
				tcbs[i].sleepCt -= 1;
			}
			else if (tcbs[i].blockPt == 0){  // threads that is not blocked
				tcbs[i].age++;
			}
			if ((tcbs[i].age > 8) && (tcbs[i].WorkPriority > 0)){ 
				tcbs[i].age = 0;
				tcbs[i].WorkPriority -= 1;
			}
		}
#else
		if((!tcbs[i].available) && tcbs[i].sleepCt) {
			tcbs[i].sleepCt -= 1;
		}
#endif
	}
}

void InitTimer3A(void) {
	long sr;

	sr = StartCritical();
  SYSCTL_RCGCTIMER_R |= 0x08;
  while((SYSCTL_RCGCTIMER_R & 0x08) == 0){} // allow time for clock to stabilize
	
  TIMER3_CTL_R &= ~TIMER_CTL_TAEN; // 1) disable timer3A during setup
                                   // 2) configure for 32-bit timer mode
  TIMER3_CFG_R = TIMER_CFG_32_BIT_TIMER;
                                   // 3) configure for periodic mode, default down-count settings
  TIMER3_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER3_TAILR_R = 0xFFFFFFFF - 1;     // 4) reload value
                                   // 5) clear timer3A timeout flag
  TIMER3_ICR_R = TIMER_ICR_TATOCINT;
  TIMER3_IMR_R |= TIMER_IMR_TATOIM;// 6) arm timeout interrupt
								   // 7) priority shifted to bits for timer3A
  NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)|(1 << 29);	//1
  NVIC_EN1_R = NVIC_EN1_INT35;     // 8) enable interrupt 35 in NVIC
  TIMER3_TAPR_R = 0;
  TIMER3_CTL_R |= TIMER_CTL_TAEN;  // 9) enable timer3A
	
  EndCritical(sr);
}

void Timer3A_Handler(void){ 
  TIMER3_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer1A timeout	
}

void InitTimer4A(uint32_t period, uint32_t priority) {
	long sr;
	
	sr = StartCritical();
  SYSCTL_RCGCTIMER_R |= 0x10;
	
  while((SYSCTL_RCGCTIMER_R & 0x10) == 0){} // allow time for clock to stabilize
	
  TIMER4_CTL_R &= ~TIMER_CTL_TAEN; // 1) disable timer4A during setup
                                   // 2) configure for 32-bit timer mode
  TIMER4_CFG_R = TIMER_CFG_32_BIT_TIMER;
                                   // 3) configure for periodic mode, default down-count settings
  TIMER4_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER4_TAILR_R = period - 1;     // 4) reload value
                                   // 5) clear timer4A timeout flag
  TIMER4_ICR_R = TIMER_ICR_TATOCINT;
  TIMER4_IMR_R |= TIMER_IMR_TATOIM;// 6) arm timeout interrupt
								   // 7) priority shifted to bits 15-13 for timer1A
  NVIC_PRI17_R = (NVIC_PRI17_R&0xFF00FFFF)|(priority << 21);	//3
  NVIC_EN2_R = NVIC_EN2_INT70;     // 8) enable interrupt 70 in NVIC
  TIMER4_TAPR_R = 0;
  TIMER4_CTL_R |= TIMER_CTL_TAEN;  // 9) enable timer4A
	
  EndCritical(sr);
}

void Timer4A_Handler(void){ 
  TIMER4_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer4A timeout
	(*PeriodicTask2)();
}

// Switch Tasks ------------------------------------------------------------------------

#define BUTTON1   (*((volatile uint32_t *)0x40007100))  /* PD6 */
#define BUTTON2   (*((volatile uint32_t *)0x40007200))  /* PD7 */
volatile static uint32_t Last1,Last2;


void ButtonOneInit(uint8_t priority){
  SYSCTL_RCGCGPIO_R |= 0x00000008; // 1) activate clock for Port D
  while((SYSCTL_PRGPIO_R&0x08) == 0){};// allow time for clock to stabilize
	                                 // 2) no need to unlock PD6
  GPIO_PORTD_AMSEL_R &= ~0x40;     // 3) disable analog on PD6
                                   // 4) configure PD6 as GPIO
  GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0xF0FFFFFF)+0x00000000;
  GPIO_PORTD_DIR_R &= ~0x40;       // 5) make PD6 input
  GPIO_PORTD_AFSEL_R &= ~0x40;     // 6) disable alt funct on PD6
	GPIO_PORTD_DEN_R |= 0x40;        // 7) enable digital I/O on PD6
	GPIO_PORTD_PUR_R |= 0x40;     //     enable weak pull-up on PD6
  GPIO_PORTD_IS_R &= ~0x40;     // (d) PD6 is edge-sensitive
  GPIO_PORTD_IBE_R |= 0x40;     //     PD6 is both edges
	GPIO_PORTD_ICR_R = 0x40;      // (e) clear flag6
  GPIO_PORTD_IM_R |= 0x40;      // (f) arm interrupt on PD6 *** No IME bit as mentioned in Book ***
	
  NVIC_PRI0_R = (NVIC_PRI0_R&0x1FFFFFFF)|(priority << 29); // (g) priority 2
  NVIC_EN0_R = 0x00000008;      // (h) enable interrupt 3 in NVIC  
	Last1 = BUTTON1;
}

void ButtonTwoInit(uint8_t priority){
  SYSCTL_RCGCGPIO_R |= 0x00000008; // 1) activate clock for Port D
  while((SYSCTL_PRGPIO_R&0x08) == 0){};// allow time for clock to stabilize
	GPIO_PORTD_LOCK_R = 0x4C4F434B;  // 2) unlock GPIO Port D
  GPIO_PORTD_CR_R = 0xFF;          // allow changes to PD7-0
  GPIO_PORTD_AMSEL_R &= ~0x80;     // 3) disable analog on PD7
                                   // 4) configure PD7 as GPIO
  GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0x0FFFFFFF)+0x00000000;
  GPIO_PORTD_DIR_R &= ~0x80;       // 5) make PD6 input
  GPIO_PORTD_AFSEL_R &= ~0x80;     // 6) disable alt funct on PD7
	GPIO_PORTD_DEN_R |= 0x80;        // 7) enable digital I/O on PD7
	GPIO_PORTD_PUR_R |= 0x80;     //     enable weak pull-up on PD7
  GPIO_PORTD_IS_R &= ~0x80;     // (d) PD6 is edge-sensitive
  GPIO_PORTD_IBE_R |= 0x80;     //     PD6 is both edges
	GPIO_PORTD_ICR_R = 0x80;      // (e) clear flag7
  GPIO_PORTD_IM_R |= 0x80;      // (f) arm interrupt on PD7 *** No IME bit as mentioned in Book ***
	
  NVIC_PRI0_R = (NVIC_PRI0_R&0x1FFFFFFF)|(priority << 29); // (g) priority 2
  NVIC_EN0_R = 0x00000008;      // (h) enable interrupt 3 in NVIC  
	Last2 = BUTTON2;
}

void static DebouncePD6(void) {
  OS_Sleep(10);      //foreground sleep, must run within 5ms
  Last1 = BUTTON1;
  GPIO_PORTD_ICR_R = 0x40;
  GPIO_PORTD_IM_R |= 0x40;
  OS_Kill(); 
}

void static DebouncePD7(void) {
  OS_Sleep(10);      //foreground sleep, must run within 5ms
  Last2 = BUTTON2;
  GPIO_PORTD_ICR_R = 0x80;
  GPIO_PORTD_IM_R |= 0x80;
  OS_Kill(); 
}

void GPIOPortD_Handler(void) {  // called on touch of either SW1 or SW2

	if(GPIO_PORTD_RIS_R & 0x40){   // BUTTON1 touched
		GPIO_PORTD_IM_R &= ~0x40;  //disarm interrupt on PD6
		if (Last1){
			(*ButtonOneTask)();
		}
		OS_AddThread(DebouncePD6,128,2);
	}
	else if(GPIO_PORTD_RIS_R & 0x80){  // BUTTON2 touched
		GPIO_PORTD_IM_R &= ~0x80;  //disarm interrupt on PD7
		if (Last2){
			(*ButtonTwoTask)();
		}
		OS_AddThread(DebouncePD7,128,2);
	}
}

//******** OS_AddSW1Task *************** 
// add a background task to run whenever the BUTTON1 (PD6) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
int OS_AddSW1Task(void(*task)(void), unsigned long priority) { 
	ButtonOneTask = task;
	ButtonOneInit(priority);
	return 1;
}

//******** OS_AddSW2Task *************** 
// add a background task to run whenever the BUTTON2 (PD7) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed user task will run to completion and return
// This task can not spin block loop sleep or kill
// This task can call issue OS_Signal, it can call OS_AddThread
// This task does not have a Thread ID
int OS_AddSW2Task(void(*task)(void), unsigned long priority) { 
	ButtonTwoTask = task;
	ButtonTwoInit(priority);
	return 1;
}




static uint32_t random_seed = 1;

#define RANDOM_MAX ((1u << 31u) - 1u)
uint32_t lcg_parkmiller(uint32_t *state)
{
		uint32_t N = OS_Time()&0xFFFF;			//TIMER1_TAILR_R;	//0x7fffffff
    uint32_t G = TIMER2_TAILR_R&0xFFFF;	//48271u


    /*  
        Indirectly compute state*G%N.
        Let:
          div = state/(N/G)
          rem = state%(N/G)
        Then:
          rem + div*(N/G) == state
          rem*G + div*(N/G)*G == state*G
        Now:
          div*(N/G)*G == div*(N - N%G) === -div*(N%G)  (mod N)
        Therefore:
          rem*G - div*(N%G) === state*G  (mod N)
        Add N if necessary so that the result is between 1 and N-1.
    */
    uint32_t div = *state / (N / G);  /* max : 2,147,483,646 / 44,488 = 48,271 */
    uint32_t rem = *state % (N / G);  /* max : 2,147,483,646 % 44,488 = 44,487 */

    uint32_t a = rem * G;        /* max : 44,487 * 48,271 = 2,147,431,977 */
    uint32_t b = div * (N % G);  /* max : 48,271 * 3,399 = 164,073,129 */

    return *state = (a > b) ? (a - b) : (a + (N - b));
}
/*
uint16_t next_random() 
{
	uint32_t randVal = lcg_parkmiller(&random_seed);
  return (randVal & 0xffff);
}
uint8_t next_small_random() 
{
	uint32_t randVal = next_random();
  return (randVal & 0xff);
}
*/

uint8_t getNumThreads()
{
	uint32_t randVal = next_small_random();
	uint8_t rand_num;
	if (randVal >0&&randVal<51)
	{rand_num=1;}
  else if(randVal >=51&&randVal<102)
	{rand_num=2;}
	else if(randVal >=102&&randVal<153)
	{rand_num=3;}
	else if(randVal >=153&&randVal<204)
	{rand_num=4;}
	else if(randVal >=204&&randVal<255)
	{rand_num=5;}
	return rand_num;
}
uint8_t getXY()
{
	uint32_t randVal = next_small_random();
	uint8_t rand_num;
	if (randVal >0&&randVal<42)
	rand_num=0;
  else if(randVal >=42&&randVal<84)
	rand_num=1;
	else if(randVal >=84&&randVal<126)
	rand_num=2;
	else if(randVal >=126&&randVal<168)
	rand_num=3;
	else if(randVal >=168&&randVal<210)
	rand_num=4;
	else if(randVal >=210&&randVal<255)
	rand_num=5;
	return rand_num;
}

uint8_t getDir()
{
	uint32_t randVal = next_small_random();
	uint8_t rand_num;
	if (randVal >0 &&randVal<63)
	rand_num=0;
  else if(randVal >=63&&randVal<127)
	rand_num=1;
	else if(randVal >=127&&randVal<191)
	rand_num=2;
	else 
	rand_num=3;
	return rand_num;
}

uint8_t getColors()
{
	uint32_t randVal = next_small_random();
	uint8_t rand_num;
	if (randVal >0&&randVal<23)
	rand_num=0;
  else if(randVal >=23&&randVal<46)
	rand_num=1;
	else if(randVal >=46&&randVal<69)
	rand_num=2;
	else if(randVal >=69&&randVal<92)
	rand_num=3;
	else if(randVal >=92&&randVal<115)
	rand_num=4;
	else if(randVal >=115&&randVal<138)
	rand_num=5;
	else if(randVal >=138&&randVal<161)
	rand_num=6;
	else if(randVal >=161&&randVal<184)
	rand_num=7;
	else if(randVal >=184&&randVal<207)
	rand_num=8;
	else if(randVal >=207&&randVal<230)
	rand_num=9;
	else 
	rand_num=10;
	return rand_num;
}



#define POLY_MASK_32	0xb4bcd35c
#define POLY_MASK_31	0x7a5bc2e3
unsigned int lfsr32, lfsr31;
//unsigned int POLY_MASK_32, POLY_MASK_31;

int shift_lfsr(unsigned int *lfsr, unsigned int poly_mask)
{
	int feedback;
	
	feedback = *lfsr & 1;
	*lfsr >>= 1;
	if (feedback == 1)
	{
		*lfsr ^= poly_mask;
		
	}
	return *lfsr;
}

void init_lfsrs(void)
{
	lfsr32 = OS_Time()*0xfffffff;	//0xABCDE		;
	lfsr31 = 0x234567;  //89	;
	
	
	
	/*
	lfsr32 = TIMER1_TAILR_R&0xFFFFFFFF;
	lfsr31 = TIMER2_TAILR_R&0x7FFFFFFF;
	
	POLY_MASK_32 = (lfsr32>>=1)&0x7FFFFFFF;
	POLY_MASK_31 = (lfsr31>>=1)&0x3FFFFFFF;
	*/

	}

uint16_t next_random(void)
{
	/* 	Shifts 32bit lfsr twice, then XOR's result with 
			31bit lfsr and then takes the bottom 16 bits for 
			the random value
	*/
	int random = 0;
	
	shift_lfsr(&lfsr32, POLY_MASK_32);
	random = shift_lfsr(&lfsr32, POLY_MASK_32) ^ shift_lfsr(&lfsr31, POLY_MASK_31) & 0xffff;

	
	return random;

}

uint8_t next_small_random(void)
{
	int random_val;
	
	init_lfsrs();
	
	random_val = next_random();	//Value between 0 and 65535

//	return (random_val / 256);	// Decreases to 8 bit
	return (random_val & 0xFF);	// Decreases to 8 bit
}