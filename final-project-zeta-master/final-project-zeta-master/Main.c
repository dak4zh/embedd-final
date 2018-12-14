// Main.c
// Runs on LM4F120/TM4C123
// You may use, edit, run or distribute this file 
// You are free to change the syntax/organization of this file

// Jonathan W. Valvano 2/20/17, valvano@mail.utexas.edu
// Modified by Sile Shu 10/4/17, ss5de@virginia.edu
// Modified by Mustafa Hotaki 7/29/18, mkh3cf@virginia.edu

#include <stdint.h>
#include "OS.h"
#include "tm4c123gh6pm.h"
#include "LCD.h"
#include <string.h> 
#include "UART.h"
#include "FIFO.h"
#include "joystick.h"
#include "PORTE.h"

// Constants
#define BGCOLOR     					LCD_BLACK
#define CROSSSIZE            	5
#define PERIOD               	4000000   // DAS 20Hz sampling period in system time units
#define PSEUDOPERIOD         	8000000
#define LIFETIME             	1000
#define RUNLENGTH            	600 // 30 seconds run length

void OS_DisableInterrupts(void);	// Disable interrupts
void OS_EnableInterrupts(void);  	// Enable interrupts

int COLORS[] = {LCD_BLUE,LCD_DARKBLUE,LCD_RED,LCD_GREEN,LCD_LIGHTGREEN,LCD_ORANGE,LCD_CYAN,LCD_MAGENTA,LCD_YELLOW,LCD_WHITE,LCD_GREY};
int x1_pos, y1_pos, color1;
int x2_pos, y2_pos, color2;
int x3_pos, y3_pos, color3;
int x4_pos, y4_pos, color4;

//Globals for score and life
uint16_t score = 0; 
uint16_t life = 3;

extern Sema4Type LCDFree;
uint16_t origin[2]; 	// The original ADC value of x,y if the joystick is not touched, used as reference
int16_t x = 63;  			// horizontal position of the crosshair, initially 63
int16_t y = 63;  			// vertical position of the crosshair, initially 63
int16_t prevx, prevy;	// Previous x and y values of the crosshair
uint8_t select;  			// joystick push
uint8_t area[2];
uint32_t PseudoCount;

unsigned long NumCreated;   		// Number of foreground threads created
unsigned long NumSamples;   		// Incremented every ADC sample, in Producer
unsigned long UpdateWork;   		// Incremented every update on position values
unsigned long Calculation;  		// Incremented every cube number calculation


#define VERTICALNUM 6
#define HORIZONTALNUM 6
typedef struct {
	uint32_t position[2];
	Sema4Type BlockFree;
} block;
block BlockArray[HORIZONTALNUM][VERTICALNUM];


typedef struct {
	uint32_t position[2];
	int direction;
	int lifetime;
	int hit;
	int expired;
	int color;
	int available;
} cube;


/*
struct cube {
	uint32_t position[2];
	int direction;
	int lifetime;
	int hit;
	int expired;
	int color;
};
*/

cube CubeArray[20];
cube cube1;
cube cube2;






//---------------------User debugging-----------------------
unsigned long DataLost;     // data sent by Producer, but not received by Consumer
long MaxJitter;             // largest time jitter between interrupts in usec
#define JITTERSIZE 64
unsigned long const JitterSize=JITTERSIZE;
unsigned long JitterHistogram[JITTERSIZE]={0,};
unsigned long TotalWithI1;
unsigned short MaxWithI1;

void Device_Init(void){
	UART_Init();
	BSP_LCD_OutputInit();
	BSP_Joystick_Init();
}
//------------------Task 1--------------------------------
// background thread executed at 20 Hz
//******** Producer *************** 
int UpdatePosition(uint16_t rawx, uint16_t rawy, jsDataType* data){
	if (rawx > origin[0]){
		x = x + ((rawx - origin[0]) >> 9);
	}
	else{
		x = x - ((origin[0] - rawx) >> 9);
	}
	if (rawy < origin[1]){
		y = y + ((origin[1] - rawy) >> 9);
	}
	else{
		y = y - ((rawy - origin[1]) >> 9);
	}
	if (x > 127){
		x = 127;}
	if (x < 0){
		x = 0;}
	if (y > 120 - CROSSSIZE){
		y = 120 - CROSSSIZE;}
	if (y < 0){
		y = 0;}
	data->x = x; data->y = y;
	return 1;
}

void Producer(void){
	uint16_t rawX,rawY; // raw adc value
	uint8_t select;
	jsDataType data;
	unsigned static long LastTime;  // time at previous ADC sample
	unsigned long thisTime;         // time at current ADC sample
	long jitter;                    // time between measured and expected, in us
	if (NumSamples < RUNLENGTH){
			BSP_Joystick_Input(&rawX,&rawY,&select);
		thisTime = OS_Time();       // current time, 12.5 ns
		UpdateWork += UpdatePosition(rawX,rawY,&data); // calculation work
		NumSamples++;               // number of samples
		if(JsFifo_Put(data) == 0){ // send to consumer
			DataLost++;
		}
	//calculate jitter
		if(UpdateWork > 1){    // ignore timing of first interrupt
			unsigned long diff = OS_TimeDifference(LastTime,thisTime);
			if(diff > PERIOD){
				jitter = (diff-PERIOD+4)/8;  // in 0.1 usec
			}
			else{
				jitter = (PERIOD-diff+4)/8;  // in 0.1 usec
			}
			if(jitter > MaxJitter){
				MaxJitter = jitter; // in usec
			}       // jitter should be 0
			if(jitter >= JitterSize){
				jitter = JITTERSIZE-1;
			}
			JitterHistogram[jitter]++; 
		}
		LastTime = thisTime;
	}
}

//--------------end of Task 1-----------------------------

//------------------Task 2--------------------------------
// background thread executes with SW1 button
// one foreground task created with button push
// foreground treads run for 1 sec and die
// ***********ButtonWork*************
void ButtonWork(void){
	uint32_t StartTime,CurrentTime,ElapsedTime;
	StartTime = OS_MsTime();
	ElapsedTime = 0;
	OS_bWait(&LCDFree);
	BSP_LCD_FillScreen(BGCOLOR);
	while (ElapsedTime < LIFETIME){

		CurrentTime = OS_MsTime();
		ElapsedTime = CurrentTime - StartTime;
		BSP_LCD_Message(0,5,0,"Life Time:",LIFETIME);
		BSP_LCD_Message(1,0,0,"Horizontal Area:",area[0]);
		BSP_LCD_Message(1,1,0,"Vertical Area:",area[1]);
		BSP_LCD_Message(1,2,0,"Elapsed Time:",ElapsedTime);
		OS_Sleep(50);
	}
	BSP_LCD_FillScreen(BGCOLOR);
	OS_bSignal(&LCDFree);
  OS_Kill();  // done, OS does not return from a Kill
} 

//************SW1Push*************
// Called when SW1 Button pushed
// Adds another foreground task
// background threads execute once and return
void SW1Push(void){
  if(OS_MsTime() > 20 ){ // debounce
    if(OS_AddThread(&ButtonWork,128,4)){
      NumCreated++; 
    }
    OS_ClearMsTime();  // at least 20ms between touches
  }
}

//--------------end of Task 2-----------------------------

//------------------Task 3--------------------------------

//******** Consumer *************** 
// foreground thread, accepts data from producer
// Display crosshair and its positions
// inputs:  none
// outputs: none
void Consumer(void){
	while(NumSamples < RUNLENGTH){

		jsDataType data;
		JsFifo_Get(&data);
		OS_bWait(&LCDFree);
			
		BSP_LCD_DrawCrosshair(prevx, prevy, LCD_BLACK); // Draw a black crosshair
		BSP_LCD_DrawCrosshair(data.x, data.y, LCD_RED); // Draw a red crosshair

		BSP_LCD_Message(1, 5, 1, "Score ", score);		
		BSP_LCD_Message(1, 5, 12, "Life ", life);
		
		//BSP_LCD_DrawCube(x1_pos, y1_pos, color1);
		
		
		//BSP_LCD_Message(1, 5, 3, "X: ", x);		
		//BSP_LCD_Message(1, 5, 12, "Y: ", y);
		OS_bSignal(&LCDFree);
		prevx = data.x; 
		prevy = data.y;
		
				if (life == 0){
					break;
				}
		
	}
  OS_Kill();  // done
}


//--------------end of Task 3-----------------------------

void CubeNumCalc(void){ 
	uint16_t CurrentX,CurrentY;
  while(1) {
		if(NumSamples < RUNLENGTH){
			CurrentX = x; CurrentY = y;
			area[0] = CurrentX / 22;
			area[1] = CurrentY / 20;
			Calculation++;
		}
  }
}



// This task implements the motions of the cubes
static uint32_t CubeNum = 0;
void CubeThread1 (void){

	unsigned char i,j;	 
	int32_t status,cube;
	int oldx, oldy, olddir ,a,b, newdir;
	int incriment = 2;
	int rcolor = next_small_random()%11;

	
	if (CubeNum == 20){ // no available cubes
		//return 0;
		//This condition should never be true
  }
	else{
			for (i=0;i<20;i++){
				if (CubeArray[i].available) break;   // find an available tcb for the new thread
			}
			cube = i;
			
			/*
			getXY();
			getXY();
			getXY();
			getXY();
			getXY();
			//getXY();
			//getXY();
			getDir();
			getDir();
			getDir();
			getDir();
			getNumThreads();
			getNumThreads();
			getNumThreads();
*/
			
			CubeArray[i].available = 0; // make this tcb no longer available
			CubeArray[i].position[0]= next_small_random()%5; //0-5
			CubeArray[i].position[1]= 0; //0-5
			CubeArray[i].direction= 3; //0-3
			CubeArray[i].color= COLORS[rcolor]; //0-10 //DONE
			CubeArray[i].lifetime = next_small_random()%25; //getNumThreads()*2 + 10; //
			CubeArray[i].hit = 0;
			CubeArray[i].expired = 0;
			
			
			 //SEMAPHORES
		for(a = 0; a < VERTICALNUM; a++){
			for(b = 0; b < HORIZONTALNUM; b++){
				OS_bSignal(&BlockArray[a][b].BlockFree); // initial available
			}  
		} 
			
			
			/*
		if (i ==0){
			CubeArray[i].available = 0; // make this tcb no longer available
			CubeArray[i].position[0]= 4; //0-5
			CubeArray[i].position[1]= 0; //0-5
			CubeArray[i].direction= 3; //0-3
			CubeArray[i].color= COLORS[0]; //0-10 //DONE
			CubeArray[i].lifetime = 20; //
			CubeArray[i].hit = 0;
			CubeArray[i].expired = 0;
		}
		else if (i ==1){
			CubeArray[i].available = 0; // make this tcb no longer available
			CubeArray[i].position[0]= 4; //0-5
			CubeArray[i].position[1]= 5; //0-5
			CubeArray[i].direction= 0; //0-3
			CubeArray[i].color= COLORS[2]; //0-10 //DONE
			CubeArray[i].lifetime = 20; //
			CubeArray[i].hit = 0;
			CubeArray[i].expired = 0;
		}
		else if (i ==2){
			CubeArray[i].available = 0; // make this tcb no longer available
			CubeArray[i].position[0]= 0; //0-5
			CubeArray[i].position[1]= 0; //0-5
			CubeArray[i].direction= 3; //0-3
			CubeArray[i].color= COLORS[rcolor]; //0-10 //DONE
			CubeArray[i].lifetime = 20; //
			CubeArray[i].hit = 0;
			CubeArray[i].expired = 0;
		}
		*/
		
		//OS_bWait(&BlockArray[CubeArray[i].position[0]][CubeArray[i].position[1]].BlockFree);
			
		
		CubeNum++;
		
	}
	


	
	//jsDataType data;
	//JsFifo_Get(&data); 
	//cube cube1;
	
	/*
	
	cube1.position[0]= 0;
	cube1.position[1]= 0;
	cube1.direction= 3;
	cube1.color= COLORS[7];
	cube1.lifetime = 10;
	cube1.hit = 0;
	cube1.expired = 0;
	*/
	
 // 1.allocate an idle cube for the object
 // 2.initialize color/shape and the first direction
 // 3.move the cube while it is not hit or expired
	while(life){ // Implement until the game is over
		while( CubeArray[i].hit == 0 && !(CubeArray[i].expired) ){
		// first, check if the object is hit by the crosshair
			if(area[0]== CubeArray[i].position[0] && area[1]== CubeArray[i].position[1]){
			//if(cube1.hit){
			// Increase the score
				OS_bWait(&LCDFree);
				BSP_LCD_DrawCube(CubeArray[i].position[0], CubeArray[i].position[1], LCD_BLACK);
				OS_bSignal(&LCDFree);
				
				score++;
				
				CubeArray[i].hit = 1;
				//OS_bSignal(&CubeArray[i][j].CubeFree); //COME BACK TO THIS!!!!!!!!!!!!!
			}
 // second, check if the object is expired
			else if (CubeArray[i].lifetime == 0 ){
				CubeArray[i].expired = 1;
				OS_bWait(&LCDFree);
				BSP_LCD_DrawCube(CubeArray[i].position[0], CubeArray[i].position[1], LCD_BLACK);
				OS_bSignal(&LCDFree);
				life--;

				//OS_bSignal(&CubeArray[i][j].CubeFree); //COME BACK TO THIS!!!!!!!!!!!!!
			}
			else{
				
	// if the object is neither hit nor expired,
 // update the cube information
 // then, display the object
 // last,decide next direction
				
				//BlockArray[a][b].BlockFree
				
				oldx = CubeArray[i].position[0];
				oldy = CubeArray[i].position[1];
				olddir = CubeArray[i].direction;
				
				//CubeArray[i].lifetime--;
				
					
				
				
				// if (CubeArray[i].direction == 3){

					CubeArray[i].position[1]++;
					if (CubeArray[i].position[1] == 5){
						//while (CubeArray[i].direction == olddir){
							//Insert random number generator
							OS_bWait(&LCDFree);
						  BSP_LCD_DrawCube(oldx, oldy, LCD_BLACK);
				//    BSP_LCD_DrawCube(CubeArray[i].position[0], CubeArray[i].position[1], CubeArray[i].color);	
				
				//Signal
				    // OS_bSignal(&LCDFree);
              	//	OS_Sleep(100);	;	
							//	OS_bWait(&LCDFree);
					CubeArray[i].position[0]= next_small_random()%5; //0-5
			    CubeArray[i].position[1]= 0;
					BSP_LCD_DrawCube(CubeArray[i].position[0], CubeArray[i].position[1], CubeArray[i].color);	
				
				//Signal
					OS_bSignal(&LCDFree);
					OS_Sleep(100);
						
             //  if( score > 20 && randcubes==0 ) {
    /*    for(i = 0; i < randcubes; i++){
		CubeArray[i].available = 1; // initial available
	}*/

    //   for (i=0;i<randcubes;i++){
		//NumCreated += OS_AddThread(&CubeThread1,128,2);
	//}
     // }
     
					}	
					else {
					     
					OS_bWait(&LCDFree);
					BSP_LCD_DrawCube(oldx, oldy, LCD_BLACK);
					BSP_LCD_DrawCube(CubeArray[i].position[0], CubeArray[i].position[1], CubeArray[i].color);	
				
				//Signal
					OS_bSignal(&LCDFree);
					OS_Sleep(100);	
					}
				//}
					
					



					//OS_bWait(&BlockArray[CubeArray[i].position[0]][CubeArray[i].position[1]].BlockFree);
					
					//OS_bSignal(&BlockArray[oldx][oldy].BlockFree);
					
					//CubeArray[i].position[1]++;

			//	}
				
				//OS_bSignal(&BlockArray[oldx][oldy].BlockFree);
			//	OS_bWait(&BlockArray[CubeArray[i].position[0]][CubeArray[i].position[1]].BlockFree);
				//OS_bSignal(&BlockArray[oldx][oldy].BlockFree);
				
				/*
				
				if (CubeArray[i].direction == 0){	
					OS_bWait(&BlockArray[oldx][oldy-1].BlockFree);
					CubeArray[i].position[1]--;
					OS_bSignal(&BlockArray[oldx][oldy].BlockFree);
					if (CubeArray[i].position[1]==0){
						while(CubeArray[i].direction == olddir){
							//Insert random number generator
							CubeArray[i].direction = 3; // Insert random number generator							
						}
					}
				}
				else if (CubeArray[i].direction == 1){
					OS_bWait(&BlockArray[oldx-1][oldy].BlockFree);
					CubeArray[i].position[0]--;
					OS_bSignal(&BlockArray[oldx][oldy].BlockFree);
					
					//CubeArray[i].position[0]--;
					if (CubeArray[i].position[0] == 0){
						while (CubeArray[i].direction == olddir){
							//Insert random number generator
							CubeArray[i].direction = 2; // Insert random number generator
						}
					}
				}
				else if (CubeArray[i].direction == 2){
					OS_bWait(&BlockArray[oldx+1][oldy].BlockFree);
					CubeArray[i].position[0]++;
					OS_bSignal(&BlockArray[oldx][oldy].BlockFree);
					
					//CubeArray[i].position[0]++;
					if (CubeArray[i].position[0] == 5){
						while (CubeArray[i].direction == olddir){
							//Insert random number generator
							CubeArray[i].direction = 1; // Insert random number generator
						}
					}
				}
				else if (CubeArray[i].direction == 3){
					OS_bWait(&BlockArray[oldx][oldy+1].BlockFree);
					CubeArray[i].position[1]++;
					OS_bSignal(&BlockArray[oldx][oldy].BlockFree);
					
					//CubeArray[i].position[1]++;
					if (CubeArray[i].position[1] == 5){
						while (CubeArray[i].direction == olddir){
							//Insert random number generator
							CubeArray[i].direction = 0; // Insert random number generator
						}
					}
				}
				*/				
				
				//Wait(insert semaphore)
				//OS_bWait(&LCDFree);
			//	BSP_LCD_DrawCube(oldx, oldy, LCD_BLACK);
			//	BSP_LCD_DrawCube(CubeArray[i].position[0], CubeArray[i].position[1], CubeArray[i].color);	
				
				//Signal
			//	OS_bSignal(&LCDFree);
				
				//OS_bSignal(&BlockArray[oldx][oldy].BlockFree);
				
				if (life == 0){
					break;
				}
				
				
				
				
				
			}
		}
		OS_Kill(); // Cube should disappear, kill the thread
	}
	OS_Kill(); //Life = 0, game is over, kill the thread
}






//--------------end of Task 5-----------------------------


void CrossHair_Init(void){
	BSP_LCD_FillScreen(BGCOLOR);
	BSP_Joystick_Input(&origin[0],&origin[1],&select);
}

//******************* Main Function**********
int main(void){

	
	//int randcubes = getNumThreads(); // INSERT RANDOM NUMBER GENERATOR
	int randcubes = 3;
	int i,j,a,b;
	//score = 0;
	//life = 0;
  OS_Init();           // initialize, disable interrupts
	Device_Init();
  CrossHair_Init();
  DataLost = 0;        // lost data between producer and consumer
  NumSamples = 0;
  MaxJitter = 0;       // in 1us units
	
	getNumThreads();
	getNumThreads();
	getNumThreads();

//********initialize communication channels
  JsFifo_Init();

//*******attach background tasks***********
  OS_AddSW1Task(&SW1Push,2);
  OS_AddPeriodicThread(&Producer,PERIOD,1); // 2 kHz real time sampling of PD3
	
	/*
		for(a = 0; a < VERTICALNUM; a++){
			for(b = 0; b < HORIZONTALNUM; b++){
				BlockArray[a][b].BlockFree = 1; // initial available
			}  
		} 
*/		
	
	for(i = 0; i < randcubes; i++){
		CubeArray[i].available = 1; // initial available
	}  
	
	
  NumCreated = 0 ;
// create initial foreground threads

  NumCreated += OS_AddThread(&Consumer,128,1); 
	NumCreated += OS_AddThread(&CubeNumCalc,128,1);
	for (i=0;i<randcubes;i++){
		NumCreated += OS_AddThread(&CubeThread1,128,2);
	}


	
 
 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
	return 0;            // this never executes
}
