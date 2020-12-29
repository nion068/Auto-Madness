/*
 * AutoMadness.c
 *
 * Created: 7/16/2018 2:26:06 PM
 *  Author: nion
 */ 


#include <avr/io.h>
#include <string.h>
#include <avr/eeprom.h>
#define F_CPU 1000000UL
#include <util/delay.h>

#include <inttypes.h>		/* Include integer type header file */
#include <stdlib.h>		/* Include standard library file */
#include <stdio.h>		/* Include standard I/O library file */
#include "MPU6050_res_define.h"	/* Include MPU6050 register define file */
#include "I2C_Master_H_file.h"	/* Include I2C Master header file */

#define LCD_Dir  DDRB			/* Define LCD data port direction */
#define LCD_Port PORTB			/* Define LCD data port */
#define RS PB0				/* Define Register Select pin */
#define EN PB1 				/* Define Enable signal pin */

#define HIGH_SCORE_ADDRESS 64
#define RESTART 4
#define PAUSE 5
#define HIGH_SCORE 6
#define BUZZER 2
#define LEFT 0
#define RIGHT 1
#define NO 0
#define YES 1
#define INSERT_LEFT_STARTED 0
#define INSERT_LEFT_END 1
#define SENSITIVITY .15

int rows[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};	// row selector array
	
int printCar[16][3] = { {0xF2, 0xFF, 0xFF}, {0xF1, 0xF2, 0xF3}, {0xFF, 0xF2, 0xFF}, {0xF1, 0xF2, 0xF3}, 
						{0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF},
							{0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF},
								{0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF} };
int printCar2[16][3] = { {0xFA, 0xFF, 0xFF}, {0xF9, 0xFA, 0xFB}, {0xFF, 0xFA, 0xFF}, {0xF9, 0xFA, 0xFB},
{0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF},
{0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF},
{0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF} };	
	
int playerCarLeft[16][3] = { {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF},
							{0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF},
							{0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF},
							{0xFF, 0xAF, 0xFF}, {0x9F, 0xAF, 0xBF}, {0xFF, 0xAF, 0xFF}, {0x9F, 0xAF, 0xBF} };

int playerCarRight[16][3] = { {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF},
							{0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF},
							{0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF},  
							{0xFF, 0xDF, 0xFF}, {0xCF, 0xDF, 0xEF}, {0xFF, 0xDF, 0xFF}, {0xCF, 0xDF, 0xEF} };

int anotherCar[16][3] = { {0xFF, 0xF5, 0xFF}, {0xF4, 0xF5, 0xF6}, {0xFF, 0xF5, 0xFF}, {0xF4, 0xF5, 0xF6},
	{0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF},
{0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF},
{0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF} };

int anotherCar2[16][3] = { {0xFF, 0xFD, 0xFF}, {0xFC, 0xFD, 0xFE}, {0xFF, 0xFD, 0xFF}, {0xFC, 0xFD, 0xFE},
	{0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF},
{0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF},
{0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF} };
	
int A[6][6] = {
	{0xFF, 0xF2, 0xF3, 0xF4, 0xF5, 0xFF},
		{0xF1, 0xFF, 0xFF, 0xFF, 0xF6, 0xFF},
			{0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6},
				{0xF1, 0xFF, 0xFF, 0xFF, 0xFF, 0xF6},
					{0xF1, 0xFF, 0xFF, 0xFF, 0xFF, 0xF6},
						{0xF1, 0xFF, 0xFF, 0xFF, 0xFF, 0xF6}
};

int T[6][6] = {
	{0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6},
	{0xFF, 0xFF, 0xF3, 0xF4, 0xFF, 0xFF},
	{0xFF, 0xFF, 0xF3, 0xF4, 0xFF, 0xFF},
	{0xFF, 0xFF, 0xF3, 0xF4, 0xFF, 0xFF},
	{0xFF, 0xFF, 0xF3, 0xF4, 0xFF, 0xFF},
	{0xFF, 0xFF, 0xF3, 0xF4, 0xFF, 0xFF},
};


int displayUpper[16][6];
int displayLower[16][6];

int playerCarPosition = LEFT;

int insertCarState = 3;

int insertOpponentCarPosition;

int insertReady;

int collision = NO;

uint8_t score = 0;

uint8_t lastScore = 0;

uint8_t highScore = 0;

int level = 1;

long int cycle = 1;

int moveSpeed = 4;

int newCarInsert = 36;

int levelLength = 216;

char stringLCD[15];

float Ya;

void initialize()
{
	DDRA = 0x0F; // row selection output; A3 - A0
	
	DDRD = 0xFF; // green and red selection output; D3-D0 : red and D7-D4 : green
	
	
}

// LCD code start

void LCD_Command( unsigned char cmnd )
{
	LCD_Port = (LCD_Port & 0x0F) | (cmnd & 0xF0); /* sending upper nibble */
	LCD_Port &= ~ (1<<RS);		/* RS=0, command reg. */
	LCD_Port |= (1<<EN);		/* Enable pulse */
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);

	_delay_us(200);

	LCD_Port = (LCD_Port & 0x0F) | (cmnd << 4);  /* sending lower nibble */
	LCD_Port |= (1<<EN);
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);
	_delay_ms(2);
}


void LCD_Char( unsigned char data )
{
	LCD_Port = (LCD_Port & 0x0F) | (data & 0xF0); /* sending upper nibble */
	LCD_Port |= (1<<RS);		/* RS=1, data reg. */
	LCD_Port|= (1<<EN);
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);

	_delay_us(200);

	LCD_Port = (LCD_Port & 0x0F) | (data << 4); /* sending lower nibble */
	LCD_Port |= (1<<EN);
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);
	_delay_ms(2);
}

void LCD_Init (void)			/* LCD Initialize function */
{
	LCD_Dir = 0xFF;			/* Make LCD port direction as o/p */
	_delay_ms(20);			/* LCD Power ON delay always >15ms */
	
	LCD_Command(0x02);		/* send for 4 bit initialization of LCD  */
	LCD_Command(0x28);              /* 2 line, 5*7 matrix in 4-bit mode */
	LCD_Command(0x0c);              /* Display on cursor off*/
	LCD_Command(0x06);              /* Increment cursor (shift cursor to right)*/
	LCD_Command(0x01);              /* Clear display screen*/
	_delay_ms(2);
}


void LCD_String (char *str)		/* Send string to LCD function */
{
	int i;
	for(i=0;str[i]!=0;i++)		/* Send each char of string till the NULL */
	{
		LCD_Char (str[i]);
	}
}

void LCD_String_xy (char row, char pos, char *str)	/* Send string to LCD with xy position */
{
	if (row == 0 && pos<16)
	LCD_Command((pos & 0x0F)|0x80);	/* Command of first row and required position<16 */
	else if (row == 1 && pos<16)
	LCD_Command((pos & 0x0F)|0xC0);	/* Command of first row and required position<16 */
	LCD_String(str);		/* Call LCD string function */
}

void LCD_Clear()
{
	LCD_Command (0x01);		/* Clear display */
	_delay_ms(2);
	LCD_Command (0x80);		/* Cursor at home position */
}

// LCD code end
//gyro code start

float Acc_x,Acc_y,Acc_z,Temperature,Gyro_x,Gyro_y,Gyro_z;

void Gyro_Init()		/* Gyro initialization function */
{
	_delay_ms(150);		/* Power up time >100ms */
	I2C_Start_Wait(0xD0);	/* Start with device write address */
	I2C_Write(SMPLRT_DIV);	/* Write to sample rate register */
	I2C_Write(0x07);	/* 1KHz sample rate */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(PWR_MGMT_1);	/* Write to power management register */
	I2C_Write(0x01);	/* X axis gyroscope reference frequency */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(CONFIG);	/* Write to Configuration register */
	I2C_Write(0x00);	/* Fs = 8KHz */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(GYRO_CONFIG);	/* Write to Gyro configuration register */
	I2C_Write(0x18);	/* Full scale range +/- 2000 degree/C */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(INT_ENABLE);	/* Write to interrupt enable register */
	I2C_Write(0x01);
	I2C_Stop();
}

void MPU_Start_Loc()
{
	I2C_Start_Wait(0xD0);	/* I2C start with device write address */
	I2C_Write(ACCEL_XOUT_H);/* Write start location address from where to read */
	I2C_Repeated_Start(0xD1);/* I2C start with device read address */
}

void Read_RawValue()
{
	MPU_Start_Loc();									/* Read Gyro values */
	Acc_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Acc_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Acc_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	//Temperature = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	//Gyro_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	//Gyro_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	//Gyro_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Nack());
	I2C_Stop();
}

// gyro code end

void initializeDisplayLCD(){
	for(int i = 0; i < 16; i++){
		for(int j = 0; j < 6; j++){
			displayUpper[i][j] = 0xFF;
			displayLower[i][j] = 0xFF;
		}
	}
	
	LCD_Clear();
	
	score = 0;
	level = 1;
	moveSpeed = 4;
	newCarInsert = 36;
	
	itoa(score, stringLCD , 10);
	LCD_String("Score: ");
	LCD_String(stringLCD);
	
	LCD_Command(0xC0); // newline
	
	itoa(level, stringLCD, 10);
	LCD_String("Level: ");
	LCD_String(stringLCD);
}

int printLaneTwoCar[16][3];
int printLaneTwoCar2[16][3];


void showCar(int a[16][3], int a2[16][3])
{
	for(int i = 0; i < 16; i++){
		PORTA = rows[i];
		if(i < 8){
			for(int j = 0; j < 3; j++){
				PORTD = a[i][j];
				_delay_us(500);
			}
		}
		else{
			for(int j = 0; j < 3; j++){
				PORTD = a2[i][j];
				_delay_us(500);
			}
		}
		_delay_us(500);
	}
	
	for(int i = 15; i >= 1; i--){
		for(int j = 0; j < 3; j++){
			a[i][j] = a[i-1][j];
			a2[i][j] = a2[i-1][j];
		}
	}
	for(int j = 0; j < 3; j++){
		a[0][j] = 0xFF;
		a2[0][j] = 0xFF;
	}
}

void increaseScore()
{
	score++;
	
	LCD_Command(0x80); // cursor on home position

	itoa(score, stringLCD , 10);
	LCD_String("Score: ");
	LCD_String(stringLCD);
	
}

void increaseLevel()
{
	if(moveSpeed == 1) return;
	
	level++;
	
	moveSpeed--;
	newCarInsert = 9*moveSpeed;
	
	LCD_Command(0xC0); // cursor on 2nd line
	
	itoa(level, stringLCD, 10);
	LCD_String("Level: ");
	LCD_String(stringLCD);
	
	buzzer();
}

void showPlayerCarLeft()
{
	for(int i = 12; i < 16; i++){
		//PORTA = rows[i];
		for(int j = 0; j < 3; j++){
			displayLower[i][j] = playerCarLeft[i][j];
		}
		//for(int j = 3; j < 6; j++){
			////displayLower[i][j] = 0xFF;
			////_delay_us(700);
		//}
		//_delay_us(500);
	}
	//_delay_us(500);
}

void showPlayerCarRight()
{
	//for(int i = 12; i < 16; i++){
		//for(int j = 3; j < 6; j++){
			
			//////displayLower[i][j] = 0xFF;
			//////_delay_us(700);
		//}
	//}
	
	for(int i = 12; i < 16; i++){
		//PORTA = rows[i];
		
		for(int j = 3; j < 6; j++){
			displayLower[i][j] = playerCarRight[i][j-3];	
			
			//_delay_us(700);
		}
		if (collision == YES) break;
		//_delay_us(500);
	}
	//_delay_us(500);
}

void insertOpponentCarLeft(){
	//for(int i = 0; i < 4; i++){
		//for(int j = 0; j < 3; j++){
			//displayUpper[i][j] = printCar[i][j];
			//displayLower[i][j] = printCar2[i][j];
		//}
	//}
	if(insertCarState == 3) {
		for(int j = 0; j < 3; j++){
			displayUpper[0][j] = printCar[insertCarState][j];
			displayLower[0][j] = printCar2[insertCarState][j];
		}
		insertCarState--;
	}
	else if(insertCarState == 2) {
		for(int j = 0; j < 3; j++){
			displayUpper[0][j] = printCar[insertCarState][j];
			displayLower[0][j] = printCar2[insertCarState][j];
		}
		insertCarState--;
	}
	else if(insertCarState == 1) {
		for(int j = 0; j < 3; j++){
			displayUpper[0][j] = printCar[insertCarState][j];
			displayLower[0][j] = printCar2[insertCarState][j];
		}
		insertCarState--;
	}
	else if(insertCarState == 0) {
		for(int j = 0; j < 3; j++){
			displayUpper[0][j] = printCar[insertCarState][j];
			displayLower[0][j] = printCar2[insertCarState][j];
		}
		insertCarState = 3;
	}
}

void insertOpponentCarRight(){
	//for(int i = 0; i < 4; i++){
		//for(int j = 3; j < 6; j++){
			//displayUpper[i][j] = anotherCar[i][j-3];
			//displayLower[i][j] = anotherCar2[i][j-3];	
		//}
	//}
	if(insertCarState == 3) {
		for(int j = 3; j < 6; j++){
			displayUpper[0][j] = anotherCar[insertCarState][j-3];
			displayLower[0][j] = anotherCar2[insertCarState][j-3];
		}
		insertCarState--;
	}
	else if(insertCarState == 2) {
		for(int j = 3; j < 6; j++){
			displayUpper[0][j] = anotherCar[insertCarState][j-3];
			displayLower[0][j] = anotherCar2[insertCarState][j-3];
		}
		insertCarState--;
	}
	else if(insertCarState == 1) {
		for(int j = 3; j < 6; j++){
			displayUpper[0][j] = anotherCar[insertCarState][j-3];
			displayLower[0][j] = anotherCar2[insertCarState][j-3];
		}
		insertCarState--;
	}
	else if(insertCarState == 0) {
		for(int j = 3; j < 6; j++){
			displayUpper[0][j] = anotherCar[insertCarState][j-3];
			displayLower[0][j] = anotherCar2[insertCarState][j-3];
		}
		insertCarState = 3;
	}
}

void showDisplay()
{
	for(int i = 0; i < 16; i++){
		PORTA = rows[i];
		if(i < 8){
			for(int j = 0; j < 6; j++){
				PORTD = displayUpper[i][j];
				_delay_us(250);
			}
		}
		else{
			for(int j = 0; j < 6; j++){
				PORTD = displayLower[i][j];
				_delay_us(250);
			}
		}
		//_delay_us(200);
	}
	PORTD = 0xFF;
}

void showA()
{
	for(int i = 0; i < 6; i++){
		PORTA = rows[i+1];
		for(int j = 0; j < 6; j++){
			PORTD = A[i][j];
			_delay_us(200);
		}
		
	}
}

void showT()
{
	for(int i = 0; i < 6; i++){
		PORTA = rows[i+1];
		for(int j = 0; j < 6; j++){
			PORTD = T[i][j];
			_delay_us(200);
		}
		
	}
}


void opponentCarMove(){
	
	if(playerCarPosition == LEFT && displayLower[11][1] == 0xFA){
		collision = YES;
		return;
	}
	else if(playerCarPosition == RIGHT && displayLower[11][4] == 0xFD){
		collision = YES;
		return;
	}
	
	for(int i = 15; i > 0;i--){
		for(int j = 0; j < 6; j++){
			displayUpper[i][j] = displayUpper[i-1][j];
			displayLower[i][j] = displayLower[i-1][j];
		}
		
		if(collision == YES) break;
		
	}
	for(int j = 0; j < 6; j++){
		displayUpper[0][j] = 0xFF;
		displayLower[0][j] = 0xFF;
	}
	if(playerCarPosition == LEFT) showPlayerCarLeft();
	else showPlayerCarRight();
}

void buzzer()
{
	PORTB |= 1 << BUZZER;
	_delay_ms(20);
	PORTB &= 0 << BUZZER;
}

void gameOverBlinking()
{
	for(int i = 0; i < 20; i++){
		showDisplay();
		buzzer();
	}
}

void writeHighScore(uint8_t highScore)
{
	eeprom_update_byte((uint8_t*)HIGH_SCORE_ADDRESS, highScore);
}

uint8_t readHighScore()
{
	return eeprom_read_byte((const uint8_t*) HIGH_SCORE_ADDRESS);
}

int main(void)
{	
	
	initialize();
	
	LCD_Init();			/* Initialization of LCD*/
	I2C_Init();		/* Initialize I2C */
	Gyro_Init();		/* Initialize Gyro */
	
	initializeDisplayLCD();
	
	
	insertReady = YES;
	
	//char buffer[20], float_[10];
	//float Xa,Ya,Za,t;
	//float Xg=0,Yg=0,Zg=0;
	
	
	int prevPosition = LEFT;
	int currPosition = LEFT;
	
	int pausePressed = NO;
	int restartPressed = NO;
	
	highScore = readHighScore();
	
	if(highScore == 255) highScore = 0;
	
	uint8_t test;
	
	while(1){
		
		showDisplay();
		
		if(PINA == 0b11101111){
			restartPressed = YES;
			_delay_ms(500);
		}
		else if(PINA == 0b00111111){
			LCD_Clear();
			
			highScore = readHighScore();
			
			if(highScore == 255) highScore = 0;
			
			itoa(highScore, stringLCD, 10);
			LCD_String("High Score: ");
			LCD_String(stringLCD);
			
			LCD_Command(0xC0); // cursor on 2nd line
			
			itoa(lastScore, stringLCD, 10);
			LCD_String("Last Score: ");
			LCD_String(stringLCD);
			
			_delay_ms(2000);
			initializeDisplayLCD();
			//showPlayerCarLeft();
		}
	
		while(restartPressed == YES)
		{
		
			if(PINA == 0b11011111) {
				pausePressed = YES;
				_delay_ms(500);
			}
			else if(PINA == 0b11101111){
				initializeDisplayLCD();
				//showPlayerCarLeft();
				restartPressed = NO;
				insertCarState = 3;
				_delay_ms(500);
				break;
			}
		
			while(pausePressed == YES){
				showDisplay();
				if(PINA == 0b11011111) {
					pausePressed = NO;
					_delay_ms(500);
				}
			}
		
			Read_RawValue();

			/* Divide raw value by sensitivity scale factor */
			//Xa = Acc_x/16384.0;
			Ya = Acc_y/16384.0;
			//Za = Acc_z/16384.0;
			//Ya = 0.5;
			//Xg = Gyro_x/16.4;
			//Yg = Gyro_y/16.4;
			//Zg = Gyro_z/16.4;
	//
			///* Convert temperature in /c using formula */
			//t = (Temperature/340.00)+36.53;
		
			//dtostrf( Xa, 3, 2, float_ );
			//sprintf(buffer," Ax = %s g\t",float_);
			//
			//dtostrf( Ya, 3, 2, float_ );
			//sprintf(buffer," Ay = %s g\t",float_);
		
			showDisplay();
		
			if(prevPosition == RIGHT && Ya < -SENSITIVITY){
				currPosition = RIGHT;
			}
			else if(prevPosition == RIGHT && Ya > SENSITIVITY){
				currPosition = LEFT;
			}
			else if(prevPosition == LEFT && Ya > SENSITIVITY){
				currPosition = LEFT;
			}
			else if(prevPosition == LEFT && Ya < -SENSITIVITY){
				currPosition = RIGHT;
			}
		
			if(prevPosition != currPosition){
				if(currPosition == LEFT){
					prevPosition = currPosition;
					playerCarPosition = LEFT;
				
					for(int i = 12; i < 16; i++){
						for(int j = 0; j < 3; j++){
							if(displayLower[i][j] != 0xFF){
								collision = YES;
							}
						}
					}
				
					if(collision == NO){
						showPlayerCarLeft();
						for(int i = 12; i < 16; i++){
							//PORTA = rows[i];
							for(int j = 3; j < 6; j++){
								displayLower[i][j] = 0xFF;
								//_delay_us(700);
							}
							//_delay_us(500);
						}
					}
				
				}
				else if(currPosition == RIGHT){
					prevPosition = currPosition;
					playerCarPosition = RIGHT;
				
					for(int i = 12; i < 16; i++){
						for(int j = 3; j < 6; j++){
							if(displayLower[i][j] != 0xFF){
								collision = YES;
							}
						}
					}
					if(collision == NO){
						showPlayerCarRight();
						for(int i = 12; i < 16; i++){
							//PORTA = rows[i];
							for(int j = 0; j < 3; j++){
								displayLower[i][j] = 0xFF;
								//_delay_us(700);
							}
							//_delay_us(500);
						}
					}
					
				}
			}
		
			
		
			if(collision == YES){
				//LCD_String("GAME OVER");
				lastScore = score;
				if(score > highScore){
					writeHighScore(score);
				}
				gameOverBlinking();
				initializeDisplayLCD();
				//showPlayerCarLeft();
				insertCarState = 3;
				restartPressed = NO;
				collision = NO;
				break;
			}
		
			if(insertReady == YES && insertCarState == 3){
				insertReady = NO;
				if(rand() % 10 < 5){
					insertOpponentCarLeft();
					insertOpponentCarPosition = LEFT;
				}
				else{
					insertOpponentCarRight();
					insertOpponentCarPosition = RIGHT;
				}
				increaseScore();
			}
			else if(cycle % moveSpeed == 0 && insertCarState != 3 ){
				if(insertOpponentCarPosition == LEFT){
					opponentCarMove();
					insertOpponentCarLeft();
				}
				else{
					opponentCarMove();
					insertOpponentCarRight();
				}
			}
			else if(cycle % moveSpeed == 0){
				opponentCarMove();
			}
		
			//if(cycle % moveSpeed == 0){
				//opponentCarMove();
			//}
		
			if(cycle % newCarInsert == 0) insertReady = YES;
		
			if(cycle % levelLength == 0) increaseLevel();
		
			cycle++;
		
		
		}
		
	}
	
}