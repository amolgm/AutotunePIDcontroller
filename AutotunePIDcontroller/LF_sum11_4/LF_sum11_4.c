/*
 * LF_sum11_4.c
 *
 * Created: 10/3/2011 7:11:35 PM
 *  Author: Amol
 */ 

#define F_CPU 16000000UL

#include<avr/io.h>
#include<util/delay.h>
#include<math.h>

#define right	OCR1A
#define left	OCR1B

#define chkbit(x,y) x&(1<<y)

//float Kp = 250,Ki = 0,Kd = 3;
uint16_t adc_result_max[5] = {0,0,0,0,0};
uint16_t adc_result_min[5] = {1024,1024,1024,1024,1024};
double param[4] = {0.0,0.0,0.0,0.0};
double dp[4] = {1.0,1.0,1.0,1.0};
int prev_err_1 = 0,prev_err_2 = 0;
double prev_res = 0;
int max = 375;
int base = 115;
double sensors[5] = {-6,-2.5,0,2.5,6};


void InitADC()
{
ADMUX=(1<<REFS0);									// For Aref=AVcc;
ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);	//Rrescalar div factor =128
}

uint16_t ReadADC(uint8_t ch)
{
	//Select ADC Channel ch must be 0-7
	ch=ch&0b00000111;
	ADMUX &= 0;
	ADMUX |= ch;
	
	//Start Single conversion
	ADCSRA|=(1<<ADSC);

	//Wait for conversion to complete
	while(!(ADCSRA & (1<<ADIF)));

	//Clear ADIF by writing one to it
	//Note you may be wondering why we have write one to clear it
	//This is standard way of clearing bits in io as said in datasheets.
	//The code writes '1' but it result in setting bit to '0' !!!

	ADCSRA|=(1<<ADIF);

	return(ADC);
}


void uartinit()
{
	UCSRA=0x00;
	UCSRB=0x18;
	UCSRC=0x86;
	UBRRH=0x00;
	UBRRL=0x19;
}

char recv(void)
{
while(!(chkbit(UCSRA,RXC)));
return(UDR);
}

void trans(char data)
{
while(!(chkbit(UCSRA,UDRE)));
UDR = data;
_delay_ms(1);
}

void transtring(char name[30])
{
	unsigned int len = 0;
	
	while(name[len]!='\0')
	{	
		trans(name[len]);
		len++;
	}
	trans('\r');
}

void transnum(unsigned int num)
{
	unsigned char tenthou, thou, hund, ten, one;
	unsigned char charac[10] = {'0','1','2','3','4','5','6','7','8','9'};
	
	tenthou = num/10000;
	thou	= ( num - (tenthou*10000) )/1000;
	hund	= ( num - (tenthou*10000) - (thou*1000) )/100;
	ten		= ( num - (tenthou*10000) - (thou*1000) - (hund*100) )/10;
	one		= ( num - (tenthou*10000) - (thou*1000) - (hund*100) - (ten*10) );

	trans(charac[tenthou]);
	trans(charac[thou]);
	trans(charac[hund]);
	trans(charac[ten]);
	trans(charac[one]);
	trans('\r');
}

void connect()
{
	while(!(chkbit(UCSRA,UDRE)));
	UDR = '4';
	_delay_ms(1);
	while(!(chkbit(UCSRA,UDRE)));
	UDR = '5';
	_delay_ms(1);
	while(!(chkbit(UCSRA,UDRE)));
	UDR = '2';
	_delay_ms(1);
}

void set_motors(int left_speed,int right_speed)
{
	double temp_l = left_speed*23;
	temp_l /= 1023;
	temp_l *= 3;
	temp_l *= 5;
	temp_l += 680;
	double temp_r = right_speed*23;
	temp_r /= 1023;
	temp_r *= 3;
	temp_r *= 5;
	temp_r += 680;
	left	=	(int)temp_l;
	right	=	(int)temp_r;
}

void Calibrate()
{
	uint16_t adc_result[5] = {0,0,0,0,0};
	PORTD = 0x48;
	set_motors(50,50);	//set_motors(180,180);
	
	for (unsigned int j = 1; j<20000; j++)
	{
	for(int i=0;i<5;i++)
	{
		adc_result[i] = ReadADC(i);				// Read Analog value from channel-i

		
		if (adc_result[i]>adc_result_max[i])
		{
			adc_result_max[i] = adc_result[i];
		}
		if (adc_result[i]<adc_result_min[i])
		{
			adc_result_min[i] = adc_result[i];
		}
		
	}
	}
	
	
	PORTD = 0x88;
	set_motors(0,0);


}	

void initialise()
{
	//Defining I/O Ports
	DDRA 	=	0x00;	//Sensors	as i/p
	DDRD 	=	0xFF;	//Motors	as o/p
	PORTA	=	0xFF;	//Pull-up for sensors
	PORTD	=	0x88;	//Motors Forward

	TCCR1A	=	0xA3;	//0xA1;
  	TCCR1B	=	0x09;	//0x0A;

	left	=	0;
  	right	=	0;
	  
	uartinit();
	connect();		
	
//	transtring("Welcome!!!");
	  	  
	//Initialize ADC
	InitADC();
	Calibrate();

}


int read_line(unsigned int mode)
{
int position	= 0;
int temp = 0;

if(mode==0)
{

}
else
{
//	transnum(2000);
	uint16_t adc_result[5] = {0,0,0,0,0};
	for(uint8_t i=0; i<5; i++)
	{
		adc_result[i] = ReadADC(i);				// Read Analog value from channel-i
		adc_result[i] = adc_result[i] - adc_result_min[i];
		adc_result[i] *= 128;		
		adc_result[i] = adc_result[i]/(adc_result_max[i] - adc_result_min[i]);
		adc_result[i] *= 8;	
		
		temp += (int)sensors[i]*(int)adc_result[i];
		
//		_delay_ms(25);
//		transnum(adc_result[i]);
//		_delay_ms(25);
//		sum += (float)adc_result[i];
	}
//	transnum(adc_result[0]+adc_result[1]+adc_result[2]+adc_result[3]+adc_result[4]);
//	_delay_ms(25);
	position = temp;
	
//	transnum(position);
	
//	if (position < 0)
//		transnum((unsigned int)(-1*position));
		
//	_delay_ms(1000);
}			
		
	
return (position);
}

unsigned int follow_path(unsigned int mode)
{

int power_difference;
double pwr;
double Kp = param[0];
double Ki = param[1];
double Kd = param[2];

/*
Kp	=	0.01;//0.05;
Ki	=	0.4;
Kd	=	0.12;
*/
	
//	uartinit();
//	connect();		
	
//	transtring("Welcome!!!");

	
//	while(1)
	{

	int position = read_line(mode);

	int error	=	position/3;

	int P		=	(error);// -	prev_err_1);
	
	int I		=	(error	+	prev_err_1);///2);

	int D		=	(error	-	prev_err_1);//	+	prev_err_2);
	
	pwr	=	((double)P*Kp + (double)I*Ki + (double)D*Kd);	

	power_difference	=	(int)pwr;

//	transnum((unsigned int)power_difference);
//	_delay_ms(500);	

	


	if(power_difference > max)
	power_difference = max;
	
	if(power_difference < -max)
	power_difference = -max;

	prev_res	=	pwr;
//	prev_err_2	=	prev_err_1;
	prev_err_1	=	error;

	if(power_difference < 0)
	set_motors(base+max+power_difference,base+max);
	else
	set_motors(base+max,base+max-power_difference);

// Good on turns, but wobbly on straight
//	set_motors(max+power_difference,max-power_difference);

//	transnum((unsigned int)I);

	}

	int position = read_line(mode)/3;
	
	if (position < 0)
		return ((unsigned int)(-1*position));
		
	return (unsigned int)position;
}

int main()
{
	unsigned int mode = 1;		//mode==0 for digital and mode==1 for analog
	initialise();
	PORTD = 0x00;
	_delay_ms(1000);
	PORTD = 0x88;
	
	
	int j = 0,k;
	int acc_sum = 0, acc_vec[11];
	int err_t = 0;
	int err_t_1 = 10000;
	int flag = 0;
	int min_err = 30000, max_err = 0;

unsigned int best_err = follow_path(mode);
unsigned int err;
double sum = (dp[0]+dp[1]+dp[2]);
//transtring("sum");
//transnum((unsigned int)((dp[1]+dp[2]+dp[3])*1000));
//transnum((unsigned int)(sum*1000));
	
while(sum > 0.000000001)
{
	for (int i=0;i<3;i++)
	{
		param[i] += dp[i];
		err = follow_path(mode);
		
		if (err<best_err)
		{
			best_err = err;
			dp[i] *= 1.1;
		} 
		else
		{
			param[i] -= 2*dp[i];
			err = follow_path(mode);
			
			if (err<best_err)
			{
				best_err = err;
				dp[i] *= 1.1;
			} 
			else
			{
				param[i] += dp[i];
				dp[i] *= 0.9;
			}
		}
	}		

	sum = (dp[0]+dp[1]+dp[2]);

}
PORTD = 0x00;
_delay_ms(2000);
PORTD = 0x88;

while(1)
{
	follow_path(mode);
}	

	while(1);
	return 0;
}

/*	
	if (j<10)
	{
		acc_vec[j] = follow_path(mode);
		
		if (acc_vec[j]>max_err)
		{
			max_err = acc_vec[j];
		}
		if (acc_vec[j]<min_err)
		{
			min_err = acc_vec[j];
		}
		
	}
	
	j += 1;
	
	if (j==10 || flag==1)
	{
		j = 0;
		for(k=0;k<10;k++)
		{
			acc_sum += acc_vec[k];
		}
		err_t = acc_sum/10;
		if (err_t < err_t_1)
		{
			max += 10;
		}
		else
		{
			max -= 10;
		}
					
		err_t_1 = err_t;
		acc_sum = 0;
		flag = 1;			
	}
*/	
/*	1st method
	if (j<10)
	{
		acc_sum += follow_path(mode);
	}
	
	if (j==10)
	{
		j = 0;
		err_t = acc_sum/10;
		if (err_t < err_t_1)
		{
			max += 10;
		}
		else
		{
			max -= 10;
		}
		err_t_1 = err_t;
		acc_sum = 0;			
	}
*/	