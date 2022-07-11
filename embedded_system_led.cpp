#include <stdio.h>
#include <string.h>
#include<machine.h>
#include "3687.h"

#define SCI_BPS_57600		10

#define SCI_TXBUF_SIZE		256	
#define SCI_STAT_TXFULL		0x02
#define SCI_RXBUF_SIZE		64


int	sci_rxcnt;
int	sci_txcnt;
int	sci_rxtop;
int	sci_rxend;
int	sci_txtop;
int	sci_txend;
unsigned char sci_stat;
unsigned char sci_rxbuf[SCI_RXBUF_SIZE];
unsigned char sci_txbuf[SCI_TXBUF_SIZE];

#define SCI_STAT_OER 0x20
#define SCI_STAT_FER 0x10
#define SCI_STAT_PER 0x08
#define SCI_STAT_TXFULL 0x02
#define SCI_STAT_RXFULL 0x01

#define SMP_TIME10 0.010f
#define PI 3.1415926535897932f

#ifdef __cplusplus
extern "C" {
void abort(void);
}
#endif

void init(void);
void main(void);
void int_tim_b1(void);
int sci_txbuf_chk(void);
int sci_puts(char *str);
void sci_txbuf_set(unsigned char data);
void int_sci(void);
int sci_rxbuf_get(unsigned char *data);

#ifdef __cplusplus
void abort(void)
{

}
#endif

long c_time;

unsigned char sci_stat;

typedef struct{
	float time;
	float target;
} tbl_target;

void control_pid(void);
void control_log(void);
float TBL_target(float sec,tbl_target *tbl);
float TBL_linear(float sec,tbl_target *tbl);

#define M_CONT_H1  IO.PDR6.BIT.B0
#define M_CONT_H2  IO.PDR6.BIT.B1
#define M_CONT_L1  IO.PDR6.BIT.B2
#define M_CONT_L2  IO.PDR6.BIT.B3
#define M_PWM_C0   TZ0.GRC
#define M_PWM_D0   TZ0.GRD

#define PWM_CONST   100

#define DEAD_TIME   10

char c_start=0;
char c_log=0;
long c_time=0;
float theta_ref;
int potentio_ad=0;
float theta;
float theta_error=0.0f;
float theta_error_1=0.0f;
float theta_error_sum=0.0f;
float theta_error_dot=0.0f;

float theta_dot=0.0f;
float theta_1=0.0f;

//ゲイン//
float u_pwm=0.0f;
const float Kp=30.0f;
const float Ki=0.0f;
const float Kd=1.0f;

long  log_c_time;
float log_theta_ref;
int   log_potentio_ad;
float log_theta;
float log_theta_error_sum;
float log_theta_error_dot;

float log_theta_dot=0;

float log_u_pwm;

void tim_b1_init(void);
void motor_pwm_init(void);
void motor_pwm_set(long duty);
void sci_init(int baud);
void adc_init(void);
void adc_potentio(void);

const tbl_target  tbl_target_angle[]={
			0,				0,
			4.99f,			0,
			5,				PI/2,
			10,				PI/2,
			15,				0,
			20,				0,
			-1,				-1,		
};

const tbl_target  tbl_potentio_angle[]={
			0,				0,
			145,			PI/4,
			282,			PI/2,
			446,			PI*3/4,
			646,			PI,
			-1,				-1,	
};

void init(void)
{
	IO.PCR5=0xFF;
	
	IO.PCR6=0xFF;
	
	IO.PMR1.BIT.TXD=1;
	sci_init(SCI_BPS_57600);
	set_imask_ccr(0);
	tim_b1_init();
	adc_init();
	motor_pwm_init();
}

void log_send(int sel){
	
	int len;
	char buff[256];
	
	buff[0]='\0';
	if(!sel){
		sprintf(buff,"time,theta_ref,potentio_ad,theta\r\n");
	}
	else if(c_log){
		sprintf(buff,"%ld,%ld,%d,%ld\r\n",
		log_c_time,
		(long)(log_theta_ref*100000),
		log_potentio_ad,
		(long)(log_theta*100000)
		);
		c_log=0;	
	}
	
	if('\0'!=buff[0]){
		len=strlen(buff);
		if(len<=sci_txbuf_chk()){
			sci_puts(buff);
		}	
	}	
}

int potentio_ad;
void adc_init(void){
	AD.ADCSR.BIT.SCAN=0;
	AD.ADCSR.BIT.CKS=1;
}

void sci_init(int baud){
	volatile int i;
	
	sci_rxcnt=0;
	sci_txcnt=0;
	sci_rxtop=0;
	sci_rxend=0;
	sci_txtop=0;
	sci_txend=0;
	sci_stat=0;
	
	SCI3.SCR3.BYTE=0x00;
	SCI3.SMR.BYTE=0x00;
	SCI3.BRR=(unsigned char)baud;
	
	for(i=baud*8;i>0;i--);
	SCI3.SSR.BYTE&=0x00;
	SCI3.SCR3.BYTE=0x70;
}

int sci_txbuf_chk(void){
	int size;
	SCI3.SCR3.BIT.TIE=0;
	size=SCI_TXBUF_SIZE-sci_txcnt-1;
	SCI3.SCR3.BIT.TIE=1;
	
	return size;
}

int sci_puts(char *str){
	int size;
	size=strlen(str);
	if(size>sci_txbuf_chk()){
		return 1;
	}
	while(*str){
		sci_txbuf_set((unsigned char)(*str));
		str++;
	}
	return 0;
}

void sci_txbuf_set(unsigned char data){
	int end;
	SCI3.SCR3.BIT.TIE=0;
	end=(sci_txend>=SCI_TXBUF_SIZE-1)? 0 : sci_txend+1;
	if(sci_txtop!=end){
		sci_txbuf[sci_txend]=data;
		sci_txend=end;
		sci_txcnt++;
	}
	else{
		sci_stat|=SCI_STAT_TXFULL;
	}
	SCI3.SCR3.BIT.TIE=1;
}


int sci_rxbuf_get(unsigned char *data){
	int ret;
	
	SCI3.SCR3.BIT.RIE=0;

	if(sci_rxtop!=sci_rxend){
		*data=sci_rxbuf[sci_rxtop];
		sci_rxtop=(sci_rxtop>=SCI_RXBUF_SIZE-1)? 0 : sci_rxtop+1;
		sci_rxcnt--;
		ret=1;
	}
	else{
		ret=0;
	}
	SCI3.SCR3.BIT.RIE=1;
	return ret;
}

#pragma interrupt(INT_SCI3)
void int_sci(void){
	int end;
	unsigned char ssr;
	unsigned char rdr;
	unsigned char tdr;
	
	ssr=SCI3.SSR.BYTE;
	if(SCI3.SCR3.BIT.RIE){
		if(0x38&ssr){
			sci_stat|=(ssr&0x38);
			SCI3.SSR.BYTE&=0xC7;
		}
		else if(0x40&ssr){
			rdr=SCI3.RDR;
			
			end=(sci_rxend>=SCI_RXBUF_SIZE-1)?0:sci_rxend+1;
			if(sci_rxtop!=end){
				sci_rxbuf[sci_rxend]=rdr;
				sci_rxend=end;
				sci_rxcnt++;
			}
			else{
				sci_stat|=SCI_STAT_RXFULL;
			}
		}
	}
	
	
	if(SCI3.SCR3.BIT.TIE){
		if(0x80&ssr){
			if(sci_txcnt){
				tdr=sci_txbuf[sci_txtop];
				SCI3.TDR=tdr;
				
				sci_txtop=(sci_txtop>=SCI_TXBUF_SIZE-1)?0:sci_txtop+1;
				sci_txcnt--;
			}
			else{
				SCI3.SCR3.BIT.TIE=0;
			}
		}
	}
}
		
unsigned char sci_status(void){
	return sci_stat;
}

void tim_b1_init(void){
	TB1.TMB1.BYTE=0xF9;
	TB1.TCB1=158;
	
	IENR2.BIT.IENTB1=1;
}

#pragma interrupt(INT_TimerB1)
void int_tim_b1(void){
	
	IRR2.BIT.IRRTB1=0;
	
	if(c_start){
		control_pid();
		control_log();
		c_time+=10;
	}
	
}

void control_pid(void)
{
	theta_ref=TBL_target((float)c_time/1000,tbl_target_angle);
	adc_potentio();
	theta=TBL_linear((float)potentio_ad, tbl_potentio_angle);
	theta_error=theta_ref-theta;
	theta_error_sum+=(theta_error+theta_error_1)*SMP_TIME10/2;
	theta_error_dot=(theta_error-theta_error_1)/SMP_TIME10;
	theta_error_1=theta_error;
	
	u_pwm=Kp*theta_error;
	u_pwm+=Ki*theta_error_sum;
	u_pwm+=Kd*theta_error_dot;
	
	theta_dot=(theta-theta_1)/SMP_TIME10;
	theta_1=theta;
	
	motor_pwm_set((long)u_pwm);
			
}

void motor_pwm_init(void){
	M_CONT_H1=0;
	M_CONT_H2=0;
	M_CONT_L1=0;
	M_CONT_L2=0;
	
	TZ0.TCR.BYTE=0x23;
	TZ.TPMR.BYTE=0x8E;
	TZ.TOCR.BYTE=0x00;
	TZ0.GRA=PWM_CONST-1;
	TZ0.GRC=PWM_CONST-1;
	TZ0.GRD=PWM_CONST-1;
	TZ.TOER.BYTE=0xF3;
	TZ.TSTR.BIT.STR0=1;
}

void motor_pwm_set(long duty){
	volatile int cnt;
	unsigned char m_cont_h1;
	unsigned char m_cont_h2;
	
	if(duty>0){
		m_cont_h1=1;
		m_cont_h2=0;
	}
	else if(duty<0){
		m_cont_h1=0;
		m_cont_h2=1;
	}
	else{
		m_cont_h1=0;
		m_cont_h2=0;
	}
	
	if((m_cont_h1 !=M_CONT_H1)||(m_cont_h2 !=M_CONT_H2)){
		M_CONT_H1=0;
		M_CONT_H2=0;
		M_PWM_C0=PWM_CONST - 1;
		M_PWM_D0=PWM_CONST - 1;
		
		for(cnt=DEAD_TIME;cnt>0;cnt--);
			M_CONT_H1=m_cont_h1;
			M_CONT_H2=m_cont_h2;
		}
		
		if(duty>0){
			if(duty>=PWM_CONST){
				M_PWM_C0=PWM_CONST-1;
				M_PWM_D0=PWM_CONST+1;
			}
			else{
				M_PWM_C0=PWM_CONST-1;
				M_PWM_D0=duty-1;
			}
		}
		else if(duty<0){
			duty=-duty;
			if(duty>=PWM_CONST){
				M_PWM_C0=PWM_CONST+1;
				M_PWM_D0=PWM_CONST-1;
			}
			else{
				M_PWM_C0=duty-1;
				M_PWM_D0=PWM_CONST-1;
			}
		}
		else{
			M_PWM_C0=PWM_CONST-1;
			M_PWM_D0=PWM_CONST-1;
		}
	}
			
				

void control_log(void)
{
	if(!c_log){
		log_c_time=c_time;
		log_theta_ref=theta_ref;
		log_potentio_ad=potentio_ad;
		log_theta=theta;
		log_theta_dot=theta_dot;
		log_theta_error_sum=theta_error_sum;
		log_theta_error_dot=theta_error_dot;
		log_u_pwm=u_pwm;
		
		c_log=1;
	}
}

void adc_potentio(void){
	AD.ADCSR.BIT.ADST=0;
	AD.ADCSR.BIT.CH  =2;
	AD.ADCSR.BIT.ADST=1;
	
	IO.PDR5.BYTE=(AD.ADDRC>>8);
	
	while(AD.ADCSR.BIT.ADF==0){}
	
	potentio_ad=(int)(AD.ADDRC>>6);
}	

float TBL_target(float sec,tbl_target *tbl)
{
	int i;
	float a,b;
	
	i=0;
	while(1){
		if(sec>tbl[i].time){
			if(0>tbl[i].time){
				if(i==0){
					return 0;
				}
				else if(i==1){
					a=tbl[i-1].target/tbl[i-1].time;
					b=0;
					return a*sec+b;
				}
				else{
					a=(tbl[i-1].target-tbl[i-2].target)/(tbl[i-1].time-tbl[i-2].time);
					b=tbl[i-1].target-a*tbl[i-1].time;
					return a*sec+b;
				}
			}	
			i++;
		}
		else if(sec==tbl[i].time){
			return tbl[i].target;
		}
		else{
			if(i>0){
				a=(tbl[i].target-tbl[i-1].target)/(tbl[i].time-tbl[i-1].time);
				b=tbl[i].target-a*tbl[i].time;
				return a*sec+b;
			}
			else{
				a=tbl[i].target/tbl[i].time;
				b=0;
				return a*sec+b;
			}
		}
	}		
}

float TBL_linear(float sec, tbl_target *tbl)
{
	int i;
	float a,b;
	
	i=0;
	while(1){
		if(sec>tbl[i].time){
			if(0>tbl[i].time){
				if(i==0){
					return 0;
				}
				else if(i==1){
					a=tbl[i-1].target/tbl[i-1].time;
					b=0;
					return a*sec+b;
				}
				else{
					a=(tbl[i-1].target-tbl[i-2].target)/(tbl[i-1].time-tbl[i-2].time);
					b=tbl[i-1].target-a*tbl[i-1].time;
					return a*sec+b;
				}
			}	
			i++;
		}
		else if(sec==tbl[i].time){
			return tbl[i].target;
		}
		else{
			if(i>0){
				a=(tbl[i].target-tbl[i-1].target)/(tbl[i].time-tbl[i-1].time);
				b=tbl[i].target-a*tbl[i].time;
				return a*sec+b;
			}
			else{
				a=(tbl[i+1].target-tbl[i].target)/(tbl[i+1].time-tbl[i].time);
				b=tbl[i].target-a*tbl[i].time;
				return a*sec+b;
			}
		}
	}		
}

void main(void)
{
	init();
	log_send(0);
	
	c_start=1;
	
	while(1){
		log_send(1);
	}
}


#ifdef __cplusplus
void abort(void)
{

}
#endif