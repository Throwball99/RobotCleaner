#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#define BIT_CHECK(a,b)  (!!((a) & (1<<(b))))

unsigned int timer_count; // 타이머 카운터 부분 1개의 오버플로우당 1ms
unsigned int timer_count_for_error = 0;

int i;
int j;
int data[5][8];
volatile unsigned char flag; // 스마트폰에서 atmega로 들어오는 변수값이 저장되는 곳
unsigned char humidifier_mode = 0; // 가습기 on/off 상태 표시 0일때는 off, 1일때는 on
volatile unsigned char humidity; // 습도를 표현하는 변수
volatile unsigned char robot_mode = 0; // 초기상태 0일때 정지, 1은 수동모드 이후 2~@까진 자동모드 
volatile unsigned char a;
volatile unsigned char error_check_start = 0;

int RH_integral;  // data[0]
int RH_decimal;   // data[1]
int T_integral;   // data[2]
int T_decimal;    // data[3]
int parity;       // data[4]

// dht_11 구동 부분---------------------------------------------------------------------------------------------------------		
void send_signal()
{
	DDRC = 0xff;
	PORTC = 0xff;
	PORTC = 0x01;  // low
	_delay_ms(20);  // wait 18ms
	PORTC = 0x80;  // high
	_delay_us(40);  // wait 20~40us
}

int response()
{
	DDRC = 0x01;
	if((PINC & 0x80) != 0)   // low가 아니면
	{
		return 1;
	}
	_delay_us(80);  // until 80us
	if((PINC & 0x80) == 0)  // high가 아니면
	{
		return 1;
	}
	_delay_us(80);  // until 80us
	return 0;
}

void send_data()
{
	if(response() == 0)
	{
		DDRC = 0x01;
		for(i = 0; i < 5; i++)   // RH, T, Parity bit
		{
			for(j=0; j < 8; j++)   // 8bit
			{
				while((PINC & 0x80) == 0)
				{
					;
				}
				_delay_us(30);
				if((PINC & 0x80) == 0)    // 26 ~ 28us
				{
					data[i][j] = 0;
				}
				else   //  70us
				{
					data[i][j] = 1;
					while((PINC & 0x80) == 128)  // while(PINC & (1<<7))
					{
						;
					}
				}
			}
		}
	}
	else
	{
		;
	}
}

void calculator_t_and_h(void)
{
	RH_integral = data[0][0] * 128 + data[0][1] * 64 + data[0][2] * 32 + data[0][3] * 16 + data[0][4] * 8 + data[0][5] * 4 + data[0][6] * 2 + data[0][7];
	RH_decimal = data[1][0] * 128 + data[1][1] * 64 + data[1][2] * 32 + data[1][3] * 16 + data[1][4] * 8 + data[1][5] * 4 + data[1][6] * 2 + data[1][7];
	T_integral = data[2][0] * 128 + data[2][1] * 64 + data[2][2] * 32 + data[2][3] * 16 + data[2][4] * 8 + data[2][5] * 4 + data[2][6] * 2 + data[2][7];
	T_decimal = data[3][0] * 128 + data[3][1] * 64 + data[3][2] * 32 + data[3][3] * 16 + data[3][4] * 8 + data[3][5] * 4 + data[3][6] * 2 + data[3][7];
	parity = data[4][0] * 128 + data[4][1] * 64 + data[4][2] * 32 + data[4][3] * 16 + data[4][4] * 8 + data[4][5] * 4 + data[4][6] * 2 + data[4][7];
}

// uart 통신 세팅 부분---------------------------------------------------------------------------------------------------------		
void uart1_tx(unsigned char number)
{
	while(!(UCSR1A & (1<<UDRE1)));
	UDR1 = number;
}

ISR(USART1_RX_vect)
{
	flag=UDR1;
}


// 타이머 세팅 부분---------------------------------------------------------------------------------------------------------	
void init_timer_0(void)   // 고속 PWM 모드, 64-분주비, 왼쪽 모터 A 초기상태 - 정지
{
	TCCR0 = 0x4c;  // foc0 = 0, wgm00 = 1, com01 = 0, com00 = 0, wgm01 = 1, cs02 = 1, cs01 = 0, cs00 = 0 , com 출력X(모터 회전 X)
	TCNT0 = 6;    //  4us x 250 = 1ms, 주기 1KHz
	OCR0 = 0;
	TIMSK |= (1<< TOIE0);
}

ISR(TIMER0_OVF_vect)
{
	TCNT0 = 6;
	timer_count++;
}

void init_timer_1(void)  //16비트 타이머,  8비트 고속 PWM 모드, 64-분주비   wgm13 = 0, wgm12 = 1, wgm11 = 0, wgm10 = 1
{
	TCCR1A = 0xa9;   // com1a1 = 1, com1a0 = 0, com1b1 = 1, com1b0 = 0, com1c1 = 1, com1c0 = 0, 1wgm11 = 0, wgm10 = 1
	TCCR1B = 0x0b;   // 0, 0, 0,  wgm13 = 0, wgm12 = 1, cs12 = 0, cs11 = 1, cs10 = 1
	TCCR1C = 0;
	TCNT1L = 6;       //  1ms, 주기 1KHz
	OCR1AL = 0;      // 오른쪽 모터 A 초기상태 - 정지
	OCR1BL = 0;      // 오른쪽 모터 B 초기상태 - 정지
	OCR1CL = 0;      // 왼쪽 모터 B 초기상태 - 정지
}

void init_timer_3(void)  //16비트 타이머, 고속 PWM모드, 8-분주비   wgm13 = 1, wgm12 = 1, wgm11 = 1, wgm10 = 0
{
	TCCR3A = 0x02;   // com3a1 = 1, com3a0 = 0, com3b1 = 1, com3b0 = 0, com3c1 = 1, com3c0 = 0, wgm31 = 1, wgm30 = 0
	TCCR3B = 0x1a;   // wgm33 = 1, wgm32 = 1, cs12 = 0, cs11 = 1, cs10 = 0
}




// 모터 세팅 부분---------------------------------------------------------------------------------------------------------	
void DC_1_forward(void)  // 왼쪽 모터 정방향 duty 50%
{
	TCCR0 = 0x6c;   // foc0 = 0, wgm00 = 1, com01 = 1, com00 = 0, wgm01 = 1, cs02 = 1, cs01 = 0, cs00 = 0 , com 자동출력 non-inverting mode
	OCR0 = 237;
	TCCR1A = 0xa9;   // com1a1 = 1, com1a0 = 0, com1b1 = 1, com1b0 = 0, com1c1 = 1, com1c0 = 0, 1wgm11 = 0, wgm10 = 1
	OCR1CL = 0;   // low
}

void DC_1_reverse(void)  // 왼쪽 모터 역방향 duty 50%
{
	TCCR0 = 0x7c;   // foc0 = 0, wgm00 = 1, com01 = 1, com00 = 1, wgm01 = 1, cs02 = 1, cs01 = 0, cs00 = 0   , com 출력X(모터 회전 X)
	OCR0 = 0;        // low
	TCCR1A = 0xa9;   // com1a1 = 1, com1a0 = 0, com1b1 = 1, com1b0 = 0, com1c1 = 1, com1c0 = 1, 1wgm11 = 0, wgm10 = 1
	OCR1CL = 237;
}

void DC_2_forward(void)  // 오른쪽 모터 정방향 duty 50%   --> 정방향
{
	TCCR3A = 0xaa;   // com3a1 = 1, com3a0 = 0, com3b1 = 0, com3b0 = 0, com3c1 = 0, com3c0 = 0, wgm31 = 1, wgm30 = 0
	OCR3A = 1650;
	OCR3B = 0;   // low
	ICR3 = 1999;
}

void DC_2_reverse(void) // 오른쪽 모터 역방향 
{
	TCCR3A = 0x22;   // com3a1 = 0, com3a0 = 0, com3b1 = 1, com3b0 = 0, com3c1 = 0, com3c0 = 0, wgm31 = 1, wgm30 = 0
	OCR3A = 0;   // low
	OCR3B = 1650;
	ICR3 = 1999;
}

void stop(void) // 양쪽 모터 정지 함수 바로 갔다 쓰면 됨.
{
	TCCR0 = 0x4c;   // foc0 = 0, wgm00 = 1, com01 = 0, com00 = 0, wgm01 = 1, cs02 = 1, cs01 = 0, cs00 = 0   , com 출력X(모터 회전 X)
	TCCR1A = 0x02;   // com1a1 = 1, com1a0 = 0, com1b1 = 1, com1b0 = 0, com1c1 = 1, com1c0 = 0, 1wgm11 = 0, wgm10 = 1
	TCCR3A = 0x02;   // com3a1 = 1, com3a0 = 0, com3b1 = 1, com3b0 = 0, com3c1 = 1, com3c0 = 0, wgm31 = 1, wgm30 = 0
	OCR0 = 0;
	OCR1CL = 0;
	ICR3 = 100;
}

void turn_left(void) //  왼쪽으로 회전하는 함수 바로 갔다 쓰면 됨.
{
	DC_1_reverse();
	DC_2_forward();
}

void turn_right(void) //  오른쪽으로 회전하는 함수 바로 갔다 쓰면 됨.
{
	DC_1_forward();
	DC_2_reverse();
}

void turn_straight(void) // 직진 함수 바로 갔다 쓰면 됨.
{
	DC_1_forward();
	DC_2_forward();
}

void back(void) // 역돌격 바로 갔다 쓰면 됨.
{
	DC_1_reverse();
	DC_2_reverse();
}


// 자동 청소 모드 부분---------------------------------------------------------------------------------------------------------	
void auto_mode_1(void) // [자동 청소 모드 1] 전방 적외선 센서에서 물체 감지시 오른쪽 적외선 감지or 미감지시 왼쪽으로 U턴, 왼쪽 적외선 감지시 오른쪽으로 U턴
                       // 오른쪽 왼쪽 동시에 감지 시, 제자리에서 360도 턴 위의 상황이 아니면 직진
{
		
	if((BIT_CHECK(PINF, 1) == 0) && (BIT_CHECK(PINF, 2) == 1) && (BIT_CHECK(PINF, 0) == 0))  // 가운데 감지, 왼쪽 미감지, 오른쪽 감지 했을 때
	{
		turn_left();
		_delay_ms(2000);
		turn_straight();
		_delay_ms(2000);
		turn_left();
		_delay_ms(2000);
	}
	if((BIT_CHECK(PINF, 1) == 0) && (BIT_CHECK(PINF, 2) == 1) && (BIT_CHECK(PINF, 0) == 1))  // 가운데 감지, 왼쪽 미감지, 오른쪽 미감지 했을 때
	{
		turn_left();
	    _delay_ms(2000);
	    turn_straight();
	    _delay_ms(2000);
		turn_left();
		_delay_ms(2000);
	}
	
	if((BIT_CHECK(PINF, 1) == 0) && (BIT_CHECK(PINF, 2) == 0) && (BIT_CHECK(PINF, 0) == 1))  // 가운데 감지, 왼쪽 감지, 오른쪽 미감지 했을 때
	{
		turn_right();
		_delay_ms(2000);
		turn_straight();
		_delay_ms(2000);
		turn_right();
		_delay_ms(2000);
	}
	if((BIT_CHECK(PINF, 1) == 0) && (BIT_CHECK(PINF, 2) == 0) && (BIT_CHECK(PINF, 0) == 0))  // 가운데 감지, 왼쪽 감지, 오른쪽 감지 했을 때
	{
		turn_left();
		_delay_ms(4000);
	}
	else
	{
		turn_straight();
	}
}

void auto_mode_2(void) // 자동 청소 모드2
{
	if(BIT_CHECK(PINF, 1) == 0)  // 가운데 감지
	{
		turn_left();
		turn_straight();
		_delay_ms(2000);
		while(BIT_CHECK(PINF,0) == 1)
		{
			DDRF = 0x01;
			_delay_ms(1000);
			turn_right();
			turn_straight();
			while(BIT_CHECK(PINF, 1) == 1)
			{
				turn_straight();
			}
			if(BIT_CHECK(PINF, 1) == 0)
			{
				turn_left();
				turn_straight();
				DDRF = 0x00;
				_delay_ms(2000);
			}
		}
		turn_left();
		turn_straight();
		
		while(BIT_CHECK(PINF, 1) == 1)
		{
			turn_straight();
		}
		
		if(BIT_CHECK(PINF, 1) == 0)   // U자 턴 한 후 가운데 감지되면
		{
			turn_right();
			turn_straight();
			_delay_ms(2000);
			while(BIT_CHECK(PINF,2) == 1)
			{
				DDRF = 0x04;
				_delay_ms(1000);
				turn_left();
				turn_straight();
				while(BIT_CHECK(PINF, 1) == 1)
				{
					turn_straight();
				}
				if(BIT_CHECK(PINF, 1) == 0)
				{
					turn_right();
					turn_straight();
					DDRF = 0x00;
					_delay_ms(2000);
				}
			}
			turn_right();
			turn_straight();
			
		}
	}
}



// 작동 후 초기 레지스터 세팅 부분------------------------------------------------------------------------------------------------
void set_reg1ster(void)
{
	//출력 관련 레지스터 설정 및 포트 입/출력 결정---------------------------------------------------------------------------------
	
	DDRA = 0xff;  // PORTA 모든 핀 출력 설정, led 존재했으나 지금은 없엠.
	PORTA =0x00;  
 	
	DDRB = 0xff; // 모터 드라이버 왼쪽, 오른쪽 존재, PB4~PB7
	PORTB = 0xff; 
	
	DDRC = 0xff; // DHT_11 PORTC 에 있음
	PORTC = 0x00;  // 초기 상태
	
	DDRE = 0xff;
	
	DDRF = 0x00; // 적외선 3개 있음, PF0 ~ PF2 입력 0일때 적외선 감지, 1일때 적외선 미감지
	PORTF = 0x00;
	
	DDRG = 0xff; // 가습기 PORTG 0번핀으로 옮김
	PORTG = 0x00; // 초기상테 가습기 on
	
	
   //통신 부분 레지스터----------------------------------------------------------------------------------------------------------
	UCSR1A=0x00;
	UCSR1B=0x98; // 0xD8
	UCSR1C=0x06;
	UBRR1H=0;
	UBRR1L=0x67; //  103
}



//기타 함수(새로 만드는 함수는 여기다가 만들고 나중에 옮기기 에러방지!!!!)-----------------------------------------------------------

void calculation_humidity(void) // 소수점 제외한 습도 
{
	humidity = RH_integral + (RH_decimal / 100);
}

/*void error_check(void)
{	
	if (error_check_code == 1) // error_check_code 초깃값을 내가 0으로 설정함.
	{
		continue;
	}
	if (BIT_CHECK(PINF, 1) == 0) // 전방 센서 감지
	{
		error_check_code = 1;
	}
}*/

void error_check_method(void)
{
	if (BIT_CHECK(PINF, 1) == 0)
	{
		error_check_start = 1;
	}
}

void error_check_start_gogo(void)
{
	if (error_check_start == 1)
	{
		if(timer_count == 1000)
		{
			timer_count_for_error++;
			timer_count = 0;
		}
	}
}

//------------------------------------------------------------------------------------------------------------------------------







// 메인문 ----------------------------------------------------------------------------------------------------------------------
int main(void)
{
	set_reg1ster();// 바로 위에 레지스터 설정 부분들
	timer_count = 0;
	
	
	init_timer_0();
	init_timer_1();
	init_timer_3();
	stop();// 로봇청소기 초기상태 / 정지상태 이후 버튼 조작에 따라서 상태 변화
	sei();

  // while 문------------------------------------------------------------------------------------------------------------------
	while (1)
	{
    // dht_11 구동 부분---------------------------------------------------------------------------------------------------------		
		send_signal();
		response();
		send_data();
		
	// 테스트 함수 넣기 --------------------------------------------------------------------------------------------------------
	    calculation_humidity(); // dht_11에서 받아온 값을 계산해주는 함수
		
    // 온습도 통신 부분----------------------------------------------------------------------------------------------------------	
		calculator_t_and_h(); // dht_11의 값들을 정리해주는 함수
		if((RH_integral + RH_decimal + T_integral + T_decimal) == parity)
		{
			uart1_tx((RH_integral / 10) + '0');
			uart1_tx((RH_integral % 10) + '0');
			uart1_tx('.');
			uart1_tx((RH_decimal / 10) + '0');
			uart1_tx((RH_decimal % 10) + '0');
			uart1_tx((T_integral / 10) + '0');
			uart1_tx((T_integral % 10) + '0');
			uart1_tx('.');
			uart1_tx((T_decimal / 10) + '0');
			uart1_tx((T_decimal % 10) + '0');
		}
		else
		{
			;
		}

    // 로봇청소기 구동 관련 부분--------------------------------------------------------------------------------------------------		
    //DC_1 = 우측모터 DC_2 = 좌측모터
    // reverse 모터 뒷쪽으로 회전 forward 모터 앞쪽으로 회전
    //turn straight = 양쪽 모터 전진 
    //turn left = 왼쪽으로 회전 turn right = 오른쪽으로 회전
	
	// flag값에 따른 robot 모드 설정----------------------------------------------------------------------------------------------	
	    if (flag == 'E') // E를 받았을때 로봇 정지 
	    {
		    robot_mode = 0; // robot mode 0일때 정지, 1일때 수동조작, 2일때 자동모드
	    }
		
		if (flag == 'H') // H를 받았을땐 스마트폰 앱에서 조이패드 나옴, 로봇은 수동조작 모드로 전환됨
		{
			robot_mode = 1;
		}
		
		 if (flag == 'F') // F를 받았을땐 자동 모드 1로 전환
		 {
			 robot_mode = 2;
		 }
		 
		 if (flag == 'G') // G를 받았을땐 자동 모드 2로 전환 
		 {
			 robot_mode = 3;
		 }
		
	// robot 모드에 따른 동작----------------------------------------------------------------------------------------------------
		if (robot_mode == 0) // robot mode 0일땐 로봇 정지
		{
			stop();
		}
		
	    if (robot_mode == 1) // robot mode 1일땐 로봇 수동 모드 
		{
			if (flag == 'A') // 수동 모드 (전진)
			{
				turn_straight();
			}
			
			if (flag == 'B') // 수동 모드 (후진)
			{
				back();
			}
			if (flag == 'C') // 수동 모드 (우측 회전)
			{
				turn_right();
			}
			if (flag == 'D') // 수동 모드 (좌측 회전)
			{
				turn_left();
			}
		}
		
		
	// 자동 모드----------------------------------------------------------------------------------------------------------------
	   
		if (robot_mode == 2) // 자동 모드 1
		{
			auto_mode_1();
		}
		
		
		if (robot_mode == 3) // 자동 모드 2
		{
			auto_mode_2();
		}
		
		
		
		
    // 가습기 부분---------------------------------------------------------------------------------------------------------------
	// Z라는 플래그를 스마트폰으로 받으면 humidifier_mode가 1로 바뀜, 이후 스마트폰에서 2자리의 원하는 습도의 자연수를 보내면 그 값이 
	// 현재 습도보다 낮으면 가습을 멈추고, 높으면 가습을 시작함.
	
	//flag 값에 따른 가습기 모드--------------------------------------------------------------------------------------------------
	
	// humidifier_mode = 0 => 가습기 off 상태
	// humidifier_mode = 1 => 자동 습도 조절 모드
	// humidifier_mode = 2 => 수동 습도 조절 모드
	// humidifier_mode = 3 => 기상청 api 활용 모드
	
		if (flag == 'Z')
		{
			humidifier_mode = 1; 
		}
		
		if (flag == 'Y')
		{
			humidifier_mode = 0;
		}
		
		if (flag == 'X')
		{
			humidifier_mode = 2;
		}
		
		if (flag == 'W')
		{
			humidifier_mode = 0;
		}
		
		if (flag == 'V')
		{
			humidifier_mode = 3;
		}
		
		
		//humidifier_mode에 따른 가습기 on/off [PORTG = 0x00 = 가습기 off] [PORTG = 0x01 = 가습기 on]------------------------------
		
		if (humidifier_mode == 1) // 이후 스마트폰에서 사용자가 원하는 습도값 전송받음
		{
			a = flag;
			if (a > humidity) // flag에 두자릿수 자연수의 습도값이 들어오고 그 값이 현재 습도보다 높으면 가습기 on
			{
				PORTG = 0x00; // 가습기 off
			}
			if (a <= humidity)// flag에 두자릿수 자연수의 습도값이 들어오고 그 값이 현재 습도보다 낮으면 가습기off 
			{
				PORTG = 0x01; // 가습기 on
			}
		}

        // 가습기 on/off 모드 ---------------------------------------------------------------------------------------------------
		if ( humidifier_mode == 0)
		{
			PORTG = 0x00; // 가습기 off
		}
		
		if (humidifier_mode == 2)
		{
			PORTG = 0x01;
		}
		
		// 기상청 api 활용 습도 조절 기능-----------------------------------------------------------------------------------------
		if (humidifier_mode == 3)
		{
			a = flag;
			if (a > humidity)
			{
				PORTG = 0x00;
			}
			if (a <= humidity)
			{
				PORTG = 0x01;
			}
		}
		
	}
}

