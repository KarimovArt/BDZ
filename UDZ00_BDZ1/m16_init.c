

#include <MCP2515.h>
#ifdef DEBUG
#define UART_DEFAULT
#include <uart.h>
#endif
#define CAN_DEFAULT
#include <canspi.h>
#include "rtc.h"

extern void clearArch(void);

static inline void PORTS_init(void)
{
//DDR==0 вход; PORT==0 T; PORT==1 P;
//DDR==1 вЫход; PORT==0 низкий; PORT==1 высокий;

// Port A initialization
// PA7=In-P PA6=Out-0 PA5=Out-0 PA4=Out-0 PA3=Out-0 PA2=Out-0 PA1=Out-0 PA0=Out-0
PORTA=0x80;
DDRA=0x7F;

// Port B initialization
// PB7=Out-1 PB6=In-P PB5=Out-1 PB4=Out-1 PB3=Out-1 PB2=Out-1 PB1=Out-0 PB0=Out-1
PORTB=0b11110101;
DDRB =0b10111111;


// Port C initialization
// PC7=In-T PC6=In-T PC5=In-T PC4=In-T PC3=Out-1 PC2=Out-1 PC1=Out-1 PC0=Out-1
PORTC=0b00001111;
DDRC =0b00001111;

// Port D initialization
// PD7=In-T PD6=In-T PD5=In-T PD4=In-T PD3=Out-1 PD2=In-P PD1=Out-1 PD0=Out-1
PORTD=0b00001111;
DDRD =0b00001011;

/*
	// Timer/Counter 0 initialization
	// Clock source: System Clock
	// Clock value: Timer 0 Stopped
	// Mode: Normal top=FFh
	// OC0 output: Disconnected
	TCCR0=0x00;
	TCNT0=0x00;
	OCR0=0x00;

	// Timer/Counter 1 initialization
	// Clock source: System Clock
	// Clock value: 7,813 kHz
	// Mode: CTC top=OCR1A
	// OC1A output: Discon.
	// OC1B output: Discon.
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer 1 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: On
	// Compare B Match Interrupt: Off
	TCCR1A=0x00;
	TCCR1B=0x0D;
	TCNT1H=0x00;
	TCNT1L=0x00;
	ICR1H=0x00;
	ICR1L=0x00;
	OCR1AH=0x1E;
	OCR1AL=0x85;
	OCR1BH=0x00;
	OCR1BL=0x00;

	// Timer/Counter 2 initialization
	// Clock source: System Clock
	// Clock value: Timer 2 Stopped
	// Mode: Normal top=FFh
	// OC2 output: Disconnected
	ASSR=0x00;
	TCCR2=0x00;
	TCNT2=0x00;
	OCR2=0x00;

	// External Interrupt(s) initialization
	// INT0: Off
	// INT1: On
	// INT1 Mode: Falling Edge
	// INT2: Off
	GICR|=0x80;
	MCUCR=0x08;
	MCUCSR=0x00;
	GIFR=0x80;

	// Timer(s)/Counter(s) Interrupt(s) initialization
	TIMSK=0x10;

	// USART initialization
	// Communication Parameters: 7 Data, 1 Stop, Even Parity
	// USART Receiver: On
	// USART Transmitter: On
	// USART Mode: Asynchronous
	// USART Baud Rate: 9600
	UCSRA=0x00;
	UCSRB=0x98;
	UCSRC=0xA4;
	UBRRH=0x00;
	UBRRL=0x33;

	// Analog Comparator initialization
	// Analog Comparator: Off
	// Analog Comparator Input Capture by Timer/Counter 1: Off
	ACSR=0x80;
	SFIOR=0x00;

	// ADC initialization
	// ADC Clock frequency: 1000,000 kHz
	// ADC Voltage Reference: Int., cap. on AREF
	// ADC Auto Trigger Source: None
	//ADMUX=0xff;
	//ADCSRA=0x83;
*/

}

static inline void TIMERS_init(void)
{
// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value:  250000
// Mode: CTC
// OC0 output: Disconnected

TCCR0=0x0B;
TCNT0=0x00;
OCR0=25;	//прерывание 100мкс

// Timer/Counter 1 initialization
// Clock source: System Clock
// Mode: CTC top=OCR1A
// OC1A output: Discon.
// OC1B output: Discon.
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer 1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: On
// Compare B Match Interrupt: Off
TCCR1A=0x00;
TCCR1B=0x0B; // Clock value: 250 kHz
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=250;	//1ms
OCR1BH=0x00;
OCR1BL=0x00;

//TImer2
//TCCR2=0x05; // 125kHz

// Timer(s)/Counter(s) Interrupt(s) initialization
TIMSK=0x12;
}

/*

static inline void EXTINT_init(void)
{
	//int0
	DDRD &= ~(1<<DDD2);						//PD2 in
	PORTD |= 1<<DDD2;						//PD2 pullUp
	clrBit(MCUCR,ISC00);clrBit(MCUCR,ISC01);//The low level of INT0 generates an interrupt request.

//	//int1:
//	DDRD &= ~(1<<DDD3);						//PD3 in
//	PORTD |= 1<<DDD3;						//PD3 pullUp
//	clrBit(MCUCR,ISC10);setBit(MCUCR,ISC11);//The falling edge of INT1 generates an interrupt request.

	setBit(GICR,INT0);						//enable INT0
//	setBit(GICR,INT1);						//enable INT1

	//настройка прерывания
	//DDRB &= ~(1<<2);	//PORTB.2 вход
	//PORTB |= 1<<2;		//pull up
	//GICR=0x00;	//прерывание по низкому уровню
}
*/

signed char CAN_init(unsigned char mode)
{
	signed char result;

	SPI_init();


	//настройка прерывания
	//DDRB &= ~(1<<2);	//PORTB.2 вход
	//PORTB |= 1<<2;		//pull up
	//GICR=0x00;	//прерывание по низкому уровню

	CSPINDDR |= 1<<CSPIN; 	//пин CS выход
	CSPINPORT |= 1<<CSPIN;	// Hi-level

	CAN_reset();

	  //настройка скорости (125 kib/s)

	CAN_bitModify(CNF2,0x80,0x80); //разрешение свободного программирования PHSEG2 (BTLMODE==1)
	CAN_write(CNF1,0x05);// SJW==1Tq, BRP==6;
	CAN_write(CNF2,0x89);// BTLMODE==1,SAM==0,PRSEG2==2Tq,PHSEG1==2Tq ;
	CAN_write(CNF3,0x02);// PHSEG2==3Tq ;


	  //настройка скорости (10 kib/s)
	//  CAN_bitModify(CNF2,0x80,0x80);	//разрешение свободного программирования PHSEG2 (BTLMODE==1)
	//  CAN_write(CNF1,31);				// SJW==1Tq, BRP==31; 	=> Tq==8us.Для 10kib/s длина бита ~12Tq
	//  	  	  	  	  	  	  	  	  	//Typically, the sampling of the bit should take place at about 60-70% of the bit time
	//  	  	  	  	  	  	  	  	    // => 8Tq до выборки и 4Tq после
	//  	  	  	  	  	  	  	  	    //SyncSeg = 1Tq PropSeg = 4Tq PHSEG1=3Tq PHSEG2=4Tq
	//  CAN_write(CNF2,0b10010011);		// BTLMODE==1,SAM==0
	//  CAN_write(CNF3,0x03);				//


	#ifdef BDZ0
	//Фильтр и маска для буфера 0
	CAN_setMask(CAN_MASK_0,0x000007FF);  //фильтруем только SID
	CAN_setFilter(CAN_FILTER_0,0x000007FF,CAN_SID_FRAME);  // SID должен быть 7FF
	//Фильтры и маска для буфера 1
	CAN_setMask(CAN_MASK_1,0xFFFFFFFF);
	CAN_setFilter(CAN_FILTER_5,0x00000000,CAN_SID_FRAME);

	//включение фильтра 0 для SID
	CAN_bitModify(RXB0CTRL,0b01100111,0b00100000);
	//включение фильтра 5 для SID
	CAN_bitModify(RXB1CTRL,0b01100111,0b00100101);

	#else
	//Фильтр и маска для буфера 0	-пропускаем только сообщения к нам и широковещательные
	CAN_setMask(CAN_MASK_0,0x000000FF);  												//фильтруем по netID
	CAN_setFilter(CAN_FILTER_0,0x00000000,CAN_SID_FRAME); 								//фильтр для широковещ.сообщения (netID==00)
	CAN_setFilter(CAN_FILTER_1,0x00000000 | eeprom_read_byte(&netID),CAN_SID_FRAME); 	//фильтр для сообщения (netID==netID)
	CAN_bitModify(RXB0CTRL,0b01100000,0b00100000);										//Receive only valid messages with standart identifiers that meet filter criteria

	//Фильтры и маска для буфера 1	-пропускаем только SID==0x00000000
	CAN_setMask(CAN_MASK_1,0xFFFFFFFF);
	CAN_setFilter(CAN_FILTER_5,0x00000000,CAN_SID_FRAME);
	CAN_bitModify(RXB1CTRL,0b01100000,0b00100000);//включение фильтра для SID

	#endif

	//разрешаем прерывания по приему обоих буферов
	CAN_bitModify(CANINTE,0x03,0x03);


	//Установка режима работы
	result = CAN_setOpMode(mode);


	//  Additionally, the RXB0CTRL register can be configured
	//  such that, if RXB0 contains a valid message and
	//  another valid message is received, an overflow error
	//  will not occur and the new message will be moved into
	//  RXB1, regardless of the acceptance criteria of RXB1.


	//set rollover-mode
	CAN_bitModify(RXB0CTRL,0x04,0x04);

	//set one-shot-mode
	//CAN_bitModify(CANCTRL,0x08,0x08);

	return result;
}

int MCU_init(void)
{


	if(chkBit(MCUCSR,WDRF)){eeprom_update_byte(&error,WATCHDOG);}	//если перезагрузка была от собаки сохраняем ошибку
	MCUCSR=0;
	asm("cli");
	PORTS_init();
	TIMERS_init();

//	EXTINT_init();
	SPI_init();
#ifdef DEBUG
	if(UART_init(8,EVEN,1,9600)<0){ERROR=UART;return 0;};
#endif
	asm("sei");

	_delay_ms(5);	//время нарастания reset CAN
	if(CAN_init(CAN_MODE_NORMAL)<0) {ERROR=CAN;}
	//подсчет ЦРЦ данных,инициализация архива (если она еще не сохранена-первое включение)
	if(eeprom_read_byte(&dataCRC) ==0xFF)
	{
		eeprom_write_byte(&dataCRC,CRC());
		clearArch();
	}

	//тестирование при включении
	flag.test=1;

	//запуск WATCDOG
	WDTCR |= 1<<WDP0 | 1<<WDP1 | 1<<WDP2 ;	//время собаки ~2сек
	setBit(WDTCR,WDE);						//запускаем собаку

	return 1;
}
