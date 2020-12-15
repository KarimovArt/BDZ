/*
 * Архив это:
 * -структура времени в десятичном виде
 * -входы-источники срабатывания (ARCHIEVE.source) 					<7>ЦВХ3; <6>ЦВХ2; <5>ЦВХ1; <4>ЦВХ0; <3>ОВХ3; <2>ОВХ2; <1>ОВХ1; <0>ОВХ0  (активный (сработавший) вход ==1)
 * -входы-источники подтверждения(только ЦВХ)(ARCHIEVE.confirm)                      					<3>ЦВХ3; <2>ЦВХ2; <1>ЦВХ1; <0>ЦВХ0  (активный (сработавший) вход ==1)
 * -сработавшие выходы (ARCHIEVE.outs)                        						   					<3>OUT3; <2>OUT2; <1>OUT1; <0>OUT0  (активный (сработавший) выход ==1)
 * -время прихода сигнала МТЗ (ARCHIEVE.MTZon)      	unsigned int, 0xFF-прихода не было
 * -время пропадания сигнала МТЗ (ARCHIEVE.MTZoff)  	unsigned int, 0xFF-пропадания не было
 *
 *

**********************************************************************************************************************
ИСХОДЯЩИЕ СООБЩЕНИЯ
	 используется SID (11 bit)
	bit 7...0 -netID
	bit 8 -fltFlag	флаги ==0 когда есть авария или неисправность
	bit 9 -almFlag
	bit 10 ==1


//ALARM: в аварийном  сообщении шлем только netID & almFlag, data==0

//FAULT: в сообщении о неисправности шлем только netID & fltFlag, data==0

ALARM: шлем netID & almFlag & fltFlag
	data[0]==ALARM ==0x00

ANSW_PING:  ответ на ping (он же синхронизация времени) шлем netID & almFlag & fltFlag
	data[0]==PING  (повторяется принятая команда)

ANSW_RESET: такой же как на пинг
	data[0]==RESET (повторяется принятая команда)

ANSW_FAULT:
	data[0]==RD_FAULT (повторяется принятая команда)
	data[1] -состояния входов	 <7>ЦВХ3; <6>ЦВХ2; <5>ЦВХ1; <4>ЦВХ0; <3>ОВХ3; <2>ОВХ2; <1>ОВХ1; <0>ОВХ0  (неисправный вход ==1)
	data[2] -код ошибки {TEST=1,DATACRC,WATCHDOG,HARDMEM,SOFTHAND,SOFTTIM,UART,RTC,CAN,MSG}

ANSW_ARCH: чтение архива (оно же просмотр аварии - архив[0])
	#1	-время/дата
	data[0]==RD_ARCH_1 (повторяется принятая команда)
	data[1] -DD
	data[2] -MM
	data[3] -YY
	data[4] -hh
	data[5] -mm
	data[6] -ss

	#2	-все остальное
	data[0]==RD_ARCH_2 (повторяется принятая команда)
	data[1] -состояния входов	 <7>ЦВХ3; <6>ЦВХ2; <5>ЦВХ1; <4>ЦВХ0; <3>ОВХ3; <2>ОВХ2; <1>ОВХ1; <0>ОВХ0  (активный вход ==1)
	data[2] -состояния выходов 								   	  	 <3>OUT3; <2>OUT2; <1>OUT1; <0>OUT0  (активный выход ==1)
	data[3] -MSB время срабатывания МТЗ
	data[4] -LSB время срабатывания МТЗ
	data[5] -MSB время отпускания МТЗ
	data[6] -LSB время отпускания МТЗ

ANSW_PROG: (ответ не выдается при широковещательном программировании)
	data[0]==PROG ==0x06
	data[1]==netID
	data[2]==MSB времени МТЗ
	data[3]==LSB времени МТЗ
	data[4]==MSB времени УРОВ
	data[5]==LSB времени УРОВ


**********************************************************************************************************************
ВХОДЯЩИЕ СООБЩЕНИЯ
	используется SID (11 bit)
	bit 7...0 -netID или широковещательное 0x00
	bit 8  ==1  -
	bit 9 ==1
	bit 10 ==1

0x01-PING:
	data[0]==PING ==0x01
	data[1] -DD
	data[2] -MM
	data[3] -YY
	data[4] -hh
	data[5] -mm
	data[6] -ss

0x02-RESET:
	data[0]==RESET ==0x02

0x03-RD_FAULT:
	data[0]==RD_FAULT ==0x03

0x04-RD_ARCH_1:
	data[0]==RD_ARCH_1 ==0x04
	data[1]==индекс архива

0x05-RD_ARCH_2:
	data[0]==RD_ARCH_2 ==0x05
	data[1]==индекс архива

0x06-PROG:
	data[0]==PROG ==0x06
	data[1]==новый netID
	data[2]==MSB времени МТЗ	- если значение отрицательное-время не изменяется
	data[3]==LSB времени МТЗ
	data[4]==MSB времени УРОВ
	data[5]==LSB времени УРОВ

	если dataLenght ==1 (только data[0]) -это чтение параметров

 */

#define ALARM     0x00
#define PING      0x01
#define RESET     0x02
#define RD_FAULT  0x03
#define RD_ARCH_1 0x04
#define RD_ARCH_2 0x05
#define PROG      0x06

#define FLT_FLAG 8
#define ALM_FLAG 9
#define CONF_FLAG 10

static inline unsigned char answPing(unsigned char index,unsigned char *data);
static inline unsigned char answReset(unsigned char *data);
static inline unsigned char answFault(unsigned char *data);
static inline unsigned char answArch(unsigned char part,unsigned char *data,unsigned char archindex);
static inline unsigned char answProg(signed char index,unsigned char *data,unsigned char *answFlag);
void sendAlarm(unsigned char typeofalarm);

void readMsg(void)
{
	signed char index=CANgetPosition();
	unsigned long int answID=(unsigned long int)eeprom_read_byte(&netID) | ((ERROR)?(0):(1)<<FLT_FLAG) | ((flag.alarm)?(0):(1)<<ALM_FLAG);
	(flag.confirmed_alarm)?(setBit(answID,CONF_FLAG)):(clrBit(answID, CONF_FLAG));
	unsigned char answData[8],answDataLenght=0,noAnswerFlag=0;

	if(index<0){ERROR=CAN;return;}

	switch(CANRXbuf[index].data[0])	//в дате[0] код команды
	{
	case PING: answDataLenght=answPing(index,answData);
	break;
	case RESET: answDataLenght=answReset(answData);
	break;
	case RD_FAULT: answDataLenght=answFault(answData);
	break;
	case RD_ARCH_1: answDataLenght=answArch(1,answData,CANRXbuf[index].data[1]);
	break;
	case RD_ARCH_2: answDataLenght=answArch(2,answData,CANRXbuf[index].data[1]);
	break;
	case PROG: answDataLenght=answProg(index,answData,&noAnswerFlag);
	break;
	default:{ERROR=MSG;return;}
	break;
	}

	if(noAnswerFlag ==0) CAN_loadTXbuf(answID, answDataLenght, answData, CAN_TX_PRIORITY_3 & CAN_SID_FRAME);
	if(CANRXbuf[index].data[0] ==RESET)asm("jmp 0");
}


static inline unsigned char answPing(unsigned char index,unsigned char *data)
{
	struct RTC tmp;
	//корректируем часы
	tmp.day=CANRXbuf[index].data[1];
	tmp.month=CANRXbuf[index].data[2];
	tmp.year=CANRXbuf[index].data[3];
	tmp.hours=CANRXbuf[index].data[4];
	tmp.minutes=CANRXbuf[index].data[5];
	tmp.seconds=CANRXbuf[index].data[6];

	rtc_set(&tmp);
	data[0]=PING;

	return 1;	//длина data
}

static inline unsigned char answReset(unsigned char *data)
{
	data[0]=RESET;
	return 1;	//длина data
}

static inline unsigned char answFault(unsigned char *data)
{
	unsigned char stateInputs=0;

	for(unsigned char i=0;i<NUMINP;i++) (in[i].FLT)?(setBit(stateInputs,i)):(clrBit(stateInputs,i));

	data[0]=RD_FAULT;	// (повторяется принятая команда)
	data[1]=stateInputs;// -состояния входов	 <7>ЦВХ3; <6>ЦВХ2; <5>ЦВХ1; <4>ЦВХ0; <3>ОВХ3; <2>ОВХ2; <1>ОВХ1; <0>ОВХ0  (неисправный вход ==1)
	data[2]=ERROR;		// -код ошибки {TEST=1,DATACRC,WATCHDOG,HARDMEM,SOFTHAND,SOFTTIM,UART,RTC,CAN,MSG}

	return 3;
}

static inline unsigned char answArch(unsigned char part,unsigned char *data,unsigned char archindex)
{
	struct ARCHIEVE tmp;


	data[0]=(part==1)?(RD_ARCH_1):(RD_ARCH_2);

//uart_putchar(archindex);
	readArch(archindex,&tmp);
	switch(part)
	{
	case 1:
	{
		data[1]=tmp.rtc.day;		//-DD
		data[2]=tmp.rtc.month; 		//-MM
		data[3]=tmp.rtc.year; 		//-YY
		data[4]=tmp.rtc.hours;		//-hh
		data[5]=tmp.rtc.minutes;	//-mm
		data[6]=tmp.rtc.seconds; 	//-ss
	}
	break;
	case 2:
	{
		data[1]=tmp.source | tmp.confirm;	// -состояния входов	 <7>ЦВХ3; <6>ЦВХ2; <5>ЦВХ1; <4>ЦВХ0; <3>ОВХ3; <2>ОВХ2; <1>ОВХ1; <0>ОВХ0
		data[2]=tmp.outs;					// -состояния выходов 	 <3>OUT3; <2>OUT2; <1>OUT1; <0>OUT0  (активный выход ==1)
		data[3]=tmp.MTZon >> 8; 			//-MSB время срабатывания МТЗ
		data[4]=tmp.MTZon & 0x00FF; 		//-LSB время срабатывания МТЗ
		data[5]=tmp.MTZoff >> 8;			//-MSB время отпускания МТЗ
		data[6]=tmp.MTZoff & 0x00FF;		//-LSB время отпускания МТЗ
	}
	break;
	}
	return 7;
}

static inline unsigned char answProg(signed char index,unsigned char *data,unsigned char *answFlag)
{
	asm("cli");
	if((CANRXbuf[index].ID & (unsigned long int)0xFF) ==0) *answFlag=1;	//если запрос был широковещательным отвечать не надо

	data[0]=PROG;
	if(CANRXbuf[index].dataLength > 1)	//если получен запрос на изменение параметра
	{
		signed int param=(signed int)CANRXbuf[index].data[2]<<8 | CANRXbuf[index].data[3];	//новое время МТЗ
		if(param>=0)eeprom_update_word(&time[0],param);										//если оно положительное -записываем новое значение

		param=(signed int)CANRXbuf[index].data[4]<<8 | CANRXbuf[index].data[5];				//новое время УРОВ
		if(param>=0)eeprom_update_word(&time[1],param);

		if(*answFlag ==0) eeprom_update_byte(&netID,CANRXbuf[index].data[1]);							//если сообщение было не широковещательным сохраняем новый адрес
		eeprom_update_byte(&dataCRC,CRC());																//обновляем контрольную сумму данных
		if(CAN_init(CAN_MODE_NORMAL)<1){ERROR=CAN;}	//		asm("jmp 0");	//фильтры CAN настроены на 00 или netID.если не перезапустить прибор,то при замене адреса не обновятся фильтры.итог-потеря связи

	}
	data[1]=eeprom_read_byte(&netID);
	data[2]=MTZ_TIME >> 8;
	data[3]=MTZ_TIME & 0x00FF;
	data[4]=UROV_TIME >> 8;
	data[5]=UROV_TIME & 0x00FF;
	asm("sei");

	return 6;
}



void sendAlarm(unsigned char typeofalarm)
{
	unsigned long int answID=(unsigned long int)eeprom_read_byte(&netID);
	unsigned char data=ALARM;

	(ERROR)?(clrBit(answID,FLT_FLAG)):(setBit(answID,FLT_FLAG));
	clrBit(answID,ALM_FLAG);
	if (typeofalarm)
	{
		setBit(answID,CONF_FLAG);
		flag.confirmed_alarm=1;

	}
	else clrBit(answID,CONF_FLAG);

	//(i)?(setBit(answID,CONF_FLAG)):(clrBit(answID,CONF_FLAG));
	flag.alarm=1;

	CAN_loadTXbuf(answID, 1, &data, CAN_TX_PRIORITY_3 & CAN_SID_FRAME);
}

