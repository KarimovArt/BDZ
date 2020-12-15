/*
 * подключения:
 *
 * ЦВХ1(in[4])-Ealarm; ЦВХ4-МТЗ
 */

/*
 СТАРЫЙ БЛОК:

используется SID (11 bit)
bit 6...0 -netID
bit 7 -almFlag
**********************************************************************************************************************
ИСХОДЯЩИЕ СООБЩЕНИЯ

в аварийном сообщении:
data[0] -состояния входов <4>Ealarm; <3>VOD3; <2>VOD2; <1>VOD1; <0>VOD0 (активный вход ==0)
data[1] -MSB время срабатывания МТЗ
data[2] -LSB время срабатывания МТЗ
data[3] -MSB время отпускания МТЗ
data[4] -LSB время срабатывания МТЗ
data[5] -MSB время срабатывания ключа
data[6] -LSB время срабатывания ключа
data[7] -==0

в сообщении о неисправности
data[0] -состояния входов(неисправности) <5>MTZ; <4>Ealarm; <3>VOD3; <2>VOD2; <1>VOD1; <0>VOD0 (неисправный вход ==1)

ответ на PING
data[0] -состояния входов(неисправности) <5>MTZ; <4>Ealarm; <3>VOD3; <2>VOD2; <1>VOD1; <0>VOD0 (неисправный вход ==1)

ответ на READ
data[0] -MSB время ожидания срабатывания МТЗ
data[1] -LSB время ожидания срабатывания МТЗ
data[2] -MSB время ожидания УРОВ
data[3] -LSB время ожидания EHJD
data[4] -<1>TEST_ON; <0>MTZ_ON
data[5] -==0xFF
data[6] -==0
data[7] -==0

**********************************************************************************************************************
ВХОДЯЩИЕ СООБЩЕНИЯ

data[0] -==0xFF
data[1] -==0xFF
data[2] -==0xFF
data[3] -==netID (==0 широковещательное)
data[4] -команда:_PING 0x01; TEST 0x02; READ 0x03; WRITE 0x04; RSTFLT 0x05

WRITE
data[0] -==0xFF
data[1] -==0xFF
data[2] -==0xFF
data[3] -==netID (или широковещательное)
data[4] -==0x04  (WRITE)
data[5] -==номер параметра: ADDR 0x00; время МТЗ 0x01; время УРОВ 0x02; Флаги МТЗ_ON TEST_ON 0x03
data[6] -==MSB параметра
data[7] -==LSB параметра

*/




#define WCAN
#define NETID (0xFFFFFF80 | eeprom_read_byte(&netID))
#define ALARM  0xFFFFFF7F


////// входящие команды для блоков //////
#define PING   0x01
#define TEST   0x02
#define READ   0x03
#define WRITE  0x04
#define RSTFLT 0x05

unsigned char inputFailMemory=0; //память неисправностей
EEMEM unsigned char MTZ_ON  = 1; // 1-МТЗ включен, 0-отключен
EEMEM unsigned char TEST_ON = 1; // 1-тест включен, 0-отключен


static void ackPING(void);
static void ackREAD(void);
static void ackWRITE(unsigned char nParam,unsigned int newVal);


void readMsg(void)
{
	signed char i=CANgetPosition();
	unsigned long int receivedID = ((unsigned long int)CANRXbuf[i].data[0]<<24) | ((unsigned long int)CANRXbuf[i].data[1]<<16) | ((unsigned int)CANRXbuf[i].data[2]<<8) | ((unsigned long int)CANRXbuf[i].data[3]);  //м

	if(i<0){ERROR=CAN;return;}
	if((receivedID != NETID) && (receivedID !=0xFFFFFF00) ) return;	//если сообщение не к нам или оно не широковещательное

	switch(CANRXbuf[i].data[4])//команда
	  {
	  case PING: ackPING();
	  break;

	  case READ: ackREAD();
	  break;

	  case WRITE:
		{
		ackWRITE(CANRXbuf[i].data[5],((int)CANRXbuf[i].data[6]<<8)|CANRXbuf[i].data[7] );//(номер изм.параметра,значение параметра)
		}
	  break;

	  case RSTFLT:
		{
		inputFailMemory=0;
		PORTA|=0xF0; //убрали все сбросы
	//    flag.test=1;
		}
	  break;

	  default: break;
	  }
}

static void ackPING(void)
{
	unsigned char dataToSend[8]={0,0,0,0,0,0,0,0};

	dataToSend[0]=inputFailMemory; //старое значение состояния входов 	| 0x01

	//через CAN
	#ifdef WCAN
	CAN_loadTXbuf(NETID, 8, dataToSend, CAN_TX_PRIORITY_3 & CAN_SID_FRAME);
	#endif
}

static void ackREAD(void)
{
unsigned char data[8]={0,0,0,0,0,0,0,0};

data[0]=MTZ_TIME >> 8;    //ст.байт
data[1]=MTZ_TIME & 0x00FF;//мл.байт
data[2]=UROV_TIME >> 8;    //ст.байт
data[3]=UROV_TIME & 0x00FF;//мл.байт
data[4]=eeprom_read_byte(&MTZ_ON) | (eeprom_read_byte(&TEST_ON)<<1); // bit1==TEST_ON bit0==MTZ_ON
data[5]=0xFF;//шоб не перепутать с сообщением неисправность

//через CAN
#ifdef WCAN
CAN_loadTXbuf(NETID, 8, data, CAN_TX_PRIORITY_3 & CAN_SID_FRAME);
#endif
}

static void ackWRITE(unsigned char nParam,unsigned int newVal)
{

//#define TIME 3000 //время,через кот.произойдет тестирование после изменения параметров

	switch(nParam)
	{
	case 0:eeprom_write_byte(&netID,newVal);
	break;

	case 1:eeprom_write_word(&time[0],newVal);
	break;

	case 2:eeprom_write_word(&time[1],newVal);
	break;

	case 3:
	{
		eeprom_write_byte(&MTZ_ON, newVal & 0x0001);
		eeprom_write_byte(&TEST_ON, (newVal & 0x0002)>>1);
	}
	break;

	}
	eeprom_update_byte(&dataCRC,CRC());	//обновляем контрольную сумму данных
	flag.test=1;								//после программирования тест
	ackREAD();
//return;
}


void sendAlarm(struct ARCHIEVE *arch)
{
	unsigned char dataToSend[8];
//char tmp[9];


	dataToSend[0]= ~arch->source | 0xE0;	//для БДЗ0 - вход сработал==0. отсылаем только ОВХ1...4 и ЦВХ1 (in[4])
	dataToSend[1]=arch->MTZon >> 8;		//время сраб.МТЗ
	dataToSend[2]=arch->MTZon & 0x00FF;
	dataToSend[3]=arch->MTZoff >> 8;		//время отпуск.МТЗ
	dataToSend[4]=arch->MTZoff & 0x00FF;

	//если сработка была без подтверждений время срабатывания ключа 0мс
	if((arch->MTZon ==0xFFFF) && (arch->MTZoff ==0xFFFF) && (arch->confirm ==0))
	{
		//uart_puts("no conf");
		dataToSend[5]=dataToSend[6]=0;
	}

	//если сработка была с подтверждением МТЗ Тключа==Тприхода_мтз
	if(arch->MTZon <0xFFFF)	//MTZon ==0 || ==1 означает шо в момент срабатывания МТЗ уже был
	{
		//uart_puts("mtz");
		dataToSend[5]=arch->MTZon >> 8;
		dataToSend[6]=arch->MTZon & 0x00FF;
	}

	//сработка была с подтверждением УРОВ Тключа==Тожидания_уров
	if((arch->MTZon ==0xFFFF) && (arch->MTZoff ==0xFFFF) && (arch->confirm !=0))
	{
		//uart_puts("urov");
		dataToSend[5]=UROV_TIME >> 8;		//время сраб.ключа (==время УРОВ)
		dataToSend[6]=UROV_TIME & 0x00FF;
	}

	dataToSend[7]=0;

//	uart_puts(ltoa(NETID,tmp,16));
//	uart_puts(ltoa(NETID & ALARM,tmp,16));

	#ifdef WCAN
	CAN_loadTXbuf(NETID & ALARM, 8, dataToSend, CAN_TX_PRIORITY_3 & CAN_SID_FRAME);
	#endif

	#ifdef RS
	for(i=0;i<8;i++) putchar(dataToSend[i]);
	putsf("_A");
	#endif
}
