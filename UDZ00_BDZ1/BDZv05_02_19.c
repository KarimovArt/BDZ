#include "avr/io.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "avr/interrupt.h"
#include "util/delay.h"
#include <avr/eeprom.h>
#include <util/crc16.h>

#include "BDZ.h"


//#define DEBUG			//режим отладки
//#define BDZ_OLD		//режим совместимости с предыдущей версией БДЗ0
#define MAX_BDZ_ID 99	//максимальный ID


#define MTZ_TIME eeprom_read_word(&time[0])	//время мтз
#define UROV_TIME eeprom_read_word(&time[1])//время уров
#define TEST_TIME eeprom_read_word(&time[2])//интервал теста,ms
#define CMDLINELENGHT 15					//максимальная длина командной строки
#define READOPTINPUT (PIND>>4)				//оптические входы LSB
#define READDIGINPUT (PINC & 0xF0)			//цифровые входы MSB
#define OUT_ON 	out[0].ON || out[1].ON || out[2].ON || out[3].ON						//факт аварии
#define WAIT_MTZ_OFF 5000					//ожидание отпускания МТЗ


/*************************************ПРОГРАММИРУЕМЫЕ ПАРАМЕТРЫ*************************************/

//сетевой адрес
EEMEM unsigned char netID=13;
//времена МТЗ, УРОВ ,тест
EEMEM unsigned int time[]={200,200,10000};
//выходы(мл.тетрада) подключенные ко входам
EEMEM unsigned char o[NUMINP]=   {0b00000000,0b00000000,0b00000000,0b00000011,0b00000000,0b00000000,0b00000000,0b00000000};	//входы [о0,о1,о2,о3,ц0,ц1,ц2,ц3]
//входы подтверждения.Только ЦВХ(младшая тетрада)
EEMEM unsigned char cin[NUMINP]= {0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000};
//режимы подтверждения (для ЦВХ)(0-mtz 1-urov)
EEMEM unsigned char cmod[NUMINP]={0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000};
//задержки включения для выходных реле
EEMEM unsigned int don[NUMOUT]={0,0,0,0};	// (вых0,вых1,вых2,вых3)
//выключения
EEMEM unsigned int doff[NUMOUT]={400,400,400,400};
//контрольная сумма данных
EEMEM unsigned char dataCRC=0xFF;

/***************************************************************************************************/



struct INPUTS in[NUMINP]=
{	//флаги:ACT,FLT,ALM,MOD;адреса (в ЕЕПРОМе) входов и типов подтверждений,адрес выходов,счетчики обработчиков
	{0,0,0,0,&cin[0],&cmod[0],&o[0],0},
	{0,0,0,0,&cin[1],&cmod[1],&o[1],0},
	{0,0,0,0,&cin[2],&cmod[2],&o[2],0},
	{0,0,0,0,&cin[3],&cmod[3],&o[3],0},
	{0,0,0,0,&cin[4],&cmod[4],&o[4],0},
	{0,0,0,0,&cin[5],&cmod[5],&o[5],0},
	{0,0,0,0,&cin[6],&cmod[6],&o[6],0},
	{0,0,0,0,&cin[7],&cmod[7],&o[7],0}
};

struct OUTPUTS out[NUMOUT]=
{	//флаг срабатывания,флаг отпускания,задержка вкл. и выкл(адреса ЕЕПРОМ),порт и пин к кот.подключено реле
		{0,0,&don[0],&doff[0],&PORTC,0},
		{0,0,&don[1],&doff[1],&PORTC,1},
		{0,0,&don[2],&doff[2],&PORTC,2},
		{0,0,&don[3],&doff[3],&PORTC,3}


//	{0,0,&don[0],&doff[0],&PORTC,2},
//	{0,0,&don[1],&doff[1],&PORTC,3},
//	{0,0,&don[2],&doff[2],&PORTD,1},
//	{0,0,&don[3],&doff[3],&PORTB,2}
};

//массив указателей на обработчики
struct HANDLER *handler[NUMHAND];
//массив указателей на таймеры
struct TIMER *timer[NUMTIM];
//системные часы
volatile unsigned int sysClk;
//ожидание отпускания МТЗ
volatile unsigned int waitMTZ_off;
//№ входа для контроля отпускания мтз
unsigned char inpWaitMTZ_off;
//коды ошибок
volatile enum{TEST=1,RTC,CAN,MSG,DATACRC,WATCHDOG,HARDMEM,SOFTHAND,SOFTTIM,UART}ERROR;
//состояния PORTA для индикации
unsigned char port0,port1;
//память последней ошибки
EEMEM unsigned char error=0;
//режимы индикации
enum {SB,LB,ON};
//создает таймер включения/выключения для выхода. Получает №выхода и режим работы(ON/OFF)
static void createTimer(unsigned char Nin,unsigned char Nout,unsigned char mode);
//создает обработчик входа. Получает №входа, №входа для подтверждения, режим подтверждения(МТЗ/УРОВ)
static void createHandler(unsigned char Nin, unsigned char NconfirmInput,unsigned char mode);
//обработчик таймера
static void proceedTimers(void);
//обработчик обработчика :)
static void proceedHandlers(void);
//ситывает состояния входов,выходов,аварии,неисправности (для последующей индикации)
static inline void makeLed(unsigned char mode);
//индикация со считыванием состояния цифровых входов.возвращает состояние кнопки
static unsigned char indicate(void);
//создает требуемый обработчик
static void makeHandler(unsigned char i);
//тестирование всех входов
static void test(void);
//проверка контрольной суммы данных
unsigned char CRC (void);

#include "m16_init.c"
#include "archieve.c"
#ifdef DEBUG
#include "prog.c"
#endif

struct ARCHIEVE tmpArchieve;

#ifdef BDZ_OLD
#include "bdz_old.c"
#else
#include "bdz_new.c"
#endif


int main(void)
{
	MCU_init();
	//инициализация временного архива (во времена врем.архива пишем FF)
	tmpArchieve.MTZon=tmpArchieve.MTZoff=0xFFFF;

	#ifdef DEBUG
	uart_puts("INIT");
	#endif
_delay_ms(50);

	while(1)
	{
	#ifdef DEBUG
		char buf[CMDLINELENGHT];
		if(rx_counter){uart_gets(buf,CMDLINELENGHT-1);uart_puts(prog(buf));}//программирование
		if(UART_ERR)ERROR=UART;
	#endif

		//сохранение памяти ошибки
		if(ERROR)
		{
			asm("cli");
			eeprom_update_byte(&error,ERROR);
			asm("sei");
		}


		//если нажата кнопка (индикация со сканированием ЦВХ и кнопки)
		if(indicate() ==0)
		{
			asm("wdr");
			setBit(PORTB,0);clrBit(PORTB,1);PORTA=0;
			//com1==0 com0==1
			asm("cli");
			for(unsigned char i=0;i<4;i++)forceBit(PORTA,(4-i),chkBit(eeprom_read_byte(&error),i));	//индикация последней ошибки
			asm("sei");
			_delay_ms(500);
			asm("jmp 0");																			//если нажата кнопка перезапускаем программу
		}

		//если сработал любой вх
		if(flag.trigger)
		{
			for(unsigned char i=0; i<NUMINP; i++) if(in[i].ACT) makeHandler(i);	//обрабатываем  входы:если вход активен создаем обработчик
			flag.trigger=0;		//запущена обработка всех входов
		}

		//если нет обработчиков и таймеров
		if(flag.noTimers && flag.noHandlers)
		{
			flag.noEmpty=0;												//даем разрешение на запись во временный архив
			if(flag.saveArch && (waitMTZ_off==0))saveArch(&tmpArchieve);//если было событие и время ожидания отпускания МТЗ истекло-запись архива в ЕЕПРОМ
			if(CANRXcounter)readMsg();									//что то пришло-читаем
		#ifdef BDZ_OLD
			if(flag.test && eeprom_read_byte(&TEST_ON)) test();	//настало время теста и он разрешен
		#else
			if(flag.test) test();								//настало время теста
		#endif
		}
	}//while(1)
	return 1;
}

static void createHandler(unsigned char Nin, unsigned char NconfirmInput,unsigned char mode)
{//uart_putchar('h');

asm("cli");
	unsigned char i;

	struct HANDLER *tmp=malloc(sizeof (struct HANDLER));//выделяем память под обработчик
	if(tmp==NULL) {ERROR=HARDMEM;return;}				//нет места в памяти

	for(i=0;i<NUMHAND;i++) if(handler[i] ==NULL) break;	//если указатель на обработчик ==0 то он свободен
	if(i==NUMHAND) {ERROR=SOFTHAND;return;}				//свободных обработчиков нет

	flag.noHandlers=0;		//так надо
	//если временный архив пуст(запись еще не ведется)-обнуляем локальное время,читаем глобальное время,начинаем запись.
	if(flag.noEmpty==0)
	{
		sysClk=0;
		rtc_get(&tmpArchieve.rtc);
		flag.noEmpty=1;
	}

//	if(Nin < NUMOPTINP)setBit(DDRD,(Nin+4));	//блокируем ОПТИЧЕСКИЙ вход  (делаем выход с "0".если этого не сделать не пройдут остальные прерывания)
	handler[i]=tmp;								//указатель на созданный обработчик.Используется для освобождения памяти по окончании обработчика
	handler[i]->Nin = Nin;						//№ обрабатываемого входа
	handler[i]->NconfInput = NconfirmInput;		//№ входа для подтверждения
	handler[i]->mode = mode;					//режим подтверждения (МТЗ,УРОВ,БЕЗ_ПОДТВ)
	if(mode == NOCONF)handler[i]->time =0;		//если режим БЕЗ_ПОДТВ время ==0
	else handler[i]->time = (mode == UROV)?(UROV_TIME):(MTZ_TIME);
	in[handler[i]->Nin].handlers++;				//увеличиваем значение кол-ва обработчиков на данном входе
asm("sei");
}

static void createTimer(unsigned char Nin,unsigned char Nout,unsigned char mode)
{//uart_putchar('T');uart_putchar('0'+Nout);uart_puts("");
	unsigned char i;

	struct TIMER *tmp=malloc(sizeof (struct TIMER));
	if(tmp==NULL) {ERROR=HARDMEM;return;}				//нет места в памяти
	for(i=0;i<NUMTIM;i++) if(timer[i] ==NULL) break;	//если указатель на таймер ==0 то он свободен
	if(i==NUMTIM) {ERROR=SOFTTIM;return;}				//таймеры закончились

	flag.noTimers=0;		//так надо
	timer[i]=tmp;			//нашелся свободный указатель.Этот указатель нужно будет попользовать для free
	timer[i]->Nin=Nin;
	timer[i]->Nout=Nout;	//дальше все инициализируем
	timer[i]->mode=mode;
	asm("cli");
	timer[i]->time = (mode == T_ON)?(eeprom_read_word(out[Nout].delay_on)):(eeprom_read_word(out[Nout].delay_off));
	asm("sei");
}

static void proceedHandlers(void)
{
	unsigned char i=0,j=0;

	flag.noHandlers=1;									//этот флаг обнулится если найдется обработчик
	for(i=0;i<NUMHAND;i++)
	{
		if(handler[i] ==NULL)continue;					//если указатель на обработчик ==0 то обработчика нет.проверяем следующий
		flag.noHandlers=0;
		if(handler[i]->time) {handler[i]->time--;}		//обработчик нашелся и его время не истекло

		unsigned char outs=eeprom_read_byte(in[handler[i]->Nin].outputs);	//считали подключенные ко входу выходы
		switch(handler[i]->mode)
		{
		case NOCONF:
		{
			for(j=0;j<NUMOUT;j++) if(chkBit(outs,j)) createTimer(handler[i]->Nin,j,T_ON);	//создаем таймеры ON для всех подключенных выходов
			setBit(tmpArchieve.source,handler[i]->Nin);										//запоминаем № аварийного входа во временном архиве
			in[handler[i]->Nin].ALM=1;														//ставим флаг,что вход,вызвавший обработчик был источником аварии
			--in[handler[i]->Nin].handlers; //убавили счетчик обработчиков
			free(handler[i]); handler[i]=NULL;												//Убийство обработчика:освобождаем выделенную под него память,указатель на него обнуляем
//			flag.trigger=1;
		}
		break;
		case MTZ:
		{
			if(handler[i]->time)	//время не истекло...
			{
				if(in[handler[i]->NconfInput].ACT)	//...и вход подтверждения сработал (бит"АКТИВЕН" статуса входа ==1)
				{
					//создаем таймеры ON для всех подключенных выходов
					for(char j=0;j<NUMOUT;j++) if(chkBit(outs,j)) createTimer(handler[i]->Nin,j,T_ON);
					if(tmpArchieve.MTZon==0xFFFF)tmpArchieve.MTZon=sysClk;	//фиксим локальное время сработки МТЗ(если оно еще не зафиксино)
					setBit(tmpArchieve.source,handler[i]->Nin);				//запоминаем № аварийного входа
					setBit(tmpArchieve.confirm,handler[i]->NconfInput);		//запоминаем № входа подтверждения
				inpWaitMTZ_off=handler[i]->NconfInput;	//запоминаем № входа МТЗ
				waitMTZ_off=WAIT_MTZ_OFF;				//взводим таймер ожидания отпускания МТЗ
					in[handler[i]->Nin].ALM=1;								//вход,вызвавший обработчик был источником аварии
					in[handler[i]->NconfInput].CONF=1;						//вход был источником подтверждения
					--in[handler[i]->Nin].handlers;							//убавили счетчик обработчиков
					free(handler[i]);handler[i]=NULL;						//Убийство обработчика:освобождаем выделенную под него память,указатель на него обнуляем
					break;
				}
			}
			else	//время истекло
			{
				//убавили счетчик обработчиков и если он ==0 и обрабатывается оптический вход -убрали флаг "в обработке"
				--in[handler[i]->Nin].handlers;							//убавили счетчик обработчиков
				free(handler[i]);handler[i]=NULL;		//Убийство обработчика:освобождаем выделенную под него память,указатель на него обнуляем
			}
		}
		break;

		case UROV:
		{
			//если вход подтверждения отпустило (бит"АКТИВЕН" статуса входа  ==0),заканчиваем:
			if(in[handler[i]->NconfInput].ACT ==0)
			{
//				if(tmpArchieve.MTZoff==0xFFFF) tmpArchieve.MTZoff=sysClk;		//фиксим локальное время отпускания МТЗ
//				flag.saveArch=1;
				--in[handler[i]->Nin].handlers;							//убавили счетчик обработчиков
				free(handler[i]);handler[i]=NULL;		//Убийство обработчика:освобождаем выделенную под него память,указатель на него обнуляем
				break;
			}
			if(handler[i]->time)break;	//если время не истекло выходим из этого case (здесь вход еще не отпустило)
			else 						//а вот здесь вход еще не отпустило и время истекло.Надо сработать
			{
				//создаем таймеры ON для всех подключенных выходов
				for(char j=0;j<NUMOUT;j++) if(chkBit(outs,j)) createTimer(handler[i]->Nin,j,T_ON);
				setBit(tmpArchieve.source,handler[i]->Nin);			//запоминаем №аварийного входа
				setBit(tmpArchieve.confirm,handler[i]->NconfInput);	//запоминаем № входа подтверждения
				in[handler[i]->Nin].ALM=1;							//вход,вызвавший обработчик был источником аварии
				in[handler[i]->NconfInput].CONF=1;					//вход был источником подтверждения
				--in[handler[i]->Nin].handlers;						//убавили счетчик обработчиков
				free(handler[i]);handler[i]=NULL;					//Убийство обработчика:освобождаем выделенную под него память,указатель на него обнуляем
			}
		}
		break;
		}//switch
	}//for
}

static void proceedTimers(void)
{
flag.noTimers=1; //если найдется таймер этот флаг сбросится
//Убийство таймера:освобождаем выделенную под него память,указатель на него обнуляем
for(unsigned char i=0;i<NUMTIM;i++)
{
	if(timer[i] ==NULL){continue;}					//если указатель на таймер ==0 то таймера нет.вываливаемся
	flag.noTimers=0;
	if(timer[i]->time){timer[i]->time--;continue;}	//если оказались здесь нашелся живой таймер.время не ==0.Убавляем и вываливаемся
	switch(timer[i]->mode)							//время истекло.выясняем режим таймера
	{
	case T_ON:
	{
		clrBit(*out[timer[i]->Nout].port,out[timer[i]->Nout].pin);									//включаем реле
		setBit(tmpArchieve.outs,timer[i]->Nout);												 	//сохранили сработавшие выходы
		out[timer[i]->Nout].ON=1;																	//ставим ФЛАГ "СРАБОТАЛ"
		if(eeprom_read_word(out[timer[i]->Nout].delay_off) >0)	createTimer(timer[i]->Nin,timer[i]->Nout,T_OFF);	//если есть задержка отключения создаем таймер выключения для реле
		//else if(in[timer[i]->Nin].handlers==0)clrBit(DDRD,(timer[i]->Nin+4));						//если обработка входа закончена-разблокируем вход
		free(timer[i]);timer[i]=NULL;																//убиваем таймер
		flag.saveArch=1;																			//включение произошло.надо будет сохранить архив
		sendAlarm();
	}
	break;

	case T_OFF:
	{
		setBit(*out[timer[i]->Nout].port,out[timer[i]->Nout].pin);		//выключаем реле
		out[timer[i]->Nout].OFF=1;										//ставим ФЛАГ "ОТПУСТИЛО"
		//if(in[timer[i]->Nin].handlers==0)clrBit(DDRD,(timer[i]->Nin+4));//если обработка входа закончена-разблокируем вход
		free(timer[i]);timer[i]=NULL;									//убиваем таймер
	}
	break;
	}//switch

}//for

return;
}

static inline void makeLed(unsigned char mode)
{
#define I_ON(chan) in[chan].ACT															//условие постоянного горения
#define I_LB(chan) (in[chan].CONF || in[chan].ALM) && !in[chan].ACT						//условие длинного блинка
#define I_SB(chan) !in[chan].ACT && !in[chan].CONF && !in[chan].ALM && in[chan].FLT		//условие короткого блинка


	unsigned char i;
	static unsigned char sb=0,lb=0;

	switch (mode)
	{
	case ON:	//
	{//uart_putchar('-');
		//входы
		for(i=0;i<7;i++) (I_ON(i))?(setBit(port0,i)):(clrBit(port0,i));
		(I_ON(7))?(setBit(port1,0)):(clrBit(port1,0));

		//питание
		if(!ERROR)
			{
			setBit(port1,6);
			clrBit(*out[3].port,out[3].pin);
			out[3].ON=1;
			}
		else
			{
			setBit(*out[3].port,out[3].pin);
			out[3].OFF=1;
			}
		//выходы
		for(i=0;i<4;i++) (out[i].ON && !out[i].OFF)?(setBit(port1,(i+1))):(clrBit(port1,(i+1)));
	}
	break;
	case LB:
	{//uart_putchar('L');
		//входы
		for(i=0;i<7;i++) if(I_LB(i)) {(lb==0)?(setBit(port0,i)):(clrBit(port0,i));}//uart_putchar('n');
		if(I_LB(7)) (lb==0)?(setBit(port1,0)):(clrBit(port1,0));
		//выход
		for(unsigned char i=0;i<4;i++)									//выходы
		{
			if(out[i].ON && out[i].OFF) (lb==0)?(setBit(port1,(i+1))):(clrBit(port1,(i+1)));
		}
		if(flag.alarm)	(lb==0)?(setBit(port1,5)):(clrBit(port1,5));										//ALARM
		lb=(lb==0)?(1):(0);
	}
	break;
	case SB:
	{_delay_ms(1);
		//входы
		for(i=0;i<7;i++) if(I_SB(i)) (sb==0)?(setBit(port0,i)):(clrBit(port0,i));
		if(I_SB(7)) (sb==0)?(setBit(port1,0)):(clrBit(port1,0));
		//питание
		if(ERROR)(sb==0)?(setBit(port1,6)):(clrBit(port1,6));
		sb=(sb==0)?(1):(0);
	}

	break;


	}
}

static unsigned char indicate(void)
{
	makeLed(ON);						//постоянно включенные
	for(unsigned char j=0;j<0xFF;j++)
	{
		if(j%3 ==0)makeLed(SB);		//короткий блинк
		if(j%20 ==0)makeLed(LB);	//длинный блинк
		for(unsigned char i=0xFF;i>0;i--)
		{
			asm("wdr");
			#ifdef DEBUG
			if(rx_counter) goto EXIT;
			#endif
			if( CANRXcounter || !chkBit(PINA,KEYPIN) ) goto EXIT;	//если чо нить произошло (CAN или кнопка) надо вывалиться
			if(!chkBit(PIND,CANPIN)){CAN_readMessage();/*fakeCAN_readMessage();*/}			//если пришло CAN читаем его в буфер

			PORTA=0x80;
			PORTB ^=0x03;										//переключаем com-ы
			PORTA=(chkBit(PORTB,COM0))?(port1):(port0) | 0x80;	//0x80 необходимо для сохр. притяжки PA7 (кнопка)

			while(flag.update==0)if(flag.trigger) goto EXIT;	//100мкС пауза (время опроса входов в прерывании)
			flag.update=0;
		}
	}
EXIT:	return (PINA & (1<<DDA7));
}

static void makeHandler(unsigned char i)
{
	asm("cli");
	unsigned char confirmInp= eeprom_read_byte(in[i].conf_in);	//считали входы подтверждения
	unsigned char confirmTyp= eeprom_read_byte(in[i].conf_mod);	//считали тип подтверждения
	unsigned char outs=eeprom_read_byte(in[i].outputs);			//считали подключенные ко входу выходы
	asm("sei");

	if(in[i].handlers !=0)return;	//вход уже обрабатывается
//uart_putchar(i);
	//срабатывания могут быть ТОЛЬКО если есть подключенные выходы
	if(outs !=0)
	{
		if(confirmInp==0)  {createHandler(i,0,NOCONF);}	//срабатывания без подтверждений
		else		//срабатывания с подтверждениями (cоздаем хэндлеры для каждого входа подтверждения(только цифровые,т.е.начинаяс 4го))
		{
		for(unsigned char j=0;j<NUMDIGINP;j++)
			if(chkBit(confirmInp,j))
			{
				if(!chkBit(confirmTyp,j)) {createHandler(i,j+NUMOPTINP,MTZ);}		//если тип подтверждения не УРОВ
				else if(in[j+NUMOPTINP].ACT) {createHandler(i,j+NUMOPTINP,UROV);}		//если тип подтвержд.УРОВ и вход подтв. активен
			}
		}
	}
//	return result;
}

static void test(void)
{//uart_putchar('t');
	unsigned char i;
	static unsigned char flt[NUMOPTINP];

	//проверка на активные входы
	for(i=0;i<NUMOPTINP;i++)
	{
		if(in[i].ACT)//есть сработавший вход-надо отложить тест
		{
			if(++flt[i] >= 3)	//три ретеста (три цикла индикации)
			{
				ERROR=in[i].FLT = 1;							//ставим флаг неисправности и ошибку тестирования
				(i<4)?(setBit(DDRD,(i+4))):(setBit(DDRC,i));	//блокируем вход (делаем выход с "0")
				flt[i]=0;
			}
			else {/*countdown=255;*/return;}	//откладываем тест (повторно он запустится после цикла индикации)
		}
		else flt[i]=0;	//вход отпустило
	}

	//тест на срабатывание
	setBit(PORTB,TESTPIN);	//включить сигнал тест
	_delay_us(20);			//время срабатывания оптик и вх.усилителей
	for(i=0;i<NUMOPTINP;i++)
	{
		if(chkBit((READOPTINPUT | READDIGINPUT),i)==0)		//если вход не сработал...
		{
			ERROR=in[i].FLT = 1;							//ставим флаг неисправности и ошибку тестирования
			(i<4)?(setBit(DDRD,(i+4))):(setBit(DDRC,i));	//блокируем вход (делаем выход с "0")
		}
	}

	//тест на отпускание
	clrBit(PORTB,TESTPIN);	//выключить сигнал тест
	_delay_us(800);			//время отпускания оптик и вх.усилителей
	for(i=0;i<NUMOPTINP;i++)
	{
		if(chkBit((READOPTINPUT | READDIGINPUT),i)==1)//вход не отпустил
		{
			if(++flt[i] >= 3)	//три ретеста
			{
				ERROR=in[i].FLT = 1;							//ставим флаг неисправности и ошибку тестирования
				(i<4)?(setBit(DDRD,(i+4))):(setBit(DDRC,i));	//блокируем вход (делаем выход с "0")
				flt[i]=0;
			}
			else {/*countdown=255;*/return;}	//откладываем тест
		}
		else flt[i]=0;	//вход отпустило
	}

	//проверка ЦРЦ данных.Если ошибка то работать нельзя
	asm("cli");
	i=eeprom_read_byte(&dataCRC);
	asm("sei");
	if(i != CRC())
	{
		asm("cli");
		eeprom_update_byte(&error,DATACRC);
		asm("sei");
		setBit(PORTB,COM0);
		while(1)	//зависаем до прихода сообщения.если сообщение "прог" ЦРЦ пересчитается.
		{
			asm("wdr");
			if((PINA & (1<<DDA7))==0)asm("jmp 0");	//если нажата кнопка перезапускаем программу
			PORTA=0b01011110;
			clrBit(PORTB,COM1);
			_delay_ms(50);
			setBit(PORTB,COM1);
			PORTA=0x00;
			_delay_ms(50);
#ifdef DEBUG
			if(CANRXcounter || rx_counter)break;
#else
			if(CANRXcounter)break;
#endif
		}
	}

#ifdef BDZ0
	for(i=0;i<NUMINP;i++) (in[i].FLT)?(setBit(inputFailMemory,i)):(clrBit(inputFailMemory,i));
#endif

	flag.test=0;
}

static inline void updCRC (unsigned char *a_CRC,unsigned char *start,unsigned char lenght)
{
	for(unsigned char i=0;i<lenght;i++)	{asm("cli");*a_CRC= _crc_ibutton_update(*a_CRC,eeprom_read_byte(start+i));asm("sei");}
}

unsigned char CRC (void)
{
	unsigned char realCRC=0;

	updCRC(&realCRC,&netID,1);
	updCRC(&realCRC,(unsigned char *)time,sizeof(time));
	updCRC(&realCRC,o,sizeof(o));
	updCRC(&realCRC,cin,sizeof(cin));
	updCRC(&realCRC,cmod,sizeof(cmod));
	updCRC(&realCRC,(unsigned char *)don,sizeof(don));
	updCRC(&realCRC,(unsigned char *)doff,sizeof(doff));

	return realCRC;
}




//таймер 100мкс -сканирование входов
ISR(TIMER0_COMP_vect)
{
#define FILTER 5
	static unsigned char filters[4]={0,0,0,0};
	unsigned char result=0;

//clrBit(PORTD,3);
	flag.update=1;
	for(unsigned char k=0;k<NUMOPTINP;k++) if(in[k].ACT != chkBit(READOPTINPUT,k)) {flag.trigger=1;in[k].ACT = chkBit(READOPTINPUT,k);}
	for(unsigned char k=NUMOPTINP;k<NUMINP;k++)
	{
		if(chkBit(READDIGINPUT,k)) (filters[k-NUMOPTINP] < FILTER)?(filters[k-NUMOPTINP]++):(setBit(result,k));
		else {if(filters[k-NUMOPTINP] > 0) filters[k-NUMOPTINP]--; }
		if(in[k].ACT != chkBit(result,k)) {flag.trigger=1;in[k].ACT = chkBit(result,k);}
	}//ЦВХ без фильтра:	for(unsigned char k=NUMOPTINP;k<NUMINP;k++) if(in[k].ACT != chkBit(READDIGINPUT,k)) {flag.trigger=1;in[k].ACT = chkBit(READDIGINPUT,k);}
//setBit(PORTD,3);
}

//таймер 1мс -обработка таймаутов,хэндлеров и таймеров
ISR(TIMER1_COMPA_vect)
{
	static unsigned int test=0;
	static unsigned int ms=0;

	//часы:
	if(++ms>999){rtc.seconds++;ms=0;}
	if(rtc.seconds>59){rtc.minutes++;rtc.seconds=0;}
	if(rtc.minutes>59){rtc.hours++;rtc.minutes=0;}
	if(rtc.hours>23){rtc.day++;rtc.hours=0;}
	if(rtc.day>31){rtc.month++;rtc.day=1;}
	if(rtc.month>12){rtc.year++;rtc.month=1;}

	++sysClk;
	#ifdef DEBUG
	if(UART_timeout)UART_timeout--;
	#endif
	if(SPI_timeout)SPI_timeout--;
	if(CAN_timeout)CAN_timeout--;
	if(++test ==TEST_TIME) {flag.test=1;test=0;}
	if(waitMTZ_off)
	{
		--waitMTZ_off;
		//если вход подтверждения мтз отпустило,запоминаем время,останавливаем таймер
		if((in[inpWaitMTZ_off].ACT ==0) && (tmpArchieve.MTZoff==0xFFFF)) {tmpArchieve.MTZoff=sysClk;waitMTZ_off=0;}
	}

	if(flag.noTimers && flag.noHandlers)return;		 //нет обработчиков и таймеров-обрабатывать нечего
	proceedTimers();
	proceedHandlers();
}



