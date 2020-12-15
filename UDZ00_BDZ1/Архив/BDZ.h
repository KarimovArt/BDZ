#ifndef BDZ_H_
#define BDZ_H_

#define NUMOPTINP 4						//кол-во опт.входов
#define NUMDIGINP 4						//кол-во цифр.входов
#define NUMINP (NUMOPTINP+NUMDIGINP)	//кол-во входов
#define NUMOUT	4						//кол-во выходов
#define NUMTIM	250						//кол-во таймеров
#define NUMHAND	20						//кол-во обработчиков
#define COM0 0							//общий 0 СД
#define COM1 1							//общий 1 СД
#define TESTPIN 3						//нога для теста
#define KEYPIN 7						//нога кнопки
#define CANPIN 2						//нога наличия CAN- сообщения

#define setBit(byte,bit) (byte |= 1<<bit)
#define clrBit(byte,bit) (byte &= ~(1<<bit))
#define invBit(byte,bit) (byte ^= 1<<bit)
#define chkBit(byte,bit) ((byte & (1<<bit))?(1):(0)) //возвращает 1 если бит установлен
#define forceBit(byte,bit,val) (val)?(setBit(byte,bit)):(clrBit(byte,bit))	//устанавливает или сбрасывает бит
//#define chkBit(byte,bit) ((byte & 1<<bit)>>bit) //возвращает 1 если бит установлен

struct FLAGS {
	unsigned char noEmpty :1;				//временный архив НЕ пуст
	unsigned char saveArch :1;				//есть что сохранять в архив
	volatile unsigned char alarm :1;		//зафиксирована авария
	volatile unsigned char trigger :1;		//была сработка(для ОВХ) или изменение состояния(для ЦВХ)
	volatile unsigned char noTimers :1;		//таймеров нет
	volatile unsigned char noHandlers :1;	//обработчиков нет
	volatile unsigned char test :1;			//пора протестироваться
	volatile unsigned char update :1;		//обновление индикации
} flag;

struct INPUTS	//структура входов
{
//флаги
	volatile unsigned char ACT :1;	//вход активен
	unsigned char FLT :1;			//неисправность
	unsigned char ALM :1;			//срабатывал(источник аварии)
	unsigned char CONF :1;			//источник подтверждения

	unsigned char *conf_in;			//входы подтверждения(указатель на ЕЕПРОМ)
	unsigned char *conf_mod;		//режимы подтверждения(0-MTZ, 1-UROV)
	unsigned char *outputs;			//подключенные выходы (младшая тетрада) (0b0000xxxx  (K3,K2,K1,K0))
	volatile unsigned char handlers;//счетчик запущенных обработчиков для входа
};

struct OUTPUTS	//структура выходов
{
//unsigned char FLT:1;	//неисправность
	unsigned char ON :1;				//было срабатывание
	unsigned char OFF :1;			//было отпускание

	unsigned int *delay_on;			//задержка включения (указатель на ЕЕПРОМ)
	unsigned int *delay_off;		//задержка выключения

	volatile unsigned char *port;	//порт и пин к кот подключено реле
	const unsigned char pin;
};

//ОБРАБОТЧИКИ ВХОДОВ
enum {MTZ, UROV, NOCONF};
struct HANDLER		//обработчик создается для входа
{
	unsigned char Nin;	//№ входа, который обрабатывается
	unsigned char NconfInput;	//№ входа подтверждения
	unsigned char mode;		//режим обработчика (МТЗ,УРОВ)
	unsigned int time;		//время работы обработчика
};

//ТАЙМЕРЫ ВЫХОДОВ
enum {
	T_ON, T_OFF
};
struct TIMER		//таймер создается для выхода
{
	unsigned char mode;		//режим таймера (ON,OFF)
	unsigned int time;		//время работы таймера
	unsigned char Nin;//№ входа,вызвавший таймер.это нужно для разблокировки входа
	unsigned char Nout;	 	//обрабатываемый выход
};

#endif /* BDZ_H_ */
