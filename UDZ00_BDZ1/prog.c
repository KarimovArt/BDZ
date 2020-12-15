#define MAXTIME	32000						//максимальные времена МТЗ,УРОВ,задержек вкл/откл выходов,теста;сек

//входы:1-подтверждения,2-режим подтверждения,3-подключенные выходы
static inline void inputs(unsigned char n_io,unsigned char n_param,signed int value,char *answer);
//выходы:1-задержка включения,2-задержка выключения
static inline void outputs(unsigned char n_io,unsigned char n_param,signed int value,char *answer);
//задержки:1-МТЗ,2-УРОВ,3-тест
static inline void delay(unsigned char n_param,signed int value,char *answer);
//время/дата:1-время. str==(hh.mm.ss),2-дата. str==(dd.mm.yy) //"."-любой символ
static inline void datetime(unsigned char n_param,signed int value,char *str,char *answer);
//сетевой адрес (>0 & <127)
static inline void netid(signed int value,char *answer);
#ifdef DEBUG
static inline char printArch(char index,char cmd);
#endif
extern struct ARCHIEVE tmpArchieve;


char *prog(char *str)
{
	char answer[12]="=ERROR";
	unsigned char group=*str;			//группа команды
	unsigned char n_io=*(str+1)-'1';	//№входа/выхода
	unsigned char n_param=*(str+2);		//№параметра
	signed int value=-1;
	enum{ERR='0',INPUT,OUTPUT,DELAY,TIME,ARCH,ID};

	//вычисляем новое значение параметра(если оно есть)
	if(*(str+3)=='=')			//если четвертый символ команды "=" то это изменение параметра...иначе -чтение
	{
		value=0;
		if(group==INPUT) for(char i=0;i<4;i++) {forceBit(value,i,*(str+7-i)-'0');}	//;значение отличное от '0' воспринимается как '1'
		else value=atoi(str+4);
	}
	asm("cli");
	switch (group)
	{
	//Чтение текущей ошибки
	case ERR:itoa(ERROR,answer+1,10);break;

	//Конфигурация входов
	case INPUT:inputs(n_io,n_param,value,answer);
	break;

	//Задержки включения/выключения выходов.
	case OUTPUT:outputs(n_io,n_param,value,answer);
	break;

	//Времена МТЗ,УРОВ,тест
	case DELAY:delay(n_param,value,answer);
	break;

	//установка RTC
	case TIME:datetime(n_param,value,str,answer);
	break;

	//чтение архива
	case ARCH:
	{
#ifdef DEBUG
		char archIndex=(*(str+1)-'0')*10+*(str+2)-'0';
		if(printArch(archIndex,value) ==0) strcpy(answer+1,"OK");
#endif
	}
	break;

	//NETID
	case ID:
	{
		netid(value,answer);
	}
	break;

	default: goto EXIT;
	break;
	}
EXIT:
	strcpy(str,answer);

	eeprom_update_byte(&dataCRC,CRC());	//обновляем контрольную сумму данных
	asm("sei");
	flag.test=1;								//после программирования тест
	return 	str;
}


static inline void inputs(unsigned char n_io,unsigned char n_param,signed int value,char *answer)
{
	unsigned char *pParam,tmp;

	if(n_io >= NUMINP)return;				//ошибка №входа

	switch(n_param)
	{
		case '1': pParam=in[n_io].conf_in;break;	//входы подтверждений
		case '2': pParam=in[n_io].conf_mod;break;	//типы подтверждений 0-МТЗ 1-УРОВ
		case '3': pParam=in[n_io].outputs;break;	//подключенные выходы
		default:return;							//ошибка параметра
	}
	if(value >=0)eeprom_update_byte(pParam,(unsigned char)value);
	tmp=eeprom_read_byte(pParam);
	for(char i=0;i<4;i++) *(answer+4-i)=chkBit(tmp,i)+'0';	//отображаем в двоичном виде
	*(answer+5)=0;
}

static inline void outputs(unsigned char n_io,unsigned char n_param,signed int value,char *answer)
{
	unsigned int *pParam;

	if((value >MAXTIME)||(n_io >= NUMOUT)) return;	//ошибка значения времени или №выхода
	switch(n_param)
	{
		case '1': pParam=out[n_io].delay_on;break;			//задержка включения
		case '2': pParam=out[n_io].delay_off;break;			//...выключения
		default:return;									//ошибка параметра
	}
	if(value >=0)eeprom_update_word(pParam,value);		//если параметр <0 это чтение (обновлять не надо)
	itoa(eeprom_read_word(pParam),answer+1,10);
}

static inline void delay(unsigned char n_param,signed int value,char *answer)
{
	unsigned int *pParam;

	if(value >MAXTIME) return;		//ошибка значения времени или №выхода
	switch(n_param)
	{
		case '1': pParam=&time[0];break;	//МТЗ
		case '2': pParam=&time[1];break;	//УРОВ
		case '3': pParam=&time[2];break;	//тест
		default:return;
	}
	if(value >=0)eeprom_write_word(pParam,value);
	itoa(eeprom_read_word(pParam),answer+1,10);
}

static inline void datetime(unsigned char n_param,signed int value,char *str,char *answer)
{
	//используем tmpArchieve.rtc как временное хранилище
	unsigned char *pParam[3],separator;
	switch(n_param)
	{
	case '1':
	{
		pParam[0]=&tmpArchieve.rtc.hours;
		pParam[1]=&tmpArchieve.rtc.minutes;
		pParam[2]=&tmpArchieve.rtc.seconds;
		separator=':';
	}
	break;
	case '2':
	{
		pParam[0]=&tmpArchieve.rtc.day;
		pParam[1]=&tmpArchieve.rtc.month;
		pParam[2]=&tmpArchieve.rtc.year;
		separator='.';
	}
	break;
	default:return;
	}
	rtc_get(&tmpArchieve.rtc);	//В РТЦ пишется структура целиком. Это нужно для заполнения пустых полей структуры
	*pParam[0]=atoi(str+4);
	*pParam[1]=atoi(str+7);
	*pParam[2]=atoi(str+10);

	if(value >=0)
	{
		if(rtc_set(&tmpArchieve.rtc))return;
	}
	rtc_get(&tmpArchieve.rtc);
	sprintf(answer+1,"%02d%c%02d%c%02d",*pParam[0],separator,*pParam[1],separator,*pParam[2]);
}

#ifdef DEBUG
static inline char printArch(char index,char cmd)
{
	char buff[17];
	struct ARCHIEVE tmp;

	if((index<0)||(index >=ARCHSIZE)) return 1;
	if(cmd==0)	//стирание архива
	{
		clearArch();
		return 0;
	}
	readArch(index,&tmp);

	sprintf(buff,"DATE=%02d%c%02d%c%02d",tmp.rtc.day,'.',tmp.rtc.month,'.',tmp.rtc.year);
	uart_puts(buff);
	sprintf(buff,"TIME=%02d%c%02d%c%02d",tmp.rtc.hours,':',tmp.rtc.minutes,':',tmp.rtc.seconds);
	uart_puts(buff);
	strcpy(buff,"SOURCE=");
	for(char i=0;i<NUMINP;i++)sprintf(buff+i+7,"%d",chkBit(tmp.source,(7-i)));
	uart_puts(buff);
	strcpy(buff,"CONFIRM=");
	for(char i=0;i<NUMINP;i++)sprintf(buff+i+8,"%d",chkBit(tmp.confirm,(7-i)));
	uart_puts(buff);
	strcpy(buff,"OUTS=");
	for(char i=0;i<NUMOUT;i++)sprintf(buff+i+5,"%d",chkBit(tmp.outs,(3-i)));
	uart_puts(buff);
	strcpy(buff,"MTZon=");
	itoa(tmp.MTZon,buff+6,10);
	uart_puts(buff);
	strcpy(buff,"MTZoff=");
	itoa(tmp.MTZoff,buff+7,10);
	uart_puts(buff);

	return 0;
}
#endif

static inline void netid(signed int value,char *answer)
{
	if((value > MAX_BDZ_ID)||(value==0))return;				//адрес д.б. в пределах 1...MAX_BDZ_ID

	if(value >0)eeprom_update_byte(&netID,value);
	itoa(eeprom_read_byte(&netID),answer+1,10);
}
