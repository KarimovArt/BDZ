//архивация
#define ARCHSIZE 32
EEMEM static unsigned char EEwr_index=0;





struct ARCHIEVE
{
	struct RTC rtc;
	unsigned char source; 		//источник(и) срабатывания
	unsigned char confirm; 		//источники подтверждения
	unsigned int  MTZon,MTZoff;	//время прихода подтверждения МТЗ УРОВ
	unsigned char outs;			//сработавшие выходы
};

EEMEM struct ARCHIEVE archieve[ARCHSIZE];


//чтение архива
void readArch(unsigned char rd_index,struct ARCHIEVE *arch)
{
	asm("cli");
	char rd_index0= (eeprom_read_byte(&EEwr_index)==0)?(ARCHSIZE-1):(eeprom_read_byte(&EEwr_index)-1);	//адрес последней записи

	//вычисление абсолютного адреса архива
	rd_index=(rd_index > rd_index0)?(ARCHSIZE-(rd_index-rd_index0)):(rd_index0-rd_index);

	arch->rtc.year=eeprom_read_byte(&archieve[rd_index].rtc.year);
	arch->rtc.month=eeprom_read_byte(&archieve[rd_index].rtc.month);
	arch->rtc.day=eeprom_read_byte(&archieve[rd_index].rtc.day);
	arch->rtc.hours=eeprom_read_byte(&archieve[rd_index].rtc.hours);
	arch->rtc.minutes=eeprom_read_byte(&archieve[rd_index].rtc.minutes);
	arch->rtc.seconds=eeprom_read_byte(&archieve[rd_index].rtc.seconds);
	arch->source=eeprom_read_byte(&archieve[rd_index].source);
	arch->confirm=eeprom_read_byte(&archieve[rd_index].confirm);
	arch->outs=eeprom_read_byte(&archieve[rd_index].outs);
	arch->MTZon=eeprom_read_word(&archieve[rd_index].MTZon);
	arch->MTZoff=eeprom_read_word(&archieve[rd_index].MTZoff);
	asm("sei");
//	char tmp[10];
//	uart_putchar('r');
//	uart_puts(itoa(index,tmp,10));
}

void saveArch(struct ARCHIEVE *tmpArchieve)
{
	asm("cli");
	unsigned char wr_index=eeprom_read_byte(&EEwr_index);

	eeprom_write_byte(&archieve[wr_index].rtc.year,tmpArchieve->rtc.year);
	eeprom_write_byte(&archieve[wr_index].rtc.month,tmpArchieve->rtc.month);
	eeprom_write_byte(&archieve[wr_index].rtc.day,tmpArchieve->rtc.day);
	eeprom_write_byte(&archieve[wr_index].rtc.hours,tmpArchieve->rtc.hours);
	eeprom_write_byte(&archieve[wr_index].rtc.minutes,tmpArchieve->rtc.minutes);
	eeprom_write_byte(&archieve[wr_index].rtc.seconds,tmpArchieve->rtc.seconds);
	eeprom_write_byte(&archieve[wr_index].source,tmpArchieve->source);
	eeprom_write_byte(&archieve[wr_index].confirm,tmpArchieve->confirm);
	eeprom_write_byte(&archieve[wr_index].outs,tmpArchieve->outs);
	eeprom_write_word(&archieve[wr_index].MTZon,tmpArchieve->MTZon);
	eeprom_write_word(&archieve[wr_index].MTZoff,tmpArchieve->MTZoff);

	if(++wr_index ==ARCHSIZE)wr_index=0;
	eeprom_write_byte(&EEwr_index,wr_index);

#ifdef BDZ0
	sendAlarmMsg(tmpArchieve);
#endif

	//обнуляем источники,подтверждения,выходы. (т.к. они устанавливаются побитно-их состояния сохраняются-значит надо обнулить)
	tmpArchieve->source=tmpArchieve->confirm=tmpArchieve->outs=0;
	//во времена пишем FF
	tmpArchieve->MTZon=tmpArchieve->MTZoff=0xFFFF;
	flag.saveArch=0;							//архив сохранен
	flag.noEmpty=0;								//даем разрешение на запись во временный архив

asm("sei");

//отображение последней аварии в консоль
#ifdef DEBUG
//	char tmp[10]="500";
	extern char *prog(char *str);
	uart_puts(prog("500"));
#endif

}

void clearArch(void)
{
	//в пустом архиве-год==0 (или FF)
	asm("cli");
	for(unsigned char i=0;i<ARCHSIZE;i++) eeprom_write_byte(&archieve[i].rtc.year,0xFF);
	asm("sei");
}

