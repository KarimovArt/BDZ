

#ifndef rtc_H_
#define rtc_H_

volatile struct RTC
{
	unsigned char year;
	unsigned char month;
	unsigned char day;
	unsigned char hours;
	unsigned char minutes;
	unsigned char seconds;
}rtc={1,1,1,0,0,0};	//системные часы нереального времени. инициализируем 01.01.01 00:00:00

unsigned char rtc_set(struct RTC *tmp);
void rtc_get(struct RTC *tmp);


unsigned char rtc_set(struct RTC *tmp)
{
	if(tmp->year>99 || tmp->month>12 || tmp->day>31 || tmp->hours>23 || tmp->minutes>59 || tmp->seconds>59) {ERROR=RTC;return 1;}

	rtc=*tmp; 	//записываем новое время

	return 0;

}

inline void rtc_get(struct RTC *tmp)
{
	*tmp=rtc;

//	tmp->year=rtc.year;
//	tmp->month=rtc.month;
//	tmp->day=rtc.day;
//	tmp->hours=rtc.hours;
//	tmp->minutes=rtc.minutes;
//	tmp->seconds=rtc.seconds;
}


#endif
