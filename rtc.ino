#include <SPI.h>
#include <Wire.h>
#include "RTClib.h"

RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

void setup() {

Serial.begin(9600);

if (! rtc.begin()) {
Serial.println("Couldn't find RTC");
while (1);
}

rtc.adjust(DateTime(__DATE__, __TIME__));
Serial.print("  Clock ");
}

void loop(){
DateTime now = rtc.now();

Serial.print(now.second(), DEC);
Serial.print("s :");
Serial.print(now.minute(), DEC);
Serial.print("m :");
Serial.print(now.hour(), DEC);
Serial.print("h :");
Serial.print(now.day(), DEC);
Serial.print("-");
Serial.print(now.month(), DEC);
Serial.print("-");
Serial.print(now.year(), DEC);
Serial.print("-");
Serial.println(daysOfTheWeek[now.dayOfTheWeek()]);
delay (100);
}