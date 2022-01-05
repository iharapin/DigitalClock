#include "mbed.h"
#include "TextLCD.h"

//Message defines
#define DEFAULT_BAUDRATE     9600 //Frekvencija serijske komunikacije 
#define SERIAL_BITS             8 //Konfiguracija serijske komunikacije
#define MESSAGE_START_LOCATION  1 //Mjesto na kojem započinje poruka nakon simbola
#define ALARM_TYPE            'A' //Oznaka za Alarm
#define TIME_TYPE             'T' //Oznaka za Vrijeme
#define DATE_TYPE             'D' //Oznaka za Datum
#define ASCII_OFFSET           48 //ASCII u integer transformacijska konstanta

//Više podataka o poruci: Sama poruka izgleda slično ovome -> A123456///
#define MESSAGE_SEGMENTS    3 //Segmenti poruke (Prvi: Sat/dan; Drugi: Minuta/Mjesec; Treći: Sekunde/Godina
#define BUFFER_SIZE        10 //Veličina primljenih podataka
#define MESSAGE_TYPE        0 //Simbol poruke (A,D ili T)
#define HOUR_DAY_START      1 //Od kud počinju sati/dani unutar polja
#define HOUR_DAY_END        2 //Gdje završavaju sati/dani unutar polja
#define MINUTE_MONTH_START  3 //Od kud počinju minute/mjeseci unutar polja
#define MINUTE_MONTH_END    4 //Gdje završavaju minute/mjeseci unutar polja
#define SECONDS_YEAR_START  5 //Od kud počinju sekunde/godine unutar polja
#define SECONDS_YEAR_END    6 //Gdje završavaju sekunde/godine unutar polja
//Maksimalne i minimalne vrijednosti primljenih podataka nakon obrade
#define HOUR_VALUE_MIN      0
#define HOUR_VALUE_MAX      23
#define MINUTE_VALUE_MIN    0
#define MINUTE_VALUE_MAX    59
#define SECONDS_VALUE_MIN   0
#define SECONDS_VALUE_MAX   59
#define DAY_VALUE_MIN       1
#define DAY_VALUE_MAX       31
#define WEEK_VALUE_MIN      0
#define WEEK_VALUE_MAX      6
#define MONTH_VALUE_MIN     1
#define MONTH_VALUE_MAX     12
#define YEAR_VALUE_MIN      0
#define YEAR_VALUE_MAX      99

//RTC i I2C
#define RTC_FREQUENCY     100000 //I2C frekvencija SCL linije
#define CONTROL_REGISTER_2  0x01
#define SLAVE_ADR           0x51
#define SECONDS_ADR         0x04
#define MINUTE_ADR          0x05
#define HOUR_ADR            0x06
#define DAY_ADR             0x07
#define WEEK_ADR            0x08
#define MONTH_ADR           0x09
#define YEAR_ADR            0x0A
#define RTC_BUFFER_SIZE        7 //Od 0-6 -> 7. mu je terminator

#define ALARM_SECONDS_ADR 0x0B
#define ALARM_MINUTES_ADR 0x0C
#define ALARM_HOURS_ADR   0x0D
#define ALARM_DAY_ADR     0x0E
#define ALARM_WEEKDAY_ADR 0x0F


#define ALARM_ENABLE 1
#define ALARM_DISABLE 0
#define ALARM_FREQ 0.5

#define DEC_SHIFT(shift) pow(10.0, shift)

Serial HC06(PTE0, PTE1); //UART za Bluetooth modul
Serial pc(USBTX, USBRX); //UART za računalo
I2C rtc(PTC9, PTC8); //I2C Master za RTC
InterruptIn RTC_alarm(D3); //Prekidni pin koji
InterruptIn button(D8);
Ticker buzzer;
DigitalOut buzzer_out(D4);
TextLCD lcd(PTB0,PTB1,PTB2,PTB3,PTC2,PTC1);
Timer refresh;


//Prototipi funkcija
void appInterrupt(); //Prekidna funkcija koja se poziva kada bluetooth primi poruku
void alarmInterrupt();
void buttonInterrupt();
void buzzerInterrupt();
void reciveFromApp(); //Funkcija za čitanje podataka s mobitela
void configDevice(); //Konfiguracija protokola (postavljanje frekvencija i slično)
void processMessage(char* message_from_app);
void RTCsetAlarm(uint8_t hour, uint8_t min, uint8_t sec);
void RTCgetAlarm(char* RTC_alarm);
void message2Int(char* message_from_app, int* data_out);
void RTCread(const char* address, char* data, uint8_t size);
void RTCwrite(char* address, char* data, uint8_t size);
void RTCgetTime(char* RTC_time);
void RTCgetDate(char* RTC_date);
void RTCsetTime(char hour, char min, char sec);
void RTCsetDate(char day, char month, char year, char week = 0);
uint8_t BCD2Decimal(uint8_t value);
uint8_t Decimal2BCD(uint8_t value);
void enableClearAlarm(bool enable);
uint8_t constrain(uint8_t value, uint8_t min, uint8_t max);
void printLCDdata();

char global_message_from_app[BUFFER_SIZE] = "";
bool interruptFlag = false;
    
int main(){
    configDevice();
    RTCsetTime(21,35,45);
    RTCsetDate(20,9,21);
     
    while(true) {
        
        if(refresh.read_ms() > 1000){
            printLCDdata();
            refresh.reset();
        }

       if(interruptFlag){
        processMessage(global_message_from_app);
        interruptFlag = false;
        }
    }
}


void configDevice(){    
    HC06.baud(DEFAULT_BAUDRATE);
    HC06.format(SERIAL_BITS,SerialBase::None);
    HC06.attach(&appInterrupt, Serial::RxIrq);  
    rtc.frequency(RTC_FREQUENCY);
    refresh.start();
    enableClearAlarm(ALARM_ENABLE);
    RTC_alarm.fall(&alarmInterrupt);
 
}

void appInterrupt(){
    interruptFlag = true;
    
    for(uint8_t i = 0; i < BUFFER_SIZE; i++){
        global_message_from_app[i] = HC06.getc();
    }
}

void alarmInterrupt(){
    buzzer.attach(&buzzerInterrupt, 0.5);
    button.rise(&buttonInterrupt);
}   

void buzzerInterrupt(){
    buzzer_out = !buzzer_out;
}

void buttonInterrupt(){
    buzzer.detach();
    buzzer_out = false;
    button.rise(NULL);
    enableClearAlarm(ALARM_ENABLE);
}

void processMessage(char* message_from_app){
    const char type = message_from_app[MESSAGE_TYPE]; //Čitamo o kojoj se vrsti poruke radi
    int message_data[MESSAGE_SEGMENTS] = {NULL}; //Pripremamo array veličine MESSAGE_SEGMENTS unutar kojeg će se pohraniti brojčane vrijednosti poruke
    message2Int(message_from_app, message_data); //Vadimo brojčane vrijednosti poruke (koje su tipa char) i pretvaramo ih u integere
       
    struct tm *time_now = NULL; //Struct unutar kojeg će se nalaze ljudski čitljivo vrijeme
    time_t rawtime = time(NULL); //Čitanje trenutnog raw vremena u sekundama (epoch)
    time_now = localtime(&rawtime); //Pretvaranje trenutnog raw vremena u ljudski čitljivo vrijeme   
    
    pc.printf("\nCase: %d %d %d\n",message_data[0],message_data[1],message_data[2]);
    
    switch (type){ //Ovisno o vrsti podatka radimo sljedeće:
    case ALARM_TYPE:{ //Obrada i postavljanje alarma
         RTCsetAlarm(message_data[0], message_data[1], message_data[2]);
    }break;
    
    case TIME_TYPE:{ //Obrada i postavljanje vremena
        time_now->tm_hour = message_data[0]; 
        time_now->tm_min = message_data[1];  
        time_now->tm_sec = message_data[2];
        RTCsetTime(message_data[0], message_data[1], message_data[2]);       
    }break;
    
    case DATE_TYPE:{ //Obrada i postavljanje datuma
        time_now->tm_mday = message_data[0]; 
        time_now->tm_mon = message_data[1];
        time_now->tm_year = message_data[2];
        RTCsetDate(message_data[0], message_data[1], message_data[2]);   
    }break;
  }
    if(type != ALARM_TYPE) set_time(mktime(time_now));
}

void message2Int(char* message_from_app, int* data_out){
    const uint8_t starts[RTC_BUFFER_SIZE] ={HOUR_DAY_START, MINUTE_MONTH_START, SECONDS_YEAR_START}; //Gdje svaki segment počinje unutar arraya
    const uint8_t ends[RTC_BUFFER_SIZE] ={HOUR_DAY_END, MINUTE_MONTH_END, SECONDS_YEAR_END};//Gdje svaki segment završava unutar arraya
    
    for(uint8_t i = 0; i < MESSAGE_SEGMENTS; i++){ //Podjela podatka u tri segemnta. Segmenti poruke (Prvi: Sat/dan; Drugi: Minuta/Mjesec; Treći: Sekunde/Godina)
        for(uint8_t j = starts[i]; j <= ends[i]; j++){ //Pretvara ASCII brojeve s više znamenki u integer (npr. '3' i '2' u 32)
            uint8_t ASCII_2_int = (message_from_app[j]-ASCII_OFFSET); //Pretvori ASCII broj (npr '3') u integer (iz '3' u 3)
            uint8_t digit_placement = DEC_SHIFT(ends[i]-j); //Određuje na kojem mjestu će se nalaziti znamenka koja se obrađuje (ako se nalazi na drugom mjestu onda će digit_placement bit 10)
            data_out[i] += ASCII_2_int * digit_placement; //Množenje pozicije znamenke sa znamenkom(npr. pretvorena ASCII znamenka je 3 i nalazi se na drugom  mjestu -> 3*10 = 30 -> te se zbraja s onim što se nalazi u polju data_out[i])
        }
    }
}

void enableClearAlarm(bool enable){ //Koristimo da postavimo alarm interrupt (AIE) bit u 1 (datashit str.13)
    const char bytes_to_send = 2;
    const char bit_mask = 0x80; //Osiguramo da svi bitovi prije sedmog 0
    const char data[bytes_to_send] = {CONTROL_REGISTER_2, //Registar u koji šaljemo podatak
                                     (enable << 7) & bit_mask}; //Podatak: može biti 0b10000000 za enable 1 ili 0b00000000 za enable 0
    
    rtc.write(SLAVE_ADR << 1, data, bytes_to_send, false);
}

void RTCread(const char* address, char* data, uint8_t size){
    rtc.write(SLAVE_ADR << 1, address, 1, true); //Postavljanje početne adrese za čitanje
    rtc.read((SLAVE_ADR << 1)| 0x01 ,data ,size, false); //Čitanje podataka od postavljenje adrese i nadalje
}

void RTCgetTime(char* RTC_time){
    const char start_adr = SECONDS_ADR;
    RTCread(&start_adr, RTC_time, RTC_BUFFER_SIZE);
    
    for(uint8_t i = 0; i <=(HOUR_ADR-SECONDS_ADR); i++){
       RTC_time[i] = BCD2Decimal(RTC_time[i]);
    }
}

void RTCgetDate(char* RTC_date){
    const char start_adr = DAY_ADR;
    RTCread(&start_adr, RTC_date, RTC_BUFFER_SIZE);
    
    for(uint8_t i = 0; i <= (YEAR_ADR-DAY_ADR); i++){
       RTC_date[i] = BCD2Decimal(RTC_date[i]);
    }
}

void RTCsetTime(char hour, char min, char sec){    
    const char bytes_to_send = 4;
    
    hour = constrain(hour, HOUR_VALUE_MIN, HOUR_VALUE_MAX);
    min = constrain(min, MINUTE_VALUE_MIN, MINUTE_VALUE_MAX);
    sec = constrain(sec, SECONDS_VALUE_MIN, SECONDS_VALUE_MAX);
    
    const char data[bytes_to_send] ={SECONDS_ADR,
                                    Decimal2BCD(sec),
                                    Decimal2BCD(min),
                                    Decimal2BCD(hour)};
  
    rtc.write(SLAVE_ADR << 1,data, bytes_to_send, false); 
}

void RTCsetDate(char day, char month, char year, char week){    
    const char bytes_to_send = 5;
    
    day = constrain(day, DAY_VALUE_MIN, DAY_VALUE_MAX);
    week = constrain(week, WEEK_VALUE_MIN, WEEK_VALUE_MAX);
    month = constrain(month, MONTH_VALUE_MIN, MONTH_VALUE_MAX);
    year = constrain(year, YEAR_VALUE_MIN, YEAR_VALUE_MAX);
    
    const char data[bytes_to_send] ={DAY_ADR,
                                    Decimal2BCD(day),
                                    Decimal2BCD(week),
                                    Decimal2BCD(month),
                                    Decimal2BCD(year)}; 
     
    rtc.write(SLAVE_ADR << 1, data, bytes_to_send, false); 
}

void RTCsetAlarm(uint8_t hour, uint8_t min, uint8_t day){
    const char bytes_to_send = 5;
    char enable_disable_mask = 0x7F; //osiguravamo da je sedmi bit uvijek 0 (treba bit nula kako bi se omogućio alarm) (0b01111111) (str. 13 datashit). Ako je invertiran onda će biti 1 (0b10000000)
    
    hour = constrain(hour, HOUR_VALUE_MIN, HOUR_VALUE_MAX);
    min = constrain(min, MINUTE_VALUE_MIN, MINUTE_VALUE_MAX);
    day = constrain(day, DAY_VALUE_MIN, DAY_VALUE_MAX);
    
    const char data[bytes_to_send] ={ALARM_SECONDS_ADR,
                                    ~enable_disable_mask,
                                    Decimal2BCD(min)  & enable_disable_mask,
                                    Decimal2BCD(hour) & enable_disable_mask,
                                    Decimal2BCD(day)  & enable_disable_mask};
  
    rtc.write(SLAVE_ADR << 1,data, bytes_to_send, false);
    enableClearAlarm(ALARM_ENABLE);
}


void RTCgetAlarm(char* RTC_alarm){
        const char start_adr = ALARM_SECONDS_ADR;
        RTCread(&start_adr, RTC_alarm, RTC_BUFFER_SIZE);
    
    for(uint8_t i = 0; i <=(ALARM_WEEKDAY_ADR-ALARM_SECONDS_ADR); i++){
       RTC_alarm[i] = BCD2Decimal(RTC_alarm[i]);
        }
}

uint8_t BCD2Decimal(uint8_t value){
    return ( (value >> 4) & 0x0F ) * 10 + (value & 0x0F);
}

uint8_t Decimal2BCD(uint8_t value){
    return ( (value/10) << 4 ) | (value%10);
}

uint8_t constrain(uint8_t value, uint8_t min, uint8_t max){
    if(value < min) return min;
    else if(value > max) return max;
    else return value;  
}

void printLCDdata(){
    
    char RTC_time[RTC_BUFFER_SIZE] = "";
    RTCgetTime(RTC_time); 
    char RTC_date[RTC_BUFFER_SIZE] = "";
    RTCgetDate(RTC_date);
    char RTC_alarm[RTC_BUFFER_SIZE] = "";
    RTCgetAlarm(RTC_alarm);
    const char* first_row = (RTC_time[0]%2) ? "%02d:%02d %02d.%02d.20%.02d": "%02d %02d %02d.%02d.20%.02d";
                                             
    lcd.cls();
    lcd.locate(0,0);
    lcd.printf(first_row, RTC_time[2], RTC_time[1],RTC_date[0],RTC_date[2], RTC_date[3]);
    
    lcd.locate(0,1);
    lcd.printf("ALM: %02d:%02d %02d.%02d", RTC_alarm[2],RTC_alarm[1],RTC_alarm[3], RTC_date[2]);   
    
}