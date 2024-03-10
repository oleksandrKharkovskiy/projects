#include <mega32.h>
#include <alcd.h>
#include <sdcard.h>
#include <stdio.h>
#include <delay.h>

int t,L, Lh,L1,T,Th,Tl;
float pi = 3.14158;
float d = 0.007;// діаметр вала двигуна 7 мм
int w = 25; // оберти двигуна в секунду 

void start_motor(){
PORTB.1 = 1;//керування мотором
PORTB.2 = 1;
PORTD.6 = 1;//лампа яка показує що система запущена
PORTD.4 = 0;//лампа яка показує що система перейшла в ручний режим/вимкнулась
PORTB.0 = 0; //лампа очікування смикання
OCR2 = 0x9F;
TCCR2 = 0x11;
TIMSK=(1<<OCIE1A);//увімкнути переривання


}
void stop_motor(){
PORTB.1 = 0;//керування мотором
PORTB.2 = 0;
PORTD.6 = 0;//лампа яка показує що система запущена
PORTD.4 = 1;//лампа яка показує що система перейшла в ручний режим/вимкнулась
PORTB.0 = 0;//лампа очікування смикання
OCR2 = 0x00;
TCCR2 = 0x00;
TIMSK=(0<<OCIE1A);//вимкнути переривання
}
void cord_left(){ //функція підрахунку довжини замотаної на катушку ліски 
  T = t/55;
  Th = T/10;
  Tl = T - Th*10;
 
 L = (w*d*pi*T) ; 
 Lh = L/10;
 L1 = L - Lh* 10;
 lcd_gotoxy(1,2) ;
 lcd_putsf("Cord is pulled:")   ;
 lcd_gotoxy(3,3);
 lcd_putchar(Lh+0x30);  
 lcd_gotoxy(4,3);
 lcd_putchar(L1+0x30);
 lcd_gotoxy(6,3) ;
 lcd_putsf("meters") ;
 lcd_gotoxy(0,1);
 lcd_putsf("                 ");
 lcd_gotoxy(1,0);
 lcd_putsf("Fish on a hook!");
 

}

void end_pulling(){  //функція завершення роботи системи
lcd_gotoxy(0,0);
lcd_putsf("   Motor stop   ");
lcd_gotoxy(1,2);
lcd_putsf("Check the hook ");
lcd_gotoxy(0,3);
lcd_putsf("            ") ;
lcd_gotoxy(0,1);
lcd_putsf("         ") ;


}

interrupt [EXT_INT0] void ext_int0_isr(void)
{

stop_motor();
 

}

interrupt [EXT_INT1] void ext_int1_isr(void)
{
start_motor();
 
}

interrupt [TIM1_COMPA] void timer1_compa_isr(void)
{
disk_timerproc();
t++;
}


void main(void)
{
char data;

// Port A initialization
// Function: Bit7=Out Bit6=Out Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRA=(0<<DDA7) | (0<<DDA6) | (0<<DDA5) | (0<<DDA4) | (0<<DDA3) | (0<<DDA2) | (0<<DDA1) | (0<<DDA0);
// State: Bit7=0 Bit6=0 Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTA=(0<<PORTA7) | (0<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (0<<PORTA3) | (0<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);

// Port B initialization
// Function: Bit7=Out Bit6=In Bit5=Out Bit4=Out Bit3=In Bit2=In Bit1=In Bit0=In 
DDRB=(1<<DDB7) | (0<<DDB6) | (1<<DDB5) | (1<<DDB4) | (1<<DDB3) | (1<<DDB2) | (1<<DDB1) | (0<<DDB0);
// State: Bit7=0 Bit6=T Bit5=0 Bit4=0 Bit3=T Bit2=T Bit1=T Bit0=T 
PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Port C initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
DDRC=(1<<DDC7) | (1<<DDC6) | (1<<DDC5) | (1<<DDC4) | (1<<DDC3) | (1<<DDC2) | (1<<DDC1) | (1<<DDC0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
PORTC=(0<<PORTC7) | (0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization
// Function: Bit7=Out Bit6=Out Bit5=In Bit4=In Bit3=In Bit2=In Bit1=Out Bit0=In 
DDRD=(1<<DDD7) | (1<<DDD6) | (1<<DDD5) | (0<<DDD4) | (0<<DDD3) | (0<<DDD2) | (1<<DDD1) | (0<<DDD0);
// State: Bit7=0 Bit6=0 Bit5=T Bit4=T Bit3=T Bit2=T Bit1=0 Bit0=T 
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: Timer 0 Stopped
// Mode: Normal top=0xFF
// OC0 output: Disconnected
TCCR0=(0<<WGM00) | (0<<COM01) | (0<<COM00) | (0<<WGM01) | (0<<CS02) | (0<<CS01) | (0<<CS00);
TCNT0=0x00;
OCR0=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 4000,000 kHz
// Mode: Normal top=0xFFFF
// OC1A output: Disconnected
// OC1B output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 16,384 ms
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: On
// Compare B Match Interrupt: Off
TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10);
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x01;
OCR1AL=0x90;
OCR1BH=0x00;
OCR1BL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: 4000,000 kHz
// Mode: Normal top=0xFF
// OC2 output: Disconnected
// Timer Period: 0,064 ms
ASSR=0<<AS2;
TCCR2=(0<<PWM2) | (0<<COM21) | (0<<COM20) | (0<<CTC2) | (0<<CS22) | (0<<CS21) | (1<<CS20);
TCNT2=0xFF;
OCR2=0x00;

// Timer(s)/Counter(s) Interrupt(s) initialization
TIMSK=(0<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (1<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1) | (0<<OCIE0) | (0<<TOIE0);

// External Interrupt(s) initialization
// INT0: On
// INT0 Mode: Any change
// INT1: On
// INT1 Mode: Rising Edge
// INT2: Off
GICR|=(1<<INT1) | (1<<INT0) | (0<<INT2);
MCUCR=(1<<ISC11) | (1<<ISC10) | (0<<ISC01) | (1<<ISC00);
MCUCSR=(0<<ISC2);
GIFR=(1<<INTF1) | (1<<INTF0) | (0<<INTF2);

// USART initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART Receiver: On
// USART Transmitter: On
// USART Mode: Asynchronous
// USART Baud Rate: 4800
UCSRA=(0<<RXC) | (0<<TXC) | (0<<UDRE) | (0<<FE) | (0<<DOR) | (0<<UPE) | (0<<U2X) | (0<<MPCM);
UCSRB=(0<<RXCIE) | (0<<TXCIE) | (0<<UDRIE) | (1<<RXEN) | (1<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);
UCSRC=(1<<URSEL) | (0<<UMSEL) | (0<<UPM1) | (0<<UPM0) | (0<<USBS) | (1<<UCSZ1) | (1<<UCSZ0) | (0<<UCPOL);
UBRRH=0x00;
UBRRL=0x33;

// Analog Comparator initialization
// Analog Comparator: Off
// The Analog Comparator's positive input is
// connected to the AIN0 pin
// The Analog Comparator's negative input is
// connected to the AIN1 pin
ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
SFIOR=(0<<ACME);

// ADC initialization
// ADC disabled
ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);

// SPI initialization
// SPI disabled
SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

// TWI initialization
// TWI disabled
TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);

// Alphanumeric LCD initialization
// Connections are specified in the
// Project|Configure|C Compiler|Libraries|Alphanumeric LCD menu:
// RS: PORTC Bit 0
// RD: PORTC Bit 1
// EN: PORTC Bit 2
// D4: PORTC Bit 4
// D5: PORTC Bit 5
// D6: PORTC Bit 6
// D7: PORTC Bit 7
// Characters/line: 16


//// Watchdog Timer initialization
//// Watchdog Timer Prescaler: OSC/2048k
//WDTCR=(0<<WDTOE) | (1<<WDE) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);

// Globally enable interrupts
#asm("sei")

TIMSK=(0<<OCIE1A)   ;
lcd_init(16);
lcd_gotoxy(5,1);
lcd_putsf("Hello!");
delay_ms(2000);
lcd_clear();
PORTB.0 = 1;


while (1)
      {     
           if (PORTB.0 == 1 ){ 
            lcd_gotoxy(5,1);
            lcd_putsf("Waiting");
            lcd_gotoxy(3,2);
            lcd_putsf("for a bite"); }
           
            else if (PORTD.6 == 1){
                cord_left();}       
           
            else if(PORTD.4 == 1){
                end_pulling();
                putsf("Fish was pulled out in \r");
                 putchar(Th+0x30);
                 putchar(Tl+0x30); 
                
                putsf(" seconds \r");  
                putsf("\r");
                putsf("The distance from which the fish was pulled \r");
                putchar(Lh+0x30);
                putchar(L1+0x30);
                putsf(" meters \r");
                
                
                break;
                }

      }
}

