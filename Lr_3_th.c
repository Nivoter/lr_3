// PIC18F4525 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = HS      // Oscillator Selection bits (HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled and controlled by software (SBOREN is enabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#define _XTAL_FREQ 10000000


#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#define lcd_clear() lcd_command(1)
#define lcd_origin() lcd_command(2)
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 40000000 // ii?aaaeaiea oaeoiaie ?anoiou  40 IAo  aey ooieoee __delay_ms
#endif
#define E_pulse_with 50 //aeeoaeuiinou oaeoiauo eiioeunia LCD a ien
#define LCD_E PORTDbits.RD3
#define LCD_RS PORTDbits.RD2
#define LCD_Data4 LATD // eieoeaeecaoey ii?oia D4-D7 LCD
void lcd_clk(void) /*aaia?aoey eiioeuna ia aoia EN*/
{
  LCD_E = 1;
  __delay_us(E_pulse_with);
  LCD_E = 0;
  __delay_us(E_pulse_with);
}
void lcd_command(unsigned char outbyte) /*ioi?aaeou eiiaiao (4-aeoiue ?a-?ei ?aaiou) */
{
  LCD_RS=0; //?a?ei ia?aaa?e eiiaiau
  LCD_Data4=(LCD_Data4&0x0f)|(outbyte&0xf0); // ioi?aaea noa?oeo ?aou?ao aeo
  lcd_clk();
  LCD_Data4=(LCD_Data4&0x0f)|((outbyte<<4)&0xf0); // ioi?aaea ieaaoeo ?a-ou?ao aeo
  lcd_clk();
  __delay_ms(1);
}
void lcd_putc(char outbyte) /* ioi?aaeou aaiiua (4-aeoiay iia?aoey) */
{
  LCD_RS=1; //?a?ei ia?aaa?e aaiiuo
  LCD_Data4=(LCD_Data4&0x0f)|(outbyte&0xf0);
  lcd_clk();
  LCD_Data4=(LCD_Data4&0x0f)|((outbyte<<4)&0xf0);
  lcd_clk();
}

void lcd_puts(unsigned char line,const char *p) // *auaia no?iee ia ye?ai*
{
	lcd_origin();         // ia?aoia e ioeaaiio aa?ano LCD
	lcd_command(line);			// onoaiiaeou aa?an LCD 00H
	while(*p)                  // i?iaa?eou, ?aaai ee oeacaoaeu 0
	{
	 lcd_putc(*p);             // ioi?aaeou aaiiua ia LCD
	 p++;                      // oaaee?eou aa?an ia 1
	}
}
void inttolcd(unsigned char posi, unsigned long value) //auaia ia ye?ai cia?aiee ia?a-iaiiuo
{
	char buff[16];
	utoa(buff,value,10);
	lcd_puts(posi,buff);
}
void lcd_init() // eieoeaeecaoey LCD-aenieay
{
  TRISD &= 0x03;// ia?aaia auaiaia RD4-RD7a ?a?ei auoiaia
  LCD_Data4 &= 0b00001111;//onoaiiaea ioeae ia eeiee ia?aaa?e aaiiuo
  LCD_E=0;
  LCD_RS=0;
  __delay_ms(1);
/*eieoeaeecaoey aenieay*/
  LCD_Data4=(LCD_Data4&0x0f)|0x30;
  lcd_clk();
 __delay_ms(1);
  LCD_Data4=(LCD_Data4&0x0f)|0x30;
 lcd_clk();
__delay_ms(1);
  LCD_Data4=(LCD_Data4&0x0f)|0x30;
  lcd_clk();
  __delay_ms(1);
/*---------------------------------*/
  LCD_Data4=(LCD_Data4&0x0f)|0x20;	// ia?aee??eou ia 4-aeoiue ?a?ei ia-?a-aa?e
  lcd_clk();
  __delay_ms(1);
  lcd_command(0x28);//onoaiiaeou N=1, F=0 (aaa no?iee, ?acia? neiaiea 5*8 oi?ae
  lcd_command(0x01);	// i?enoeou an?
  lcd_command(0x06);	// aaoiiaoe?anee ia?aaaeioou eo?ni? iinea aaeoa
  lcd_command(0x0C);	// aenieae aee???i, eo?ni?a iao, ia ii?aaao
  lcd_command(0x02);	// ia?aeuiay iiceoey
  lcd_command(0x01);	// i?enoeou an? niiaa
}
void Adc_init()
 {
 TRISA|=0b11111111; //ia?aaia auaiaia RA0, RA1, RA2, RA3, RA5 a ?a?ei aoiaia

 TRISE|=0b00000111; //ia?aaia auaiaia RE0, RE1, RE2 a ?a?ei aoiaia
 ADCON1bits.PCFG=0b0111; // eiioeao?aoey aiaeiai-oeo?iauo ii?oia
 ADCON1bits.VCFG=0b00; // iii?iia iai?y?aiea Vss Vdd
 ADCON2bits.ACQT=0b111;// a?aiy i?aia?aciaaiey 20 Tad
 ADCON2bits.ADCS=0b110;// ?anoioa i?aia?aciaaiey Fosc/64 

 ADCON2bits.ADFM=0;//eaaia niauaiea
 ADCON0bits.ADON=1; // iiaoeu AOI aee??ai
 }
int read_Adc()
 {
 ADCON0bits.GO_DONE=1; // caione i?aia?aciaaiey
 while(ADCON0bits.GO_DONE==1);
 return (ADRESH<<2)+(ADRESL>>6);//caienu ?acoeuoaoa i?aia?aciaaiey
 } 

volatile unsigned long i=0;
volatile unsigned long g=0;
volatile unsigned long k=0;
volatile unsigned long n=0;

unsigned int time_h = 216;
unsigned int time_l = 239;

//unsigned int time_h = 39;
//unsigned int time_l = 16;


void interrupt HIisr (void)
{
 if (INTCONbits.INT0IF)
 {
 __delay_ms(10);// задержка для исключения влияния «дребезга» контак-
 __delay_ms(10);
 __delay_ms(10);
    if (PORTBbits.RB0==0)
    {
        if(T0CONbits.TMR0ON==1)
        {
            T0CONbits.TMR0ON=0;
            
        }
        else
        {
       T0CONbits.TMR0ON=1;
       TMR0H=time_h;
       TMR0L=time_l;
        }
       
        
    }
    
    __delay_ms(10);
   inttolcd(0x87, k);
   
  
 } 
 INTCONbits.INT0IF=0;
}

void interrupt low_priority LIisr (void)
{
    
        INTCONbits.TMR0IF=0;
        TMR0H=time_h;
        TMR0L=time_l;
        
        k++;
    
}
int main(int argc, char** argv) {
lcd_init();
TRISBbits.RB3=0;//íàñòðîéêà RB3 íà âûõîä
T0CONbits.T08BIT=0;//íàñòðîéêà òàéìåðà ¹0 íà 16-áèòíûé ðåæèì ðàáîòû
T0CONbits.PSA=1;//ðàçðåøåíèå èñïîëüçîâàòü ïðåääåëèòåëü
T0CONbits.T0PS=0b001;//ïðåääåëèòåëü ðàâåí 256
T0CONbits.T0CS=0;//âûáîð âíóòðåííåãî èñòî÷íèêà òàêòîâûõ èìïóëüñîâ
T0CONbits.TMR0ON=0;
//çàïèñü ìëàäøåãî áàéòà íà÷àëüíîãî çíà÷åíèÿ 
//T0CONbits.TMR0ON=1;
RCONbits.IPEN=1; // ðàçðåøåíèå äâóõóðîâíåâûõ ïðåðûâàíèé
INTCON2bits.TMR0IP=0;//ïðèñâîåíèå ïðåðûâàíèþ íèçêîãî ïðèîðèòåòà
INTCONbits.TMR0IF=0;//îáíóëåíèå ôëàãà ïðåðûâàíèÿ ïî ïåðåïîëíåíèþ òàéìåðà 0
INTCONbits.GIEH=1; //ðàçðåøåíèå âûñîêîóðîâíåâûõ ïðåðûâàíèé
INTCONbits.GIEL=1; //ðàçðåøåíèå íèçêîóðîâíåâûõ ïðåðûâàíèé
INTCONbits.TMR0IE=1;//ðàçðåøåíèå ïðåðûâàíèÿ ïî ïåðåïîëíåíèþ òàéìåðà 0
TRISBbits.RB3=0;//настройка RB3 на выход
 TRISBbits.RB0=1;//настройка RB0 на вход
 RCONbits.IPEN=1; // разрешение двухуровневых прерываний
 INTCON2bits.INTEDG0=0; //прерывание по ниспадающему фронту сигнала
 INTCONbits.INT0IF=0; //обнуление флага прерывания от внешнего источ-
 INTCONbits.GIEH=1; //разрешение высокоуровневых прерываний
 INTCONbits.INT0IE=1;//разрешение прерывания от внешнего источника
 LATBbits.LATB3=1; 
 while(1)
 {
     //i=(TMR0H<<8)|TMR0L;
     //i=(i<<0);
      
          inttolcd(0xC7, k);
          
      /*if(i>100000000000)
      {
          g++;
          i=0;
          
      }
      inttolcd(0x80, g);*/
 } 
}