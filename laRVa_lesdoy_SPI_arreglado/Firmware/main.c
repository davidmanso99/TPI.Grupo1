
#include <stdint.h>
typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned int   u32;

typedef signed char  s8;
typedef signed short s16;
typedef signed int   s32;


//-- Registros mapeados
////////////////// UART //////////////////

	//-- UART0 --//
#define UARTDAT  (*(volatile uint8_t*)0xE0000000)
#define UARTSTA  (*(volatile uint32_t*)0xE0000004)
#define UARTBAUD (*(volatile uint32_t*)0xE0000004)
	//-- UART1 --//
#define UARTDAT2  (*(volatile uint8_t*)0xE0000008)
#define UARTSTA2  (*(volatile uint32_t*)0xE000000C)
#define UARTBAUD2 (*(volatile uint32_t*)0xE000000C)
	//-- UART2 --//
#define UARTDAT3  (*(volatile uint8_t*)0xE0000010)
#define UARTSTA3  (*(volatile uint32_t*)0xE0000014)
#define UARTBAUD3 (*(volatile uint32_t*)0xE0000014)

////////////////// SPI //////////////////

	//-- SPI0 --//
#define SPIDAT	 (*(volatile uint32_t*)0xE0000020)
#define SPICTL	 (*(volatile uint32_t*)0xE0000024)
#define SPISTA	 (*(volatile uint32_t*)0xE0000024)
#define SPISS	 (*(volatile uint32_t*)0xE0000028)

	//-- SPI1 --//
//#define SPIDAT2	 (*(volatile uint32_t*)0xE0000070)
//#define SPICTL2	 (*(volatile uint32_t*)0xE0000074)
//#define SPISTA2	 (*(volatile uint32_t*)0xE0000074)
//#define SPISS2	 (*(volatile uint32_t*)0xE0000078)

////////////////// I2C //////////////////

#define I2CDAT	 (*(volatile uint32_t*)0xE0000040)
#define I2CCTL	 (*(volatile uint32_t*)0xE0000044)

////////////////// GPOUT //////////////////

#define GPOUT	 (*(volatile uint32_t*)0xE0000080)
#define GPIN	 (*(volatile uint32_t*)0xE0000084)

#define TCNT     (*(volatile uint32_t*)0xE0000060)

////////////////// INTERRUPTS //////////////////

#define IRQEN	 (*(volatile uint32_t*)0xE00000C0)
#define IRQVECT0 (*(volatile uint32_t*)0xE00000E0)
#define IRQVECT1 (*(volatile uint32_t*)0xE00000E4)
#define IRQVECT2 (*(volatile uint32_t*)0xE00000E8)
#define IRQVECT3 (*(volatile uint32_t*)0xE00000EC)
#define IRQVECT4 (*(volatile uint32_t*)0xE00000F0)
#define IRQVECT5 (*(volatile uint32_t*)0xE00000F4)
#define IRQVECT6 (*(volatile uint32_t*)0xE00000F8)
#define IRQVECT7 (*(volatile uint32_t*)0xE00000FC)

void delay_loop(uint32_t val);	// (3 + 3*val) cycles
#define CCLK (18000000)
#define _delay_us(n) delay_loop((n*(CCLK/1000)-3000)/3000)
#define _delay_ms(n) delay_loop((n*(CCLK/1000)-30)/3)



void _putch(int c)
{
	while((UARTSTA&2)==0);
	//if (c == '\n') _putch('\r');
	UARTDAT = c;
}

void _puts(const char *p)
{
	while (*p)
		_putch(*(p++));
}
/*
uint8_t _getch()
{
	while((UARTSTA&1)==0);
	return UARTDAT;
}

uint8_t haschar() {return UARTSTA&1;}
*/

#define putchar(d) _putch(d)
#include "printf.c"

const static char *menutxt="\n"
"\n\n"
"888                8888888b.  888     888\n"         
"888                888   Y88b 888     888\n"        
"888                888    888 888     888\n"       
"888        8888b.  888   d88P Y88b   d88P 8888b.\n"  
"888           '88b 8888888P'   Y88b d88P     '88b\n"
"888       .d888888 888 T88b     Y88o88P  .d888888\n" 
"888888888 888  888 888  T88b     Y888P   888  888\n"
"888888888 'Y888888 888   T88b     Y8P    'Y888888\n"
"\nIts Alive :-)\n"
"\n";             

uint8_t udat[32];
volatile uint8_t rdix,wrix;

uint8_t _getch()
{
	uint8_t d;
	while(rdix==wrix);
	d=udat[rdix++];
	rdix&=31;
	return d;
}
                     
uint8_t haschar() {return wrix-rdix;}

uint32_t __attribute__((naked)) getMEPC()
{
	asm volatile(
	//"	csrrw	a0,0x341,zero	\n"
	//"	csrrw	zero,0x341,a0	\n"
	"	.word	0x34101573 		\n"
	"	.word   0x34151073		\n"
	"	ret						\n"
	);
}

void __attribute__((interrupt ("machine"))) irq1_handler()
{
	_printf("\nTRAP at 0x%x\n",getMEPC());
}


void __attribute__((interrupt ("machine"))) irq2_handler()
{
	udat[wrix++]=UARTDAT;
	wrix&=31;
}

void  __attribute__((interrupt ("machine"))) irq3_handler(){
	static uint8_t a=32;
	UARTDAT=a;
	if (++a>=128) a=32;
}


// --------------------------------------------------------

uint32_t spixfer (uint32_t d)
{
	SPIDAT=d;
	while(SPISTA&1);
	return SPIDAT;
}

// --------------------------------------------------------
// SPI2
// uint32_t spixfer2 (uint32_t d)
// {
// 	SPIDAT2=d;
// 	while(SPISTA2&1);
// 	return SPIDAT2;
// }

// --------------------------------------------------------

#define BAUD 115200
#define NULL ((void *)0)

uint32_t getw()
{
	uint32_t i;
	i=_getch();
	i|=_getch()<<8;
	i|=_getch()<<16;
	i|=_getch()<<24;
	return i;
}

uint8_t *_memcpy(uint8_t *pdst, uint8_t *psrc, uint32_t nb)
{
	if (nb) do {*pdst++=*psrc++; } while (--nb);
	return pdst;
}

void main()
{
	char c,buf[17];
	uint8_t *p;
	unsigned int i,j;
	int n; 
	void (*pcode)();
	uint32_t *pi;
	uint16_t *ps;
	#define FREC_SPI 1000000
	// #define DIVIDER_SPI (*(volatile uint8_t*)((CCLK/(2*FREC_SPI))-1))
	#define DIVIDER_SPI ((CCLK/(2*FREC_SPI))-1) // Scamos el divider para que la frecuencia quede a 1Mhz -> Div = (CCLK/(2*frecuencia))-1 => Div = (18Mhz/(2*1Mhz))-1 = 8
	// SCK frequency = Fclk / (2*(DIVIDER+1));
	SPICTL= (8<<8)|DIVIDER_SPI;
	SPIDAT = 0x55;
	// spixfer(0x66);
	UARTBAUD=(CCLK+BAUD/2)/BAUD -1;	
    _putch('U');
	
	//_delay_ms(100);
	c = UARTDAT;		// Clear RX garbage
	_puts("SPI prueba\n");
	int divider = DIVIDER_SPI; // Comprobamos cual es el valor del divider
	_puts("Divider es:");
	_printf("%d\n",divider);  

	// Ponemos los pines gpout a 1 y se iluminan los led
	GPOUT = 0xF;
	
	while(1)
	{
		spixfer(0x55);
		_delay_ms(100);
	}
	
	IRQVECT0=(uint32_t)irq1_handler;
	IRQVECT1=(uint32_t)irq2_handler;
	IRQVECT2=(uint32_t)irq3_handler;


	IRQEN=1;			// Enable UART RX IRQ

	asm volatile ("ecall");
	asm volatile ("ebreak");
	_puts(menutxt);
	_puts("Hola mundo3\n");
	
	while (1)
	{
			_puts("Command [123dx]> ");
			char cmd = _getch();
			if (cmd > 32 && cmd < 127)
				_putch(cmd);
			_puts("\n");

			switch (cmd)
			{
			case '1':
			    _puts(menutxt);
				break;
			case '2':
				IRQEN^=2;	// Toggle IRQ enable for UART TX
				_delay_ms(100);
				break;
			case 'x':
				_puts("Upload APP from serial port (<crtl>-F) and execute\n");
				if(getw()!=0x66567270) break;
				p=(uint8_t *)getw();
				n=getw();
				i=getw();
				if (n) {
					do { *p++=_getch(); } while(--n);
				}

				if (i>255) {
					pcode=(void (*)())i;
					pcode();
				} 
				break;
			case 'q':
				asm volatile ("jalr zero,zero");
			case 't':
				break;
			default:
				continue;
			}
	}
}
