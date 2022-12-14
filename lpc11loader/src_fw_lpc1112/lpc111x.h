// lpc111x.h
// Modiffied by J. Arias:
// - PIO Mask registers added as arrays
// - PIO3 added
// - I/O Registers defined as arrays. This helps gcc to generate a better code
//
// LPC low level all-in-one header file for lpc111x devices
// principally targeted at lpc1114fn28 (DIP28 package)
// Written by Frank Duignan
// Latest version available at http://eleceng.dit.ie/frank/arm/cortex/lpc111x.h
// Derived from UM10398 user manual from NXP semiconductors
// Naming convention: Register names are as described in UM10398
// No claims are made for the suitability, accuracy or otherwise of this file
// for any application
// Define some bitmasks
#define BIT0 (1 << 0)
#define BIT1 (1 << 1)
#define BIT2 (1 << 2)
#define BIT3 (1 << 3)
#define BIT4 (1 << 4)
#define BIT5 (1 << 5)
#define BIT6 (1 << 6)
#define BIT7 (1 << 7)
#define BIT8 (1 << 8)
#define BIT9 (1 << 9)
#define BIT10 (1 << 10)
#define BIT11 (1 << 11)
#define BIT12 (1 << 12)
#define BIT13 (1 << 13)
#define BIT14 (1 << 14)
#define BIT15 (1 << 15)
#define BIT16 (1 << 16)
#define BIT17 (1 << 17)
#define BIT18 (1 << 18)
#define BIT19 (1 << 19)
#define BIT20 (1 << 20)
#define BIT21 (1 << 21)
#define BIT22 (1 << 22)
#define BIT23 (1 << 23)
#define BIT24 (1 << 24)
#define BIT25 (1 << 25)
#define BIT26 (1 << 26)
#define BIT27 (1 << 27)
#define BIT28 (1 << 28)
#define BIT29 (1 << 29)
#define BIT30 (1 << 30)
#define BIT31 (1 << 31)

// Macros to reduce typing later on
#define  REGISTER_32(ADDRESS) (*((volatile unsigned int *)(ADDRESS)))
#define  REGISTER_16(ADDRESS) (*((volatile unsigned int *)(ADDRESS)))
// Macros to enable/disable global interrupts
#define enable_interrupts() asm(" cpsie i ")
#define disable_interrupts() asm(" cpsid i ")

// APB Peripherals
#define I2C_BASE 		0x40000000
#define WDT_BASE 		0x40004000
#define UART_BASE 		0x40008000
#define TMR16B0_BASE	0x4000c000
#define TMR16B1_BASE	0x40010000
#define TMR32B0_BASE	0x40014000
#define TMR32B1_BASE	0x40018000
#define ADC_BASE		0x4001c000
#define PMU_BASE		0x40038000
#define FLASH_CTRL_BASE	0x4003c000
#define SSP0_BASE		0x40040000
#define IOCONFIG_BASE 	0x40044000
#define SYSCON_BASE 	0x40048000
#define C_CAN_BASE		0x40050000
#define SSP1_BASE		0x40058000

// I2C
asm (".set I2CBAS, 0x40000000");
extern volatile unsigned int I2CBAS[];
#define I2C0CONSET		(I2CBAS[0])		//REGISTER_32(I2C_BASE + 0x000)
#define I2C0STAT		(I2CBAS[1])		//REGISTER_32(I2C_BASE + 0x004)
#define I2C0DAT			(I2CBAS[2])		//REGISTER_32(I2C_BASE + 0x008)
#define I2C0ADR0		(I2CBAS[3])		//REGISTER_32(I2C_BASE + 0x00c)
#define I2C0SCLH		(I2CBAS[4])		//REGISTER_32(I2C_BASE + 0x010)
#define I2C0SCLL		(I2CBAS[5])		//REGISTER_32(I2C_BASE + 0x014)
#define I2C0CONCLR		(I2CBAS[6])		//REGISTER_32(I2C_BASE + 0x018)
#define I2C0COMMCTRL	(I2CBAS[7])		//REGISTER_32(I2C_BASE + 0x01c)
#define I2C0ADR1		(I2CBAS[8])		//REGISTER_32(I2C_BASE + 0x020)
#define I2C0ADR2		(I2CBAS[9])		//REGISTER_32(I2C_BASE + 0x024)
#define I2C0ADR3		(I2CBAS[10])	//REGISTER_32(I2C_BASE + 0x028)
#define I2C0DATA_BUFFER	(I2CBAS[11])	//REGISTER_32(I2C_BASE + 0x02c)
#define I2C0MASK0		(I2CBAS[12])	//REGISTER_32(I2C_BASE + 0x030)
#define I2C0MASK1		(I2CBAS[13])	//REGISTER_32(I2C_BASE + 0x034)
#define I2C0MASK2		(I2CBAS[14])	//REGISTER_32(I2C_BASE + 0x038)
#define I2C0MASK3		(I2CBAS[15])	//REGISTER_32(I2C_BASE + 0x03c)

// WDT
asm (".set WDTBAS, 0x40004000");
extern volatile unsigned int WDTBAS[];
#define WDMOD			(WDTBAS[0])	//REGISTER_32(WDT_BASE + 0x000)
#define WDTC			(WDTBAS[1])	//REGISTER_32(WDT_BASE + 0x004)
#define WDFEED			(WDTBAS[2])	//REGISTER_32(WDT_BASE + 0x008)
#define WDTV			(WDTBAS[3])	//REGISTER_32(WDT_BASE + 0x00c)
#define WDWARNINT		(WDTBAS[5])	//REGISTER_32(WDT_BASE + 0x014)
#define WDWINDOW		(WDTBAS[6])	//REGISTER_32(WDT_BASE + 0x018)

// UART
// registers appear to share addresses here.  Depending 
// on the state of DLAB different registers are active.
// Also some registers are read-only, some write-only

asm (".set U0BAS,0x40008000");
extern volatile unsigned int U0BAS[];

#define U0RBR			(U0BAS[0])
#define U0THR			(U0BAS[0])
#define U0DLL			(U0BAS[0])
#define U0DLM			(U0BAS[1])
#define U0IER			(U0BAS[1])
#define U0IIR			(U0BAS[2])
#define U0FCR			(U0BAS[2])
#define U0LCR			(U0BAS[3])
#define U0MCR			(U0BAS[4])
#define U0LSR			(U0BAS[5])
#define U0MSR			(U0BAS[6])
#define U0SCR			(U0BAS[7])
#define U0ACR			(U0BAS[8])
#define U0FDR			(U0BAS[10])
#define U0TER			(U0BAS[12])
#define U0RS485CTRL		(U0BAS[19])
#define U0RS485ADRMATCH		(U0BAS[20])
#define U0RS485DLY		(U0BAS[21])
 
// TMR16B0
asm (".set TMR16B0BAS,0x4000c000");
extern volatile unsigned int TMR16B0BAS[];
#define TMR16B0IR		(TMR16B0BAS[0])
#define TMR16B0TCR		(TMR16B0BAS[1])
#define TMR16B0TC		(TMR16B0BAS[2])
#define TMR16B0PR		(TMR16B0BAS[3])
#define TMR16B0PC		(TMR16B0BAS[4])
#define TMR16B0MCR		(TMR16B0BAS[5])
#define TMR16B0MR0		(TMR16B0BAS[6])
#define TMR16B0MR1		(TMR16B0BAS[7])
#define TMR16B0MR2		(TMR16B0BAS[8])
#define TMR16B0MR3		(TMR16B0BAS[9])
#define TMR16B0CCR		(TMR16B0BAS[10])
#define TMR16B0CR0		(TMR16B0BAS[11])
#define TMR16B0CR1		(TMR16B0BAS[12])
#define TMR16B0EMR		(TMR16B0BAS[15])
#define TMR16B0CTCR		(TMR16B0BAS[28])
#define TMR16B0PWMC		(TMR16B0BAS[29])

// TMR16B1
asm (".set TMR16B1BAS,0x40010000");
extern volatile unsigned int TMR16B1BAS[];
#define TMR16B1IR		(TMR16B1BAS[0])		//REGISTER_32(TMR16B1_BASE + 0x000)
#define TMR16B1TCR		(TMR16B1BAS[1])		//REGISTER_32(TMR16B1_BASE + 0x004)
#define TMR16B1TC		(TMR16B1BAS[2])		//REGISTER_32(TMR16B1_BASE + 0x008)
#define TMR16B1PR		(TMR16B1BAS[3])		//REGISTER_32(TMR16B1_BASE + 0x00c)
#define TMR16B1PC		(TMR16B1BAS[4])		//REGISTER_32(TMR16B1_BASE + 0x010)
#define TMR16B1MCR		(TMR16B1BAS[5])		//REGISTER_32(TMR16B1_BASE + 0x014)
#define TMR16B1MR0		(TMR16B1BAS[6])		//REGISTER_32(TMR16B1_BASE + 0x018)
#define TMR16B1MR1		(TMR16B1BAS[7])		//REGISTER_32(TMR16B1_BASE + 0x01c)
#define TMR16B1MR2		(TMR16B1BAS[8])		//REGISTER_32(TMR16B1_BASE + 0x020)
#define TMR16B1MR3		(TMR16B1BAS[9])		//REGISTER_32(TMR16B1_BASE + 0x024)
#define TMR16B1CCR		(TMR16B1BAS[10])	//REGISTER_32(TMR16B1_BASE + 0x028)
#define TMR16B1CR0		(TMR16B1BAS[11])	//REGISTER_32(TMR16B1_BASE + 0x02c)
#define TMR16B1CR1		(TMR16B1BAS[12])	//REGISTER_32(TMR16B1_BASE + 0x030)
#define TMR16B1EMR		(TMR16B1BAS[15])	//REGISTER_32(TMR16B1_BASE + 0x03c)
#define TMR16B1CTCR		(TMR16B1BAS[28])	//REGISTER_32(TMR16B1_BASE + 0x070)
#define TMR16B1PWMC		(TMR16B1BAS[29])	//REGISTER_32(TMR16B1_BASE + 0x074)

// TMR32B0
asm (".set TMR32B0BAS, 0x40014000");
extern volatile unsigned int TMR32B0BAS[];
#define TMR32B0IR		(TMR32B0BAS[0])
#define TMR32B0TCR		(TMR32B0BAS[1])
#define TMR32B0TC		(TMR32B0BAS[2])
#define TMR32B0PR		(TMR32B0BAS[3])
#define TMR32B0PC		(TMR32B0BAS[4])
#define TMR32B0MCR		(TMR32B0BAS[5])
#define TMR32B0MR0		(TMR32B0BAS[6])
#define TMR32B0MR1		(TMR32B0BAS[7])
#define TMR32B0MR2		(TMR32B0BAS[8])
#define TMR32B0MR3		(TMR32B0BAS[9])
#define TMR32B0CCR		(TMR32B0BAS[10])
#define TMR32B0CR0		(TMR32B0BAS[11])
#define TMR32B0CR1		(TMR32B0BAS[12])
#define TMR32B0EMR		(TMR32B0BAS[15])
#define TMR32B0CTCR		(TMR32B0BAS[28])
#define TMR32B0PWMC		(TMR32B0BAS[29])

// TMR32B1
asm (".set TMR32B1BAS, 0x40018000");
extern volatile unsigned int TMR32B1BAS[];
#define TMR32B1IR		(TMR32B1BAS[0])		//REGISTER_32(TMR32B1_BASE + 0x000)
#define TMR32B1TCR		(TMR32B1BAS[1])		//REGISTER_32(TMR32B1_BASE + 0x004)
#define TMR32B1TC		(TMR32B1BAS[2])		//REGISTER_32(TMR32B1_BASE + 0x008)
#define TMR32B1PR		(TMR32B1BAS[3])		//REGISTER_32(TMR32B1_BASE + 0x00c)
#define TMR32B1PC		(TMR32B1BAS[4])		//REGISTER_32(TMR32B1_BASE + 0x010)
#define TMR32B1MCR		(TMR32B1BAS[5])		//REGISTER_32(TMR32B1_BASE + 0x014)
#define TMR32B1MR0		(TMR32B1BAS[6])		//REGISTER_32(TMR32B1_BASE + 0x018)
#define TMR32B1MR1		(TMR32B1BAS[7])		//REGISTER_32(TMR32B1_BASE + 0x01c)
#define TMR32B1MR2		(TMR32B1BAS[8])		//REGISTER_32(TMR32B1_BASE + 0x020)
#define TMR32B1MR3		(TMR32B1BAS[9])		//REGISTER_32(TMR32B1_BASE + 0x024)
#define TMR32B1CCR		(TMR32B1BAS[10])	//REGISTER_32(TMR32B1_BASE + 0x028)
#define TMR32B1CR0		(TMR32B1BAS[11])	//REGISTER_32(TMR32B1_BASE + 0x02c)
#define TMR32B1CR1		(TMR32B1BAS[12])	//REGISTER_32(TMR32B1_BASE + 0x030)
#define TMR32B1EMR		(TMR32B1BAS[15])	//REGISTER_32(TMR32B1_BASE + 0x03c)
#define TMR32B1CTCR		(TMR32B1BAS[28])	//REGISTER_32(TMR32B1_BASE + 0x070)
#define TMR32B1PWMC		(TMR32B1BAS[29])	//REGISTER_32(TMR32B1_BASE + 0x074)

// ADC 
asm (".set ADCBAS, 0x4001c000");
extern volatile unsigned int ADCBAS[];
#define AD0CR		(ADCBAS[0])		//REGISTER_32(ADC_BASE + 0x000)
#define AD0GDR		(ADCBAS[1])		//REGISTER_32(ADC_BASE + 0x004)
#define AD0INTEN	(ADCBAS[3])		//REGISTER_32(ADC_BASE + 0x00c)
#define AD0DR0		(ADCBAS[4])		//REGISTER_32(ADC_BASE + 0x010)
#define AD0DR1		(ADCBAS[5])		//REGISTER_32(ADC_BASE + 0x014)
#define AD0DR2		(ADCBAS[6])		//REGISTER_32(ADC_BASE + 0x018)
#define AD0DR3		(ADCBAS[7])		//REGISTER_32(ADC_BASE + 0x01c)
#define AD0DR4		(ADCBAS[8])		//REGISTER_32(ADC_BASE + 0x020)
#define AD0DR5		(ADCBAS[9])		//REGISTER_32(ADC_BASE + 0x024)
#define AD0DR6		(ADCBAS[10])	//REGISTER_32(ADC_BASE + 0x028)
#define AD0DR7		(ADCBAS[11])	//REGISTER_32(ADC_BASE + 0x02c)
#define AD0STAT		(ADCBAS[12])	//REGISTER_32(ADC_BASE + 0x030)

// PMU
asm (".set PMUBAS, 0x40038000");
extern volatile unsigned int PMUBAS[];
#define PCON		(PMUBAS[0])	//REGISTER_32(PMU_BASE + 0x000)
#define GPREG0		(PMUBAS[1])	//REGISTER_32(PMU_BASE + 0x004)
#define GPREG1		(PMUBAS[2])	//REGISTER_32(PMU_BASE + 0x008)
#define GPREG2		(PMUBAS[3])	//REGISTER_32(PMU_BASE + 0x00c)
#define GPREG3		(PMUBAS[4])	//REGISTER_32(PMU_BASE + 0x010)
#define GPREG4		(PMUBAS[5])	//REGISTER_32(PMU_BASE + 0x014)

// FLASH CONTROLLER
#define FLASHCFG	REGISTER_32(FLASH_CTRL_BASE + 0x010)
#define FMSSTART	REGISTER_32(FLASH_CTRL_BASE + 0x020)
#define FMSSTOP		REGISTER_32(FLASH_CTRL_BASE + 0x024)
#define FMSW0		REGISTER_32(FLASH_CTRL_BASE + 0x02c)
#define FMSW1		REGISTER_32(FLASH_CTRL_BASE + 0x030)
#define FMSW2		REGISTER_32(FLASH_CTRL_BASE + 0x034)
#define FMSW3		REGISTER_32(FLASH_CTRL_BASE + 0x038)
#define FMSTAT		REGISTER_32(FLASH_CTRL_BASE + 0xfe0)
#define FMSTATCLR	REGISTER_32(FLASH_CTRL_BASE + 0xfe8)

// IOCONFIG
asm (".set IOCONFIGBAS, 0x40044000");
extern volatile unsigned int IOCONFIGBAS[];
#define IOCON_PIO2_6		(IOCONFIGBAS[0])	//REGISTER_32(IOCONFIG_BASE + 0x000)
#define IOCON_PIO2_0		(IOCONFIGBAS[2])	//REGISTER_32(IOCONFIG_BASE + 0x008)
#define IOCON_RESET_PIO0_0	(IOCONFIGBAS[3])	//REGISTER_32(IOCONFIG_BASE + 0x00c)
#define IOCON_PIO0_1		(IOCONFIGBAS[4])	//REGISTER_32(IOCONFIG_BASE + 0x010)
#define IOCON_PIO1_8		(IOCONFIGBAS[5])	//REGISTER_32(IOCONFIG_BASE + 0x014)
#define IOCON_SSEL1_LOC		(IOCONFIGBAS[6])	//REGISTER_32(IOCONFIG_BASE + 0x018)
#define IOCON_PIO0_2		(IOCONFIGBAS[7])	//REGISTER_32(IOCONFIG_BASE + 0x01c)
#define IOCON_PIO2_7		(IOCONFIGBAS[8])	//REGISTER_32(IOCONFIG_BASE + 0x020)
#define IOCON_PIO2_8		(IOCONFIGBAS[9])	//REGISTER_32(IOCONFIG_BASE + 0x024)
#define IOCON_PIO2_1		(IOCONFIGBAS[10])	//REGISTER_32(IOCONFIG_BASE + 0x028)
#define IOCON_PIO0_3		(IOCONFIGBAS[11])	//REGISTER_32(IOCONFIG_BASE + 0x02c)
#define IOCON_PIO0_4		(IOCONFIGBAS[12])	//REGISTER_32(IOCONFIG_BASE + 0x030)
#define IOCON_PIO0_5		(IOCONFIGBAS[13])	//REGISTER_32(IOCONFIG_BASE + 0x034)
#define IOCON_PIO1_9		(IOCONFIGBAS[14])	//REGISTER_32(IOCONFIG_BASE + 0x038)
#define IOCON_PIO3_4		(IOCONFIGBAS[15])	//REGISTER_32(IOCONFIG_BASE + 0x03c)
#define IOCON_PIO2_4		(IOCONFIGBAS[16])	//REGISTER_32(IOCONFIG_BASE + 0x040)
#define IOCON_PIO2_5		(IOCONFIGBAS[17])	//REGISTER_32(IOCONFIG_BASE + 0x044)
#define IOCON_PIO3_5		(IOCONFIGBAS[18])	//REGISTER_32(IOCONFIG_BASE + 0x048)
#define IOCON_PIO0_6		(IOCONFIGBAS[19])	//REGISTER_32(IOCONFIG_BASE + 0x04c)
#define IOCON_PIO0_7		(IOCONFIGBAS[20])	//REGISTER_32(IOCONFIG_BASE + 0x050)
#define IOCON_PIO2_9		(IOCONFIGBAS[21])	//REGISTER_32(IOCONFIG_BASE + 0x054)
#define IOCON_PIO2_10		(IOCONFIGBAS[22])	//REGISTER_32(IOCONFIG_BASE + 0x058)
#define IOCON_PIO2_2		(IOCONFIGBAS[23])	//REGISTER_32(IOCONFIG_BASE + 0x05c)
#define IOCON_PIO0_8		(IOCONFIGBAS[24])	//REGISTER_32(IOCONFIG_BASE + 0x060)
#define IOCON_PIO0_9		(IOCONFIGBAS[25])	//REGISTER_32(IOCONFIG_BASE + 0x064)
#define IOCON_SWCLK_PIO0_10 (IOCONFIGBAS[26])	//REGISTER_32(IOCONFIG_BASE + 0x068)
#define IOCON_PIO1_10		(IOCONFIGBAS[27])	//REGISTER_32(IOCONFIG_BASE + 0x06c)
#define IOCON_PIO2_11		(IOCONFIGBAS[28])	//REGISTER_32(IOCONFIG_BASE + 0x070)
#define IOCON_R_PIO0_11		(IOCONFIGBAS[29])	//REGISTER_32(IOCONFIG_BASE + 0x074)
#define IOCON_R_PIO1_0		(IOCONFIGBAS[30])	//REGISTER_32(IOCONFIG_BASE + 0x078)
#define IOCON_R_PIO1_1		(IOCONFIGBAS[31])	//REGISTER_32(IOCONFIG_BASE + 0x07c)
#define IOCON_R_PIO1_2		(IOCONFIGBAS[32])	//REGISTER_32(IOCONFIG_BASE + 0x080)
#define IOCON_PIO3_0		(IOCONFIGBAS[33])	//REGISTER_32(IOCONFIG_BASE + 0x084)
#define IOCON_PIO3_1		(IOCONFIGBAS[34])	//REGISTER_32(IOCONFIG_BASE + 0x088)
#define IOCON_PIO2_3		(IOCONFIGBAS[35])	//REGISTER_32(IOCONFIG_BASE + 0x08c)
#define IOCON_SWDIO_PIO1_3	(IOCONFIGBAS[36])	//REGISTER_32(IOCONFIG_BASE + 0x090)
#define IOCON_PIO1_4		(IOCONFIGBAS[37])	//REGISTER_32(IOCONFIG_BASE + 0x094)
#define IOCON_PIO1_11		(IOCONFIGBAS[38])	//REGISTER_32(IOCONFIG_BASE + 0x098)
#define IOCON_PIO3_2		(IOCONFIGBAS[39])	//REGISTER_32(IOCONFIG_BASE + 0x09c)
#define IOCON_PIO1_5		(IOCONFIGBAS[40])	//REGISTER_32(IOCONFIG_BASE + 0x0a0)
#define IOCON_PIO1_6		(IOCONFIGBAS[41])	//REGISTER_32(IOCONFIG_BASE + 0x0a4)
#define IOCON_PIO1_7		(IOCONFIGBAS[42])	//REGISTER_32(IOCONFIG_BASE + 0x0a8)
#define IOCON_PIO3_3		(IOCONFIGBAS[43])	//REGISTER_32(IOCONFIG_BASE + 0x0ac)
#define IOCON_SCK0_LOC		(IOCONFIGBAS[44])	//REGISTER_32(IOCONFIG_BASE + 0x0b0)
#define IOCON_DSR_LOC		(IOCONFIGBAS[45])	//REGISTER_32(IOCONFIG_BASE + 0x0b4)
#define IOCON_DCD_LOC		(IOCONFIGBAS[46])	//REGISTER_32(IOCONFIG_BASE + 0x0b8)
#define IOCON_RI_LOC		(IOCONFIGBAS[47])	//REGISTER_32(IOCONFIG_BASE + 0x0bc)
#define IOCON_CT16B0_CAP0_LOC 		(IOCONFIGBAS[48])	//REGISTER_32(IOCONFIG_BASE + 0x0c0)
#define IOCON_SCK1_LOC		(IOCONFIGBAS[49])	//REGISTER_32(IOCONFIG_BASE + 0x0c4)
#define IOCON_MISO1_LOC		(IOCONFIGBAS[50])	//REGISTER_32(IOCONFIG_BASE + 0x0c8)
#define IOCON_MOSI1_LOC		(IOCONFIGBAS[51])	//REGISTER_32(IOCONFIG_BASE + 0x0cc)
#define IOCON_CT32B0_CAP0_LOC		(IOCONFIGBAS[52])	//REGISTER_32(IOCONFIG_BASE + 0x0d0)
#define IOCON_RXD_LOC		(IOCONFIGBAS[53])	//REGISTER_32(IOCONFIG_BASE + 0x0d4)

// SYSCON
asm (".set SYSCONBAS, 0x40048000");
extern volatile unsigned int SYSCONBAS[];
#define SYSMEMREMAP		(SYSCONBAS[0])		//REGISTER_32(SYSCON_BASE + 0x000)
#define PRESETCTRL		(SYSCONBAS[1])		//REGISTER_32(SYSCON_BASE + 0x004)
#define SYSPLLCTRL		(SYSCONBAS[2])		//REGISTER_32(SYSCON_BASE + 0x008)
#define SYSPLLSTAT		(SYSCONBAS[3])		//REGISTER_32(SYSCON_BASE + 0x00c)
#define SYSOSCCTRL		(SYSCONBAS[8])		//REGISTER_32(SYSCON_BASE + 0x020)
#define WDTOSCCTRL		(SYSCONBAS[9])		//REGISTER_32(SYSCON_BASE + 0x024)
#define IRCCTL			(SYSCONBAS[10])		//REGISTER_32(SYSCON_BASE + 0x028)
#define SYSRSTSTAT		(SYSCONBAS[12])		//REGISTER_32(SYSCON_BASE + 0x030)
#define SYSPLLCLKSEL	(SYSCONBAS[16])		//REGISTER_32(SYSCON_BASE + 0x040)
#define SYSPLLCLKUEN	(SYSCONBAS[17])		//REGISTER_32(SYSCON_BASE + 0x044)
#define MAINCLKSEL		(SYSCONBAS[28])		//REGISTER_32(SYSCON_BASE + 0x070)
#define MAINCLKUEN		(SYSCONBAS[29])		//REGISTER_32(SYSCON_BASE + 0x074)
#define SYSAHBCLKDIV	(SYSCONBAS[30])		//REGISTER_32(SYSCON_BASE + 0x078)
#define SYSAHBCLKCTRL	(SYSCONBAS[32])		//REGISTER_32(SYSCON_BASE + 0x080)
#define SSP0CLKDIV		(SYSCONBAS[37])		//REGISTER_32(SYSCON_BASE + 0x094)
#define UARTCLKDIV		(SYSCONBAS[38])		//REGISTER_32(SYSCON_BASE + 0x098)
#define SSP1CLKDIV		(SYSCONBAS[39])		//REGISTER_32(SYSCON_BASE + 0x09c)
#define WDTCLKSEL		(SYSCONBAS[52])		//REGISTER_32(SYSCON_BASE + 0x0d0)
#define WDTCLKUEN		(SYSCONBAS[53])		//REGISTER_32(SYSCON_BASE + 0x0d4)
#define WDTCLKDIV		(SYSCONBAS[54])		//REGISTER_32(SYSCON_BASE + 0x0d8)
#define CLKOUTCLKSEL	(SYSCONBAS[56])		//REGISTER_32(SYSCON_BASE + 0x0e0)
#define CLKOUTUEN		(SYSCONBAS[57])		//REGISTER_32(SYSCON_BASE + 0x0e4)
#define CLKOUTCLKDIV	(SYSCONBAS[58])		//REGISTER_32(SYSCON_BASE + 0x0e8)
#define PIOPORCAP0		(SYSCONBAS[64])		//REGISTER_32(SYSCON_BASE + 0x100)
#define PIOPORCAP1		(SYSCONBAS[65])		//REGISTER_32(SYSCON_BASE + 0x104)
#define BODCTRL			(SYSCONBAS[84])		//REGISTER_32(SYSCON_BASE + 0x150)
#define SYSTCKCAL		(SYSCONBAS[85])		//REGISTER_32(SYSCON_BASE + 0x154)
#define IRQLATENCY      (SYSCONBAS[92])     //REGISTER_32(SYSCON_BASE + 0x170)
#define NMISRC			(SYSCONBAS[93])		//REGISTER_32(SYSCON_BASE + 0x174)
#define STARTAPRP0		(SYSCONBAS[128])	//REGISTER_32(SYSCON_BASE + 0x200)
#define STARTERP0		(SYSCONBAS[129])	//REGISTER_32(SYSCON_BASE + 0x204)
#define STARTRSRP0CLR	(SYSCONBAS[130])	//REGISTER_32(SYSCON_BASE + 0x208)
#define STARTSRP0		(SYSCONBAS[131])	//REGISTER_32(SYSCON_BASE + 0x20c)
#define PDSLEEPCFG		(SYSCONBAS[140])	//REGISTER_32(SYSCON_BASE + 0x230)
#define PDAWAKECFG		(SYSCONBAS[141])	//REGISTER_32(SYSCON_BASE + 0x234)
#define PDRUNCFG		(SYSCONBAS[142])	//REGISTER_32(SYSCON_BASE + 0x238)
#define DEVICE_ID		(SYSCONBAS[253])	//REGISTER_32(SYSCON_BASE + 0x3f4)

// CAN
#define CANCNTL		REGISTER_32(C_CAN_BASE + 0x000)
#define CANSTAT		REGISTER_32(C_CAN_BASE + 0x004)
#define CANEC		REGISTER_32(C_CAN_BASE + 0x008)
#define CANBT		REGISTER_32(C_CAN_BASE + 0x00c)
#define CANINT		REGISTER_32(C_CAN_BASE + 0x010)
#define CANTEST		REGISTER_32(C_CAN_BASE + 0x014)
#define CANBRPE		REGISTER_32(C_CAN_BASE + 0x018)
#define CANIF1_CMDREQ	REGISTER_32(C_CAN_BASE + 0x020)
#define CANIF1_CMDMSK_W	REGISTER_32(C_CAN_BASE + 0x024)
#define CANIF1_CMDMSK_R	REGISTER_32(C_CAN_BASE + 0x024)
#define CANIF1_MSK1		REGISTER_32(C_CAN_BASE + 0x028)
#define CANIF1_MSK2		REGISTER_32(C_CAN_BASE + 0x02c)
#define CANIF1_ARB1		REGISTER_32(C_CAN_BASE + 0x030)
#define CANIF1_ARB2		REGISTER_32(C_CAN_BASE + 0x034)
#define CANIF1_MCTRL	REGISTER_32(C_CAN_BASE + 0x038)
#define CANIF1_DA1		REGISTER_32(C_CAN_BASE + 0x03c)
#define CANIF1_DA2		REGISTER_32(C_CAN_BASE + 0x040)
#define CANIF1_DB1		REGISTER_32(C_CAN_BASE + 0x044)
#define CANIF1_DB2		REGISTER_32(C_CAN_BASE + 0x048)
#define CANIF2_CMDREQ	REGISTER_32(C_CAN_BASE + 0x080)
#define CANIF2_CMDMSK_W	REGISTER_32(C_CAN_BASE + 0x084)
#define CANIF2_CMDMSK_R	REGISTER_32(C_CAN_BASE + 0x084)
#define CANIF2_MSK1		REGISTER_32(C_CAN_BASE + 0x088)
#define CANIF2_MSK2		REGISTER_32(C_CAN_BASE + 0x08c)
#define CANIF2_ARB1		REGISTER_32(C_CAN_BASE + 0x090)
#define CANIF2_ARB2		REGISTER_32(C_CAN_BASE + 0x094)
#define CANIF2_MCTRL	REGISTER_32(C_CAN_BASE + 0x098)
#define CANIF2_DA1		REGISTER_32(C_CAN_BASE + 0x09c)
#define CANIF2_DA2		REGISTER_32(C_CAN_BASE + 0x0a0)
#define CANIF2_DB1		REGISTER_32(C_CAN_BASE + 0x0a4)
#define CANIF2_DB2		REGISTER_32(C_CAN_BASE + 0x0a8)
#define CANTXREQ1		REGISTER_32(C_CAN_BASE + 0x100)
#define CANTXREQ2		REGISTER_32(C_CAN_BASE + 0x104)
#define CANND1			REGISTER_32(C_CAN_BASE + 0x120)
#define CANND2			REGISTER_32(C_CAN_BASE + 0x124)
#define CANIR1			REGISTER_32(C_CAN_BASE + 0x140)
#define CANIR2			REGISTER_32(C_CAN_BASE + 0x144)
#define CANMSGV1		REGISTER_32(C_CAN_BASE + 0x160)
#define CANNSGV2		REGISTER_32(C_CAN_BASE + 0x164)
#define CANCLKDIV		REGISTER_32(C_CAN_BASE + 0x180)


// SSP0
asm (".set SSP0BAS, 0x40040000");
extern volatile unsigned int SSP0BAS[];

#define SSP0CR0		(SSP0BAS[0])
#define SSP0CR1		(SSP0BAS[1])
#define SSP0DR		(SSP0BAS[2])
#define SSP0SR		(SSP0BAS[3])
#define SSP0CPSR	(SSP0BAS[4])
#define SSP0IMSC	(SSP0BAS[5])
#define SSP0RIS		(SSP0BAS[6])
#define SSP0MIS		(SSP0BAS[7])
#define SSP0ICR		(SSP0BAS[8])

// SSP1
asm (".set SSP1BAS, 0x40058000");
extern volatile unsigned int SSP1BAS[];

#define SSP1CR0		(SSP1BAS[0])	//REGISTER_32(SSP1_BASE + 0x000)
#define SSP1CR1		(SSP1BAS[1])	//REGISTER_32(SSP1_BASE + 0x004)
#define SSP1DR		(SSP1BAS[2])	//REGISTER_32(SSP1_BASE + 0x008)
#define SSP1SR		(SSP1BAS[3])	//REGISTER_32(SSP1_BASE + 0x00c)
#define SSP1CPSR	(SSP1BAS[4])	//REGISTER_32(SSP1_BASE + 0x010)
#define SSP1IMSC	(SSP1BAS[5])	//REGISTER_32(SSP1_BASE + 0x014)
#define SSP1RIS		(SSP1BAS[6])	//REGISTER_32(SSP1_BASE + 0x018)
#define SSP1MIS		(SSP1BAS[7])	//REGISTER_32(SSP1_BASE + 0x01c)
#define SSP1ICR		(SSP1BAS[8])	//REGISTER_32(SSP1_BASE + 0x020)

// AHB Peripherals
#define GPIO0_BASE		0x50000000
#define GPIO1_BASE		0x50010000
#define GPIO2_BASE		0x50020000
#define GPIO3_BASE		0x50030000

asm(".set GPIO0MASK, 0x50000000");
extern volatile unsigned int GPIO0MASK[];
asm(".set GPIO0BAS8, 0x50008000");
extern volatile unsigned int GPIO0BAS8[];
#define GPIO0DATA		REGISTER_32(GPIO0_BASE + 0x3ffc)
#define GPIO0DIR		(GPIO0BAS8[0])	//REGISTER_32(GPIO0_BASE + 0x8000)
#define GPIO0IS			(GPIO0BAS8[1])	//REGISTER_32(GPIO0_BASE + 0x8004)
#define GPIO0IBE		(GPIO0BAS8[2])	//REGISTER_32(GPIO0_BASE + 0x8008)
#define GPIO0IEV		(GPIO0BAS8[3])	//REGISTER_32(GPIO0_BASE + 0x800c)
#define GPIO0IE			(GPIO0BAS8[4])	//REGISTER_32(GPIO0_BASE + 0x8010)
#define GPIO0RIS		(GPIO0BAS8[5])	//REGISTER_32(GPIO0_BASE + 0x8014)
#define GPIO0MIS		(GPIO0BAS8[6])	//REGISTER_32(GPIO0_BASE + 0x8018)
#define GPIO0IC			(GPIO0BAS8[7])	//REGISTER_32(GPIO0_BASE + 0x801c)

asm(".set GPIO1MASK, 0x50010000");
extern volatile unsigned int GPIO1MASK[];
asm(".set GPIO1BAS8, 0x50018000");
extern volatile unsigned int GPIO1BAS8[];
#define GPIO1DATA		REGISTER_32(GPIO1_BASE + 0x3ffc)
#define GPIO1DIR		(GPIO1BAS8[0])	//REGISTER_32(GPIO1_BASE + 0x8000)
#define GPIO1IS			(GPIO1BAS8[1])	//REGISTER_32(GPIO1_BASE + 0x8004)
#define GPIO1IBE		(GPIO1BAS8[2])	//REGISTER_32(GPIO1_BASE + 0x8008)
#define GPIO1IEV		(GPIO1BAS8[3])	//REGISTER_32(GPIO1_BASE + 0x800c)
#define GPIO1IE			(GPIO1BAS8[4])	//REGISTER_32(GPIO1_BASE + 0x8010)
#define GPIO1RIS		(GPIO1BAS8[5])	//REGISTER_32(GPIO1_BASE + 0x8014)
#define GPIO1MIS		(GPIO1BAS8[6])	//REGISTER_32(GPIO1_BASE + 0x8018)
#define GPIO1IC			(GPIO1BAS8[7])	//REGISTER_32(GPIO1_BASE + 0x801c)

asm(".set GPIO2MASK, 0x50020000");
extern volatile unsigned int GPIO2MASK[];
asm(".set GPIO2BAS8, 0x50028000");
extern volatile unsigned int GPIO2BAS8[];
#define GPIO2DATA		REGISTER_32(GPIO2_BASE + 0x3ffc)
#define GPIO2DIR		(GPIO2BAS8[0])	//REGISTER_32(GPIO2_BASE + 0x8000)
#define GPIO2IS			(GPIO2BAS8[1])	//REGISTER_32(GPIO2_BASE + 0x8004)
#define GPIO2IBE		(GPIO2BAS8[2])	//REGISTER_32(GPIO2_BASE + 0x8008)
#define GPIO2IEV		(GPIO2BAS8[3])	//REGISTER_32(GPIO2_BASE + 0x800c)
#define GPIO2IE			(GPIO2BAS8[4])	//REGISTER_32(GPIO2_BASE + 0x8010)
#define GPIO2RIS		(GPIO2BAS8[5])	//REGISTER_32(GPIO2_BASE + 0x8014)
#define GPIO2MIS		(GPIO2BAS8[6])	//REGISTER_32(GPIO2_BASE + 0x8018)
#define GPIO2IC			(GPIO2BAS8[7])	//REGISTER_32(GPIO2_BASE + 0x801c)

asm(".set GPIO3MASK, 0x50030000");
extern volatile unsigned int GPIO3MASK[];
asm(".set GPIO3BAS8, 0x50038000");
extern volatile unsigned int GPIO3BAS8[];
#define GPIO3DATA		REGISTER_32(GPIO3_BASE + 0x3ffc)
#define GPIO3DIR		(GPIO3BAS8[0])	//REGISTER_32(GPIO3_BASE + 0x8000)
#define GPIO3IS			(GPIO3BAS8[1])	//REGISTER_32(GPIO3_BASE + 0x8004)
#define GPIO3IBE		(GPIO3BAS8[2])	//REGISTER_32(GPIO3_BASE + 0x8008)
#define GPIO3IEV		(GPIO3BAS8[3])	//REGISTER_32(GPIO3_BASE + 0x800c)
#define GPIO3IE			(GPIO3BAS8[4])	//REGISTER_32(GPIO3_BASE + 0x8010)
#define GPIO3RIS		(GPIO3BAS8[5])	//REGISTER_32(GPIO3_BASE + 0x8014)
#define GPIO3MIS		(GPIO3BAS8[6])	//REGISTER_32(GPIO3_BASE + 0x8018)
#define GPIO3IC			(GPIO3BAS8[7])	//REGISTER_32(GPIO3_BASE + 0x801c)


// Core peripherals
#define STK_BASE	0xe000e010
#define SCB_BASE	0xe000ed00
#define NVIC_BASE 	0xe000e100
// Seems base addresses are split for some core peripherals
#define SCB_BASE2	0xe000e008
#define NVIC_BASE2	0xe000ef00 

// NVIC
asm(".set NVICBAS, 0xe000e100");
extern volatile unsigned int NVICBAS[];

#define ISER		(NVICBAS[0])	//REGISTER_32(NVIC_BASE + 0)
#define ICER		(NVICBAS[32])	//REGISTER_32(NVIC_BASE + 0x80)
#define ISPR		(NVICBAS[64])	//REGISTER_32(NVIC_BASE + 0x100)
#define ICPR		(NVICBAS[96])	//REGISTER_32(NVIC_BASE + 0x180)
#define IPR0		(NVICBAS[192])	//REGISTER_32(NVIC_BASE + 0x300)
#define IPR1		(NVICBAS[193])	//REGISTER_32(NVIC_BASE + 0x304)
#define IPR2		(NVICBAS[194])	//REGISTER_32(NVIC_BASE + 0x308)
#define IPR3		(NVICBAS[195])	//REGISTER_32(NVIC_BASE + 0x30c)
#define IPR4		(NVICBAS[196])	//REGISTER_32(NVIC_BASE + 0x310)
#define IPR5		(NVICBAS[197])	//REGISTER_32(NVIC_BASE + 0x314)
#define IPR6		(NVICBAS[198])	//REGISTER_32(NVIC_BASE + 0x318)
#define IPR7		(NVICBAS[199])	//REGISTER_32(NVIC_BASE + 0x31c)

// STK
asm (".set STKBAS, 0xe000e010");
extern volatile unsigned int STKBAS[];
#define SYST_CSR	(STKBAS[0])	//REGISTER_32(STK_BASE + 0)
#define SYST_RVR	(STKBAS[1])	//REGISTER_32(STK_BASE + 4)
#define SYST_CVR	(STKBAS[2])	//REGISTER_32(STK_BASE + 8)
#define SYST_CALIB	(STKBAS[3])	//REGISTER_32(STK_BASE + 0x0c)

// SCB_BASE
asm (".set SCBBAS, 0xe000ed00");
extern volatile unsigned int SCBBAS[];
#define CPUID		(SCBBAS[0])	//REGISTER_32(SCB_BASE + 0)
#define ICSR		(SCBBAS[1])	//REGISTER_32(SCB_BASE + 4)
#define AIRCR		(SCBBAS[3])	//REGISTER_32(SCB_BASE + 0x0c)
#define SCR			(SCBBAS[4])	//REGISTER_32(SCB_BASE + 0x10)
#define CCR			(SCBBAS[5])	//REGISTER_32(SCB_BASE + 0x14)
#define SHPR2		(SCBBAS[7])	//REGISTER_32(SCB_BASE + 0x1c)
#define SHPR3		(SCBBAS[8])	//REGISTER_32(SCB_BASE + 0x20)

