//--------------------------------------------------------------------
// RISC-V things 
// by Inés Varona, David Manso, Jorge Ansótegui, Carmen Martín (2022)
//--------------------------------------------------------------------
/*
	Description:
	A LaRVA RISC-V system with 8KB of internal memory, and one UART
	
	Memory map:
	 
	0x00000000 to 0x00001FFF	Internal RAM (with inital contents)
	0x00002000 to 0x1FFFFFFF	the same internal RAM repeated each 8KB
	0x20000000 to 0xDFFFFFFF       xxxx
	0xE0000000 to 0xE00000FF    IO registers
	0xE0000100 to 0xFFFFFFFF    the same IO registers repeated each 256B

	IO register map (all registers accessed as 32-bit words):
	
      address  |      WRITE        |      READ
    -----------|-------------------|---------------
    0xE0000000 | UART TX data      |  UART RX data
    0xE0000004 | UART Baud Divider |  UART flags
	0xE0000008 | UART1 TX data     |  UART1 RX data 
	0xE000000C | UART1 Baud Divider|  UART1 flags 
	0xE0000010 | UART2 TX data     |  UART2 RX data 
	0xE0000014 | UART2 Baud Divider|  UART2 flags  
			   |                   | 
	0xE0000020 | SPI TX data       |  SPI RX data 
	0xE0000024 | SPI Control       |  SPI flags 
	0xE0000028 | SPI Slave Select  |  xxxx 
               |                   | 
	0xE0000040 | I2C data/control  |  I2C data/status 
	0xE0000044 | I2C divider       | 
               |                   | 
	0xE0000060 |    MAX_COUNT      |     TIMER 
	           |                   | 
	0xE0000080 |      GPOUT        |     GPOUT 
	0xE0000084 |      GPOUT        |      GPIN   
	           |                   |	
	0xE00000C0 | Interrupt Enable  |  Interrupt enable 
    0xE00000E0 | IRQ vector 0 Trap | 
    0xE00000E4 | IRQ vector 1 RX   | 
    0xE00000E8 | IRQ vector 2 TX   | 
    0xE00000EC | IRQ vector 3 Timer| 
    0xE00000F0 | IRQ vector 4 RX1  | 
    0xE00000F4 | IRQ vector 5 TX1  | 
    0xE00000F8 | IRQ vector 6 RX2  | 
    0xE00000FC | IRQ vector 7 TX2  | 
    ------ 

    UART Baud Divider: Baud = Fcclk / (DIVIDER+1) , with DIVIDER >=7
    
    UART FLAGS:    bits 31-5  bit 4  bit 3 bit 2 bit 1 bit 0
                     xxxx      OVE    FE    TEND  THRE   DV
        DV:   Data Valid (RX complete if 1. Cleared reading data register)
        THRE: TX Holding register empty (ready to write to data register if 1)
        TEND: TX end (holding reg and shift reg both empty if 1)
        FE:   Frame Error (Stop bit received as 0 if FE=1)
        OVE:  Overrun Error (Character received when DV was still 1)
        (DV and THRE assert interrupt channels #4 and #5 when 1)

	------ 

    SPI Control:   bits 31-14  bits 13-8  bits 7-0 
                      xxxx        DLEN     DIVIDER 
        DLEN:    Data lenght (8 to 32 bits) 
        DIVIDER: SCK frequency = Fclk / (2*(DIVIDER+1)) 
          
    SPI Flags:     bits 31-1  bit 0 
                      xxxx     BUSY 
        BUSY:  SPI exchanging data when 1 
 
    SPI Slave Select: bits 31-2  bit 1   bit 0 
                         xxxx     ss1     ss0 
        ss0 : Selects the SPI slave 0 when 0 (active low) 
        ss1 : Selects the SPI slave 1 when 0 (active low) 
    ------ 
 
    I2C Data/Control: bit 10  bit 9  bit 8  bits 7-0 
                       STOP   START   ACK     DATA 
        STOP:  Send Stop sequence 
        START: Send Start sequence 
        ACK:   ACK bit. Must be 1 on writes and 0 on reads except last one 
        DATA:  Data to write (Must be 0xFF on reads) 
          - ACK and DATA are ignored if START or STOP are one 
          - Do not set START and STOP simultaneously 
          - Repeated START is not supported 
          - Writing to this register sets the BUSY flag until the start, 
            stop, or data, is sent 
         
    I2C Data/Status:          bit 9  bit 8  bits 7-0 
                               BUSY   ACK     DATA 
        BUSY:  Controller busy if 1.  
        ACK:   Received ACK bit (for writes) 
        DATA:  Received data (for reads) 
         
    I2C Divider: bits 6-0 
        SCL frequency = Fclk /(4*(DIVIDER+1)) 
    ------ 
    MAX_COUNT: Holds the maximum value of the timer counter. When the timer 
        reaches this value gets reset and request an interrupt if enabled. 
        Writes to MAX_COUNT also resets the timer and its interrupt flag. 
 
    TIMER: the current value of the timer (incremented each clock cycle) 
        Reads also clear the interrupt flag. 
    ------ 
    GPOUT: General purpose outputs. 
 
    GPIN: General purpose inputs. 
    ------ 
    Interrupt enable:  
        bit 0: Not used 
        bit 1: Enable UART0 RX interrupt if 1 
        bit 2: Enable UART0 TX interrupt if 1 
        bit 3: Enable TIMER    interrupt if 1 
        bit 4: Enable UART1 RX interrupt if 1 
        bit 5: Enable UART1 TX interrupt if 1 
        bit 6: Enable UART2 RX interrupt if 1 
        bit 7: Enable UART2 TX interrupt if 1 
 

    Interrupt Vectors: Hold the addresses of the corresponding interrupt 
		service routines.
*/

`include "laRVa.v"
`include "uart.v"
`include "spi.v"

////////////////////////////////////
//      Changed_Grupo1
////////////////////////////////////
`include "uart2.v"
`define SIMULATION
////////////////////////////////////
//      END_Changed_Grupo1
////////////////////////////////////

module SYSTEM (
	input clk,		// Main clock input 25MHz
	input reset,	// Global reset (active high)

	input	rxd,	// UART
	output 	txd,

	input	rxd2,	// UART2
	output 	txd2,
	
	output sck,		// SPI
	output mosi,
	input  miso,	
	
	// output sck2,	// SPI2
	// output mosi2,
	// input  miso2,	

	output[31:0] gpout,	//gpout
	
	output fssb,	// Flash CS
	output sssb		// SD card CS

);

wire		cclk;	// CPU clock
assign	cclk=clk;

///////////////////////////////////////////////////////
////////////////////////// CPU ////////////////////////
///////////////////////////////////////////////////////   

wire [31:0]	ca;		// CPU Address
wire [31:0]	cdo;	// CPU Data Output
wire [3:0]	mwe;	// Memory Write Enable (4 signals, one per byte lane)
wire irq;
wire [31:2]ivector;	// Where to jump on IRQ
wire trap;			// Trap irq (to IRQ vector generator)

laRVa cpu (
		.clk     (cclk ),
		.reset   (reset),
		.addr    (ca[31:2] ),
		.wdata   (cdo  ),
		.wstrb   (mwe  ),
		.rdata   (cdi  ),
		.irq     (irq  ),
		.ivector (ivector),
		.trap    (trap)
	);

 
///////////////////////////////////////////////////////
///// Memory mapping
wire iramcs;
wire iocs;
// Internal RAM selected in lower 512MB (0-0x1FFFFFFF)
assign iramcs = (ca[31:29]==3'b000);
// IO selected in last 512MB (0xE0000000-0xFFFFFFFF)
assign iocs   = (ca[31:29]==3'b111);


// Input bus mux
reg [31:0]cdi;	// Not a register
always@*
 casex ({iocs,iramcs})
        2'b01: cdi<=mdo; 
        2'b10: cdi<=iodo;
        default: cdi<=32'hxxxxxxxx;
 endcase

///////////////////////////////////////////////////////
//////////////////// internal memory //////////////////
///////////////////////////////////////////////////////
wire [31:0]	mdo;	// Output data
ram32	 ram0 ( .clk(~cclk), .re(iramcs), .wrlanes(iramcs?mwe:4'b0000),
			.addr(ca[12:2]), .data_read(mdo), .data_write(cdo));

//////////////////////////////////////////////////
////////////////// Peripherals ///////////////////
//////////////////////////////////////////////////
reg [31:0]tcount=0;
always @(posedge clk) tcount<=tcount+1;

wire uartcs;	// UART	at offset 0x00
wire uart2cs;	// UART2 at offset 0x00
wire spics;		
wire spics2;	
wire irqcs;		// IRQEN at offset 0xE0
				//		 ...
				// other at offset 0xE0
assign uartcs = iocs&(ca[7:5]==3'b000);

assign spics  = iocs&(ca[7:5]==3'b001);	 

assign irqcs  = iocs&(ca[7:5]==3'b111);
assign gpoutcs  = iocs&(ca[7:5]==3'b100);	//0xE00000 80
// Peripheral output bus mux
reg [31:0]iodo;	// Not a register
wire [31:0] spidat; 
always@*
 casex (ca[7:2])
	6'b000xx0: iodo<={24'h0,uart_do};
	6'b000xx1: iodo<={27'h0,ove,fe,tend,thre,dv};
	
	6'b010xx0: iodo<={24'h0,uart_do2};
	6'b010xx1: iodo<={27'h0,ove2,fe2,tend2,thre2,dv2};
	
	6'b001000: iodo<= spidat;	//ASIGNAMOS LA DIRECCION 0XE000028
	
	6'b100xxx: iodo<= gpout;
	6'b011xxx: iodo<=tcount;
	6'b111xxx: iodo<={30'h0,irqen};
	default: iodo<=32'hxxxxxxxx;
 endcase

//Necesito registro Reg gpout
reg [31:0] gpout= 0;
always @(posedge clk)
	begin
		if (gpoutcs)
			begin
				gpout <= cdo;
			end
	end

/////////////////////////////
// UART

wire tend,thre,dv,fe,ove; // Flags
wire [7:0] uart_do;	// RX output data
wire uwrtx;			// UART TX write
wire urd;			// UART RX read (for flag clearing)
wire uwrbaud;		// UART BGR write
// Register mapping
// Offset 0: write: TX Holding reg
// Offset 0: read strobe: Clear DV, OVE (also reads RX data buffer)
// Offset 1: write: BAUD divider
assign uwrtx   = uartcs & (~ca[2]) & mwe[0];
assign uwrbaud = uartcs & ( ca[2]) & mwe[0] & mwe[1];
assign urd     = uartcs & (~ca[2]) & (mwe==4'b0000); // Clear DV, OVE flgas

UART_CORE #(.BAUDBITS(12)) uart0 ( .clk(cclk), .txd(txd), .rxd(rxd), 
	.d(cdo[15:0]), .wrtx(uwrtx), .wrbaud(uwrbaud),. rd(urd), .q(uart_do),
	.dv(dv), .fe(fe), .ove(ove), .tend(tend), .thre(thre) );


wire spiwr;			// SPI wr write
wire busy;
reg [13:0] spictl;

assign spiwr = spics  & (~ca[2]); // Necesitamos crear otra señal de control/selección arriba no solo usando los bits ca[7:5] también los menos significativos
always @(posedge clk) // Quitar esto: assign spiwr = spics; 
	begin
		if (spics) // Y poner más condiciones aquí usando los bits menos significativos también
			begin
				spictl <= cdo[13:0];
			end
	end
 
SPI_master spi0 (.clk(cclk), .miso(miso),.wr(spiwr), .din(cdo), 
				.divider(spictl[7:0]), .bits(spictl[13:8]), .sck(sck), .mosi(mosi),
				.busy(busy), .dout(spidat) );
////////////////////////////////////
//      Changed_Grupo1
////////////////////////////////////

///////////////////////////// 
// UART2

wire tend2,thre2,dv2,fe2,ove2; // Flags
wire [7:0] uart_do2;	// RX output data
wire uwrtx2;			// UART TX write
wire urd2;			// UART RX read (for flag clearing)
wire uwrbaud2;		// UART BGR write
// Register mapping
// Offset 0: write: TX Holding reg
// Offset 0: read strobe: Clear DV, OVE (also reads RX data buffer)
// Offset 1: write: BAUD divider

// Tenemos que cambiar los bits de selección de mwe para cambiar la escritura de esta 2º uart.
// Antes era para la UART0: 0xE0000000
// Ahora para la UART2 : 0xE0000040

assign uwrtx2   = uart2cs & (~ca[2]) & mwe[0]; // mwe[0] esto es el bit de selección de escritura, con este a cero se escribe de tamaño bit
assign uwrbaud2 = uart2cs & ( ca[2]) & mwe[0] & mwe[1]; // y con esta selección de mwe es tamaño halfword
assign urd2     = uart2cs & (~ca[2]) & (mwe==4'b0000); // Clear DV, OVE flgas y en este caso estan todos a cero para borrar los flags, con rd activado

UART2_CORE #(.BAUDBITS(12)) uart2 ( .clk(cclk), .txd(txd2), .rxd(rxd2), 
	.d(cdo[15:0]), .wrtx(uwrtx2), .wrbaud(uwrbaud2),. rd(urd2), .q(uart_do2),
	.dv(dv2), .fe(fe2), .ove(ove2), .tend(tend2), .thre(thre2)); 

////////////////////////////////////
//      END_Changed_Grupo1
////////////////////////////////////


//////////////////////////////////////////
//    Interrupt control

// IRQ enable reg
reg [1:0]irqen=0;
always @(posedge cclk or posedge reset) begin
	if (reset) irqen<=0; else
	if (irqcs & (~ca[4]) &mwe[0]) irqen<=cdo[1:0];
end

// IRQ vectors
reg [31:2]irqvect[0:3];
always @(posedge cclk) if (irqcs & ca[4] & (mwe==4'b1111)) irqvect[ca[3:2]]<=cdo[31:2];

// Enabled IRQs
wire [1:0]irqpen={irqen[1]&thre, irqen[0]&dv};	// pending IRQs

// Priority encoder
wire [1:0]vecn = trap      ? 2'b00 : (	// ECALL, EBREAK: highest priority
				 irqpen[0] ? 2'b01 : (	// UART RX
				 irqpen[1] ? 2'b10 : 	// UART TX
				 			 2'bxx ));	
assign ivector = irqvect[vecn];
assign irq = (irqpen!=0)|trap;

endmodule	// System




//////////////////////////////////////////////////////////////////////////////
//----------------------------------------------------------------------------
//-- 32-bit RAM Memory with independent byte-write lanes
//----------------------------------------------------------------------------
//////////////////////////////////////////////////////////////////////////////

module ram32
 (	input	clk,
	input	re,
	input	[3:0]	wrlanes,
	input	[10:0]	addr,
	output	[31:0]	data_read,
	input	[31:0] 	data_write
 );

reg [31:0] ram_array [0:2047];
reg [31:0] data_out;
        
assign data_read = data_out;
        
always @(posedge clk) begin
	if (wrlanes[0]) ram_array[addr][ 7: 0] <= data_write[ 7: 0];
	if (wrlanes[1]) ram_array[addr][15: 8] <= data_write[15: 8];
	if (wrlanes[2]) ram_array[addr][23:16] <= data_write[23:16];
	if (wrlanes[3]) ram_array[addr][31:24] <= data_write[31:24];
end

always @(posedge clk) begin
	if (re) data_out <= ram_array[addr];
end

initial begin
`ifdef SIMULATION
	$readmemh("rom.hex", ram_array);
`else
	$readmemh("rand.hex", ram_array);
`endif
end

endmodule

