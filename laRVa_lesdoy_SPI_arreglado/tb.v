//-------------------------------------------------------------------

//-------------------------------------------------------------------
`include "system.v"
//`include "spi.v"
`define SIMULATION
`timescale 1ns/1ps

module tb();

//-- Registros con señales de entrada
reg clk;
reg resetb;
reg rxd;
reg rxd2;
wire txd2;

wire loop;

wire sck;
wire mosi;
reg miso;

reg [31:0] din;

//-- Instanciamos 

SYSTEM sys1(	
	.clk(clk),		// Main clock input 25MHz
	.reset(~resetb),
	.rxd2(loop),
	.txd2(loop),
	.rxd(rxd),
	.sck(sck),		// SPI
	.mosi(mosi),
	.miso(miso)
);

SPI_master spi0(
	.din(din)
);

// Reloj periódico
always #5 clk=~clk;

//-- Proceso al inicio
initial begin
	//-- Fichero donde almacenar los resultados
	$dumpfile("tb.vcd");	
	$dumpvars(0, tb);

	resetb = 0; clk=0; rxd=1;
	
	#77		resetb=1;
	// #10000  rxd=0;	//START
	
	// // Se envia el 1 -> Bit significativo primero
	// #1560   rxd=1;
	// #1560   rxd=0;
	// #1560   rxd=0;
	// #1560   rxd=0;

	// // Se envia el 4 -> Bit significativo primero
	// #1560   rxd=0;
	// #1560   rxd=0;
	// #1560   rxd=1;
	// #1560   rxd=0;

	// #1560   rxd=1;	//STOP

	#100 din = 55;
	
	#898  miso=1;
	#1560  miso=0;
	#1560  miso=1;
	#1560  miso=0;

	#1560  miso=0;
	#1560  miso=1;
	#1560  miso=0;
	#1560  miso=1;

	# 300000 $display("FIN de la simulacion");
	//# 300000 $finish;
	# 300000 $finish;
end



endmodule


