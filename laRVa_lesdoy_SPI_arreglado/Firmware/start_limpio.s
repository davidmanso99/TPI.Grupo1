# 1 "Firmware/start.S"
# 1 "<built-in>"
# 1 "<command-line>"
# 31 "<command-line>"
# 1 "/usr/include/stdc-predef.h" 1 3 4
# 32 "<command-line>" 2
# 1 "Firmware/start.S"

##############################################################################
# RESET & IRQ
##############################################################################

 .global main, irq1_handler, irq2_handler, irq3_handler

 .section .boot
reset_vec:
 j start 
 #Esta es la linea que primero se ejecuta
.section .text

######################################
### Main program
######################################

start:

# code_NUEVO

    lui a0, 0xE0000      # a0 dirección base UART. lui pone dato en bits + significativos
    addi a0, a0, 0x40     # Desplazamos la dirección base UART2: ahora añadimos el 040 en las direcciones menos significativas (040 en hexadecimal)

    #PARIDAD PAR
    addi a1, zero, +1049 #+1031 #Aunque pongamos el BAUBITS a 11 no podemos usar los 12 bits para determinar el divider, como máximo podemos poner 11bits
    #PARIDAD IMPAR
    #addi a1, zero, +1543
    #SIN PARIDAD
    #addi a1, zero, +7

    sw a1, 4(a0)            # divider = 25
    #En nuestro caso -> parconfig + divider = siendo 10 0000 0111

    addi a1, zero, +0x42    # cargo en a1 la B mayúscula
    sw a1, 0(a0)            # txd = 0x41 comienza transmisión de uart2

    #Receptor a partir de aquí
    nollega:
        lw a2, 4(a0)			# leo el dato que llega a uart2 en a2 (debe ser B)
        andi a2, a2, +1
        beqz a2, nollega		# salta 2 instrucciones atras
        lw a2, 0(a0)			# en a2 está el dato

    # code_NUEVO end

    #PARIDAD IMPAR CON 
    addi a1, zero, +2047 #2047

    #EL DIVIDER MÁXIMO (OSEA LO MAS LENTO POSIBLE) TENIENDO EN CUENTA QUE LA INSTRUCCIÓN EN ENSAMBLADOR ES LUI Y ADMITE MÁXIMO 11 BITS. ES DECIR, 111 1111 1111 => 2047, pero tarda demasiado.
    #PODRÍAMOS CAMBIAR DICHA INSTRUCCIÓN PARA CARGAR LA DIRECCIÓN BASE Y MODIFICAR EN BASE AL CÓDIGO DE SYSTEM HASTA 16 BITS ASOCIADOS A PARCONF+DIVIDER (DIVIDER, POR TANTO, TENDRÁ MÁXIMO 10 BITS)


    sw a1, 4(a0)            # divider = 511
    #En este caso -> parconfig + divider = siendo  111 1111 1111

    addi a1, zero, +0x43    # cargo en a1 la C mayúscula
    sw a1, 0(a0)            # txd = 0x41 comienza transmisión de uart2

    #Receptor a partir de aquí
    nollega2:
        lw a2, 4(a0)			# leo el dato que llega a uart2 en a2 (debe ser C)
        andi a2, a2, +1
        beqz a2, nollega2		# salta 2 instrucciones atras 
        lw a2, 0(a0)			# en a2= está el dato


    #SIN PARIDAD
    addi a1, zero, +155 

    sw a1, 4(a0)            # divider = 155
    #En este caso -> parconfig + divider = siendo 000 1001 1011

    addi a1, zero, +0x41    # cargo en a1 la A mayúscula
    sw a1, 0(a0)            # txd = 0x41 comienza transmisión de uart2

    #Receptor a partir de aquí
    nollega3:
        lw a2, 4(a0)			# leo el dato que llega a uart2 en a2 (debe ser A)
        andi a2, a2, +1
        beqz a2, nollega3		# salta 2 instrucciones atras
        lw a2, 0(a0)			# en a2 está el dato

    end:
        j end   # end, ya no ejecuta el resto del código. Solo ejecuta lo nuestro.

# code_NUEVO end

#start:

 li sp,8192

# copy data section
 la a0, _sdata
 la a1, _sdata_values
 la a2, _edata
 bge a0, a2, end_init_data
loop_init_data:
 lw a3,0(a1)
 sw a3,0(a0)
 addi a0,a0,4
 addi a1,a1,4
 blt a0, a2, loop_init_data
end_init_data:
# zero-init bss section
 la a0, _sbss
 la a1, _ebss
 bge a0, a1, end_init_bss
loop_init_bss:
 sw zero, 0(a0)
 addi a0, a0, 4
 blt a0, a1, loop_init_bss
end_init_bss:
# call main
 call main
loop:
 j loop

 .globl delay_loop
delay_loop:
 addi a0,a0,-1
 bnez a0, delay_loop
 ret
