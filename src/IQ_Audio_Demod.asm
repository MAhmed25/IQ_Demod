.section L1_data_a;  // Linker places 12 kHz LUT starting at 0x11800000

 //8 value, 2's complement look up table for generating 12 khz sine waves.
.BYTE4 LUT[16] = {0x0, 0x0, 0x005a8279, 0x005a8279, 0x007fffff, 0x7fffff, 0x005a8279, 0x005a8279, 0x0, 0x0, 0xffa57d87, 0xffa57d87, 0xff800000, 0xff800000, 0xffa57d87, 0xffa57d87};
.BYTE4 IQLUT[8] = {0x0, 0x005a8279, 0x7fffff, 0x005a8279, 0x0, 0xffa57d87, 0xff800000, 0xffa57d87};
// IQ Storage buffer/array, since blackfin only operates on 32 bit data words, need 4
// I.H, I.L, Q.H, Q.L where I.H stores the significant 32 bit value of the I value
// Nad I.L Stores the lower 32 bit values of the calculated I values.
.BYTE4 IQ[4] = {0x0, 0x0, 0x0, 0x0}; // Stores IQ Value, 
.BYTE4 IQFinal[4] = {0x0, 0x0, 0x0, 0x0}; // Stores IQ Value.

.BYTE4 LSync = 0x0;
.BYTE4 prevValue = 0x0;

.section program; 
.global _main; 
.align 4;  
# include <defBF706.h>

// DAC Freq is 96 kHz Max, if we want say a 10 kHz output, then set LUT size to 9.6, which is impossible, next closet is 10 size
// Giving a freq of 9.6 kHz. Although, usually want LUT sizes to be in powers of 2, so pick 8 giving 12 kHz.

_main:
// Setting up Modulo addressing for LUT
B0 = LUT;	// Base address = LUT(0) Address
I0 = B0; 	// Index address = actual pointer that moves through the LUT	
L0 = (LENGTH(LUT))*4;	// L0 is length in BYTES hence its length(LUT) * 4 because LUT contains .byte4 data
M0 = 0x4(Z); // Modify address, number of BYTES to increment I0 to step through LUT
M1 = 0x10(Z); // Offset to move to out of phase component, = 16
M2 = -16;

P0=0x1;
// Enable global and core interrupts
[REG_SEC0_GCTL]=P0;
P0=0x00010001;
[REG_SEC0_CCTL0]=P0;

// Enable EVT11 ISR which handles sec interrupts
P0=0x083F;
[IMASK]=P0;

// Store into EVT11 register the starting address of the ISR.
P0=(SEC_isr);
[EVT11]=P0; 

call codec_configure;
call sport_configure;

R0=0x5;
// Enable both interrupts, with transfer buffer first.
[REG_SEC0_SCTL31]=R0;
[REG_SEC0_SCTL29]=R0;

wait:
P0 = IQFinal;

R0 = [P0++];
R1 = [P0++];
R2 = [P0++];
R3 = [P0];
nop;
jump wait;

// The ADSPBF706 Only has 1 memory-mapped (MM) evt register (EVT11)
// To service ALL SEC based interrupts
// So have to Check what caused the interrupt then do different things.
// If multiple pending interrupts, have to make sure only the correct "interrupt" is serviced
SEC_isr:
// For nesting purposes, future proofing idk if neccessary
[--SP] = RETI;
[--SP] = R0;
[--SP] = R1;
// 1: Determine the source of interrupt
// Obtain the currently active interrupt
R0 = [REG_SEC0_CSID0];
// Check if the interrupt is from SCTL31 = HALFSPORT B DMA = RX channel.
R1 = 0x1F(Z);
// If it is then jump to RX data section
CC = R0 == R1;
if CC jump RX_Data;
jump TX_Data;

RX_Data:
// Empty buffer
R0=[REG_SPORT0_RXPRI_B];

// CSID is NW (non-writeable) but writing to it causes acknowledgement
[REG_SEC0_CSID0] = R0; // Assert interrupt

// When calculating I/Q as squaring a number is akin to left shifting by its power
// And the accumulators can combine to a 72 bit mega register
// Calculate the inphase component using a MAC on A1:0;
// Get the inphase component and prepare for out of phase component:
R1 = [I0 ++ M1]; // R1 contains the in phase value and moves I0 to out of phase point in LUT
R2 = [I0 ++ M2]; // R2 Contains the out of phase value and moves I0 back to original
R3 = [LSync];
BITTGL(R3, 0);
[LSync] = R3;
CC = BITTST(R3, 0); if CC jump MACIQ;
jump ResetMac;

MACIQ:
// Calculate INPHASE component
R6 = [prevValue];
P2 = IQ;
R5 = [P2++];
R4 = [P2];
A1 = R5, A0 = R4;
R5:4 = (A1:0 += R6 * R1) (IS);
[P2--] = R4;
[P2] = R5;
// Calculate the quadrature component
P2 = IQ + 0x08;
R5 = [P2++];
R4 = [P2];
A1 = R5, A0 = R4;
R5:4 = (A1:0 += R6 * R2) (IS);
[P2--] = R4;
[P2] = R5;
[prevValue] = R0;

ResetMac:
P0 = B0;
P1 = I0;
CC = P0 == P1; if !CC jump End_ISR; // Check whether to reset or not.

P0 = IQ;
P1 = IQFinal;
R0 = [P0++];
[P1++] = R0;
R0 = [P0++];
[P1++] = R0;
R0 = [P0++];
[P1++] = R0;
R0 = [P0];
[P1] = R0;

R0 = 0x0 (z);
A1 = A0 = 0;
[P0--] = R0;
[P0--] = R0;
[P0--] = R0;
[P0] = R0;

jump End_ISR;

TX_Data:
[REG_SEC0_CSID0] = R0; // Assert interrupt

// Refills the TX Buffer with next value in LUT.
R3 = [I0 ++ M0];
[REG_SPORT0_TXPRI_A] = R3;

jump End_ISR;

End_ISR:
// Interrupt end by writing CSID to SEC END register.
R2 = [REG_SEC0_CSID0];
[REG_SEC0_END] = R2;

R1 = [SP++];
R0 = [SP++];
RETI = [SP++];
RTI;

._main.end:

// Function codec_configure initialises the ADAU1761 codec. Refer to the control register
// descriptions, page 51 onwards of the ADAU1761 data sheet.
codec_configure:
[--SP] = RETS;
// R1 Controls master clock, enable master clock and set INFREQ	TO 256xfs
// Giving a base fs of 12.288 MHZ / 256 = 48 kHz -> this allows 96khz sampling.
// (12.288 mhz comes from data sheet where there adau1761 is fed by an oscillator).
R1=0x01(X); R0=0x4000(X); call TWI_write;
// R65-66 Digital clock controllers enable just enable all
R1=0x7f(X); R0=0x40f9(X); call TWI_write;
R1=0x03(X); R0=0x40fa(X); call TWI_write;

// R15 Controls the serial port for transfers! enable master mode,
// frame begins on rising edge, left channel first (Right justified mode)
R1=0x09(X); R0=0x4015(X); call TWI_write;

// R19 ADC Control, enable both ADC, leave everything else default
R1=0x13(X); R0=0x4019(X); call TWI_write;

// R22 Mixer 3 control register, enable look to datasheet (adau1761) to see location
R1=0x21(X); R0=0x401c(X); call TWI_write;
// R23 Mixer 3 control regsiter, gains for other inputs to mixer, disable all (default)
// R24 - R25 Same as above but right -> right etc
R1=0x41(X); R0=0x401e(X); call TWI_write;

// R35 actually enable the playback section of the circuit
R1=0x03(X); R0=0x4029(X); call TWI_write;
// R36-38 DAC Controllers turn both DACs on, no change in volume
R1=0x03(X); R0=0x402a(X); call TWI_write;

// R58-59 Serial input/output control( where to send L/R Data)
R1=0x01(X); R0=0x40f2(X); call TWI_write;
R1=0x01(X); R0=0x40f3(X); call TWI_write;
// R2 Controls MIC/Jack detection, not used and default disabled
// R3 Controls record? power management, not used hence ignored
// R4 (0x400a) Controls the left Mixer 1, looking at datasheet, enable LINN-GAIN, disable (mute) rest
R1=0x0b(X); R0=0x400a(X); call TWI_write;

// R5 (0x400b) Controls left mixer record (PGA and LAUX input gains not needed, default disabled
// R6 Controls the  mixer 2 right gains for RINNG,RINPG (enable RINNG only)
R1=0x0b(X); R0=0x400c(X); call TWI_write;
// R7 Controls the record mixer2 right gains for PGA, RAUX (disable both (default))
// R8 Controls the PGA For the left path (default disabled no change)
// R9 Controls the PGA for the right path (default disabled, no change)
// R10 Controls biasing for microphones? not needed, default disabled
// R11 - R14 Control ALC, default disabled, not needed

// R26 - R27 Double mixers which lets you mix the outputs of above, DISABLE! (default)
// R28 Mixer 7 (mono) controller, disable and ignore.
// R29 - R30 LHP and RHP are connected to Jack 2. enable both and increase gain to 0db from default
R1=0xe7(X); R0=0x4023(X); call TWI_write;
R1=0xe7(X); R0=0x4024(X); call TWI_write;

// R16 serial port control part 2, 64 Bits-per-audio frame (32 bit L, 32 bit R).
// Right justified.
R1=0x02(X); R0=0x4016(X); call TWI_write;
// R17 change Sampling rate to 96 kHz from default 48 kHz
R1=0x06(X); R0=0x4017(X); call TWI_write;
// R18 Ignore TDM mode only

// R60 Default, enables all clocks instead of as GPIO
// R61-62 DSP core control (ignore) (disable)
// R63 Slew mode, ignore
// R64 Serial port sampling rate - 96 Khz
R1=0x06(X); R0=0x40f8(X); call TWI_write;
// R57 DSP Samp rate - set to 96 khz
R1=0x00(X); R0=0x40eb(X); call TWI_write;

// R20 - R21 Volumt control, leave default (no attenuation)
// R31 - R32 ROUTN/P AND LOUTN/P controls, ignore (mute) (default) (datasheet)
// R33 - R34 mono control, click suppression ignore (default)
// R39-41 serial port pull up / pull down control and drive
// R42 JACK input - ignore
// R67 Dejitter control, ignore
// R43-47 Ignore
// R48-51 Ignore
// R52-R56 Ignore




NOP;
RETS = [SP++];                            // Pop stack (only for nested calls)
RTS;
codec_configure.end:

sport_configure:

R0=0x3F0(X);
[REG_PORTC_FER]=R0;          // Set up Port C in peripheral mode
[REG_PORTC_FER_SET]=R0;      // Set up Port C in peripheral mode

// Look at the data sheets for why im doing the following steps in the particular order
// To setup interrupts properly
// 1 clear 2 key register CTL,MCTL
[REG_SPORT0_CTL_A]=R0; [REG_SPORT0_MCTL_A]=R0;
[REG_SPORT0_CTL_B]=R0; [REG_SPORT0_MCTL_B]=R0;

// Set up the DIV clock registers
R0=0x00400001; [REG_SPORT0_DIV_A]=R0;      // 64 bits per frame, clock divisor of 1
R0=0x00400001; [REG_SPORT0_DIV_B]=R0;      // 64 bits per frame (stereo), clock divisor of 1

// 1 - Right justified mode, 24 bit data word, late frame sync,
// Delay of 8 bits
R0=0x80000; [REG_SPORT0_MCTL_B]=R0;
R0=0x80000; [REG_SPORT0_MCTL_A]=R0;
// Do not enable the CTLs just yet for synchronization purposes.
R0=0x001e3973; [REG_SPORT0_CTL_B]=R0;
R0=0x021e3973; [REG_SPORT0_CTL_A]=R0;
// Preload TX Buffer
R0 = 0x0; [REG_SPORT0_TXPRI_A]=R0;
[REG_SPORT0_TXPRI_A]=R0; [REG_SPORT0_TXPRI_A]=R0;

/*
// The next 2 variables are for synchronizing such that the reciever turns on after
// 1 frame sync so that the received data is in synchronization with the transfered data
// Preload TX buffer with first value of LUT.
R3 = [I0 ++ M0];
[REG_SPORT0_TXPRI_A]=R3;
// Enable transfer buffer
R0=0x021e3973; [REG_SPORT0_CTL_A]=R0;
// Now wait until the first word has been transferred then enable receiver
InOutSynchronizer:
R0=[REG_SPORT0_CTL_A]; CC=BITTST(R0, 31); if CC jump InOutSynchronizer;
// Once sent, enable receiver
R0=0x001e3973; [REG_SPORT0_CTL_B]=R0;
*/
RTS;
sport_configure.end:

TWI_write:
//step1) Reverse low order and high order bytes, since data sent in reverse order in bytes (4 bit pairs)
//so for example, 0x4256 is sent as: 5642. (bytes!!) (little endian)
R3=R0 <<0x8; R0=R0 >>>0x8; R2=R3|R0;     

// Set duty cycle this particular TWI requires a 10 MHz internal reference clock.
R0=0x3232(X);
[REG_TWI0_CLKDIV]=R0;       

// Set prescale and enable TWI. PRESCALE = Fsclk0 / 10 MHz! Need to find Freq of sysclock first
R0=0x008c(X);
[REG_TWI0_CTL]=R0;          

// I2C Address of codec - look at the ADAU DATASHEET FOR I2C ADDRESS
R0=0x0038(X);
[REG_TWI0_MSTRADDR]=R0;     

// Address of register to set, LSB then MSB - Have to transfer address of register to set first
[REG_TWI0_TXDATA16]=R2;     

// Command to send three bytes and enable tx - First 2 Bytes dictate register, 3rd byte sets value
R0=0x00c1(X);
[REG_TWI0_MSTRCTL]=R0;     

// Register interrupt cleaning so they dont trigger + delays for cpu to idle while TWI transmits data,
// No need to complicate with interrupts as this setup should only be done once ideally.
[--SP] = RETS; call delay; RETS = [SP++]; // Delay to allow transfer to complete
[REG_TWI0_TXDATA8]=R1;                    // Data to write actual value you want to modify ADAU Register to
[--SP] = RETS; call delay; RETS = [SP++]; // Delay for transfer
R0=0x050; [REG_TWI0_ISTAT]=R0;            // Clear TXSERV interrupt
[--SP] = RETS; call delay; RETS = [SP++]; // Delay
R0=0x010; [REG_TWI0_ISTAT]=R0;            // Clear MCOMP interrupt
rts;
TWI_write.end:



delay:
P0=0x8000;
loop LC0=P0;
NOP; NOP; NOP;
loop_end;
RTS;
delay.end:
