.section L1_data_a;  // Linker places 12 kHz LUT starting at 0x11800000

.BYTE4 LUT[8] = { 0x0, 0x5a8279, 0x7fffff,0x5a8279, 0x0, 0xa57d87, 0x800000, 0xa57d87 }; //2's complement so goes from 0 -> 1 -> 0 -> -1 then repeat
//.BYTE4 LUT[16] = {0x0, 0x0, 0x5a8279, 0x5a8279, 0x7fffff, 0x7fffff, 0x5a8279, 0x5a8279, 0x0, 0x0, 0xa57d87, 0xa57d87, 0x800000, 0x800000, 0xa57d87, 0xa57d87}; //2's complement so goes from 0 -> 1 -> 0 -> -1 then repeat

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
L0 = (LENGTH(LUT))*4;	// L0 is length in BYTES!! hence its length(LUT) = 4 (4 values) * 4 because LUT is .byte4
M0 = 0x4(Z); // Modify address, number of BYTES to increment I0 by!!.

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

P0=0x5; //Enables both the interrupt and the source signal for SPORT Interrupts TX, RX
[REG_SEC0_SCTL31]=P0;
[REG_SEC0_SCTL29]=P0;

call codec_configure;
call sport_configure;

wait:
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
[REG_SEC0_CSID0] = R0; // Assert interrupt
// Check if the interrupt is from SCTL31 = HALFSPORT B DMA = RX channel.
R1 = 0x1F(Z);
// If it is then jump to RX data section
CC = R0 == R1;
if CC jump RX_Data;
jump TX_Data;
/*
R0 = R0 <<< 8;
R0 = R4 >>> 8;
*/
RX_Data:
// CSID is NW (non-writeable) but writing to it causes acknowledgement
// Empty buffer
R0=[REG_SPORT0_RXPRI_B];

jump END_Isr;

TX_Data:
R3 = [I0 ++ M0];

//[REG_SPORT0_TXPRI_A]=R3; //Send to left channel DAC
[REG_SPORT0_TXPRI_A]=R3;


// For I/Q calculation, multiply recieved data using a MAC calculation.
// Also consider that the MAC registers are only 40-bits wide at max, whereas MACs with 24 bits may cause
// overflows to over 40 bits.
// Worst case, the value would be sin^2 (max).
// Need to round down to 16 bits by right shifting and keeping sign from 24 bit value. (2^16*2^16 = 2^32)
// Then perform MAC.
jump END_Isr;

END_Isr:
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
[--SP] = RETS;                            // Push stack (only for nested calls)
// R1 Controls master clock, enable master clock
R1=0x03(X); R0=0x4000(X); call TWI_write;
// R65-66 Digital clock controllers enable just enable all
R1=0x7f(X); R0=0x40f9(X); call TWI_write; // Enable all clocks
R1=0x03(X); R0=0x40fa(X); call TWI_write; // Enable all clocks

// R15 Controls the serial port for transfers! enable master mode
R1=0x09(X); R0=0x4015(X); call TWI_write;

// R19 ADC Control, enable both ADC, leave everything else default
R1=0x13(X); R0=0x4019(X); call TWI_write;

// R22 Mixer 3 control register, enable and only left dac outputs into it
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

// R16 serial port control part 2, 64 Bits-per-audio frame (32 bit L, 32 bit R) no change
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

// 1 - SETUP THE CSO, MCTL, CTL Registers but do not enable
R0=0x00000001; [REG_SPORT0_CS0_B]=R0;
R0=0x00000000; [REG_SPORT0_MCTL_B]=R0;
R0=0x000A31F0; [REG_SPORT0_CTL_B]=R0;

R0=0x00000001; [REG_SPORT0_CS0_A]=R0;
R0=0x00000000; [REG_SPORT0_MCTL_A]=R0;
R0=0x020A31F0; [REG_SPORT0_CTL_A]=R0;
// 2 - No modification of sport err/CTL2 registers required
// 3 - R-MODIFY-WRITE enable MCTL, CTL
R0=[REG_SPORT0_MCTL_B]; bitset(R0, 0);
[REG_SPORT0_MCTL_B]=R0;

R0=[REG_SPORT0_CTL_B]; bitset(R0, 0);
[REG_SPORT0_CTL_B]=R0;

R0=[REG_SPORT0_MCTL_A]; bitset(R0, 0);
[REG_SPORT0_MCTL_A]=R0;

R0=[REG_SPORT0_CTL_A]; bitset(R0, 0);
[REG_SPORT0_CTL_A]=R0;

// The adau codec generally works in stereo L/RCLK mode, if want
// Only single channel then have to 
//R0=0x02001973; [REG_SPORT0_CTL_A]=R0;
// Sport B(receiver) will be in dsp mode as we are only interested in sampling
// One of the channels, active low for left channel data 

RTS;
sport_configure.end:

/*
// Set up in Standard dsp mode as we are only interested in 1 of the channels
R0=0x02023173; [REG_SPORT0_CTL_A]=R0;
// Sport B(receiver) will be in dsp mode as we are only interested in sampling
// One of the channels, active low for left channel data 
R0=0x00123173; [REG_SPORT0_CTL_B]=R0;
*/


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



/* 
get_audio:
wait_left:
// Wait for left data to synchronize since according to data sheet its the 
// First channel to stream.
R0=[REG_SPORT0_CTL_B]; //Put into R0 the control register for the sport
CC=BITTST(R0, 31);	//Check the bits which dictate if the buffer is full
if !CC jump wait_left;	//If the buffer isnt sufficiently full keep checking
R0=[REG_SPORT0_RXPRI_B]; //After partially full (more than 1 word (16 bits) read from reciever

R1 = [I0 ++ M0];

[REG_SPORT0_TXPRI_A]=R1; //Send to left channel DAC


wait_right:
R0=[REG_SPORT0_CTL_B]; 
CC=BITTST(R0, 31); 
if !CC jump wait_right;
R0=[REG_SPORT0_RXPRI_B];

[REG_SPORT0_TXPRI_A]=R1; //Send to right channel DAC

jump get_audio; //Loop
*/


/*
codec_configure:
[--SP] = RETS;                            // Push stack (only for nested calls)
// R1 Controls master clock, enable master clock
R1=0x03(X); R0=0x4000(X); call TWI_write;
// R65-66 Digital clock controllers enable just enable all
R1=0x7f(X); R0=0x40f9(X); call TWI_write; // Enable all clocks
R1=0x03(X); R0=0x40fa(X); call TWI_write; // Enable all clocks

// R15 Controls the serial port for transfers! enable master mode
R1=0x03(X); R0=0x4015(X); call TWI_write;

// R19 ADC Control, enable both ADC, leave everything else default
R1=0x13(X); R0=0x4019(X); call TWI_write;

// R22 Mixer 3 control register, enable and only left dac outputs into it
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

// R16 serial port control part 2, 64 Bits-per-audio frame (32 bit L, 32 bit R) no change
R1=0x00(X); R0=0x4016(X); call TWI_write;
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

*/