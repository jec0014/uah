#### Timer_A

##### Two main parts to the hardware

- Timer Block: Based on the counter, a 16-bit register, TAR. TAIFG is raised when the counter returns to 0;
- Capture / Compare Channels: Based on register TACCRn, which all work in the same way except TACCR0.

##### Capture / Compare Channels Can

- Capture an input.
	- Entails recording the 'time' (value of TAR) at which the input changes in TACCRn.
- Compare the current value of TAR with the value stored in TACCRn and update an output when they match.
- Request an interrupt by setting its flag TACCRn CCIFG on either of these events.
- Sample an input at a compare event. Useful for serial comms.

##### General Notes

- The number of Capture / Compare Channels is sometimes appended to the name, as in Timer_A3.
- Channel 0 (TACCR0) is unavailable in Up and Up/Down modes. It has its own interrupt vector w/ higher priority.
- TAR increments on the rising edge.

###### An important distinction with registers

The demo lab code tends to use 'TA0CTL' when it really means 'TACTL'. 
This is most likely because it's possible to have multiple Timer_A's, 
which would make specifying *which* necessary. As it stands, we're only 
using the first Timer_A, and in the background libraries, TA0CTL is simply 
redefined as TACTL, rendering it useless. Please be aware of this while 
going through any MSP code, the registers are one and the same. To help 
alleviate the headache this causes, I will attempt to document with both names 
and then only use my preferred one while coding.

##### Register Definitions

Hint: All of the below registers, for both timers, are reset on POR.

Hint: Cap/Comp is capture & compare.

###### Timer_A / Timer0_A3

	Register				Access		Alt Access		Address
	control					TACTL		TA0CTL			0x0160 (rw)
	counter					TAR			TA0R			0x0170 (rw)
	cap/comp control 0		TACCTL0		TA0CCTL			0x0162 (rw)
	cap/comp compare 0		TACCR0		TA0CCR0			0x0172 (rw)
	cap/comp control 1		TACCTL1		TA0CCTL1		0x0164 (rw)
	cap/comp compare 1		TACCR1		TA0CCR1			0x0174 (rw)
	cap/comp control 2		TACCTL2		TA0CCTL2		0x0166 (rw)
	cap/comp compare 2		TACCR2		TA0CCR2			0x0176 (rw)
	interrupt vector		TAIV		TA0IV			0x012E (r )

###### Timer1_A

	Register				Access		Address
	control					TA1CTL		0x0180 (rw)
	counter					TA1R		0x0190 (rw)
	cap/comp control 0		TA1CCTL0	0x0182 (rw)
	cap/comp compare 0		TA1CCR0		0x0192 (rw)
	cap/comp control 1		TA1CCTL1	0x0184 (rw)
	cap/comp compare 1		TA1CCR1		0x0194 (rw)
	cap/comp control 2		TA1CCTL2	0x0186 (rw)
	cap/comp compare 2		TA1CCR2		0x0196 (rw)
	cap/comp control 3		TA1CCTL3	0x0188 (rw)
	cap/comp compare 3		TA1CCR3		0x0198 (rw)
	cap/comp control 4		TA1CCTL4	0x018A (rw)
	cap/comp compare 4		TA1CCR4		0x019A (rw)
	interrupt vector		TA1IV		0x011E (r )

##### Register Overview

	TACTL (TA0CTL) Bits:
	[15]-[10]	[9]			[8]			[7]		[6]		[5]		[4]		[3]			[2]		[1]		[0]
	[Unused]	[TASSEL9]	[TASSEL8]	[ID7]	[ID6]	[MC5]	[MC4]	[Unused]	[TACLR]	[TAIE]	[TAIFG]
				rw-(0)		rw-(0)		rw-(0)	rw-(0)	r0-(w)	rw-(0)	rw-(0)		w-(0)	rw-(0)	rw-(0)

- TASSELx
	- Timer_A clock source select
		- (TASSEL9 TASSEL8) Clock selection
		- 00 TACLK
		- 01 ACLK
		- 10 SMCLK
		- 11 Inverted TACLK
- IDx
	- Input divider. Selects the divider for the input clock.
		- (ID7 ID6) Division factor
		- 00 /1
		- 01 /2
		- 10 /4
		- 11 /8
- MCx
	- Mode control. Setting MCx = 0x00 when Timer_A is not in use conserves power.
		- (MC5 MC4) Mode
		- 00 Stop: timer is halted
		- 01 Up: timer counts to TACCR0
		- 10 Continuous: timer counts to 0xFFFF
		- 11 Up/Down: timer counts up to TACCR0 then down to 0x0000
- TACLR
	- Timer_A clear. Setting this bit resets TAR, the clock divider, and the count direction.
- TAIE
	- Timer_A interrupt enable. This bit enables the TAIFG interrupt request.
		- 0 Interrupt disabled
		- 1 Interrupt enabled
- TAIFG
	- Timer_A interrupt flag
		- 0 No interrupt pending
		- 1 Interrupt pending

TAR Bits:
	[15]-[0]
	[TARx]
	rw-(0)

- TARx
	- Timer_A register. The count of Timer_A.

TACCRx, Capture/Compare Register x
	[15]-[0]
	[TACCRx]
	rw-(0)

- TACCRx
	- Timer_A capture/compare register.
	- Compare mode: TACCRx holds the data for the comparison to the timer value in the Timer_A register, TAR
	- Capture mode: The Timer_A register, TAR, is copied into the TACCRx register when a capture is performed.

##### TACCTLx, Capture/Compare Control Register x

	[15]	[14]	[13]		[12]		[11]	[10]	[9]			[8]
	[CM15]	[CM14]	[CCIS13]	[CCIS12]	[SCS]	[SCCI]	[Unused]	[CAP]
	rw-(0)	rw-(0)	rw-(0)		rw-(0)		r0-(w)	r		r0			rw-(0)
	
	[7]			[6]		[5]		[4]		[3]			[2]		[1]		[0]
	[OUTMOD7]	[OUTMOD6]	[MC5]	[MC4]	[Unused]	[TACLR]	[TAIE]	[TAIFG]
	rw-(0)	rw-(0)	r0-(w)	rw-(0)	rw-(0)		w-(0)	rw-(0)	rw-(0)
	
##### MCx Bits
- Stop (00): Timer is halted. Registers retain their values.
- Continuous (10): Counter runs through its full range from 0x0000 to 0xFFFF.
- Up (01): Counts from 0 up to the value in TACCR0, the capture/compare register for channel 0. Period is TACCR0+1.
- Up/Down (11): Counts from 0 to TACCR0, then down again to 0. Period is 2*TACCR0.


















