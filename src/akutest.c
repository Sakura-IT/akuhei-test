/* note: this is overly complex */
/* note2: globals, globals everywhere */

#include <stdio.h>
#include <stdlib.h>

#include <proto/exec.h>

#include <exec/types.h>
#include <exec/memory.h>
#include <exec/interrupts.h>

#include <hardware/intbits.h>

#define CLOCKPORT_BASE		0xD80001
#define CLOCKPORT_STRIDE	4

#define I2CSTA			0
#define I2CTO			0
#define I2CDAT			2
#define I2CADR			1
#define I2CCON			3

#define I2CCON_CR0		(1 << 0)
#define I2CCON_CR1		(1 << 1)
#define I2CCON_CR2		(1 << 2)
#define I2CCON_CR_88KHZ		(0x4)
#define I2CCON_CR_59KHZ		(0x5)
#define I2CCON_CR_MASK		(0x7)
#define I2CCON_SI		(1 << 3)
#define I2CCON_STO		(1 << 4)
#define I2CCON_STA		(1 << 5)
#define I2CCON_ENSIO		(1 << 6)
#define I2CCON_AA		(1 << 7)

#define I2CSTA_START_SENT	0x08
#define I2CSTA_SLAR_TX_ACK_RX	0x40
#define I2CSTA_SLAR_TX_NACK_RX	0x48
#define I2CSTA_DATA_RX_ACK_TX	0x50
#define I2CSTA_DATA_RX_NACK_TX	0x58

#define I2CSTA_IDLE		0xF8

#pragma dontwarn 113

typedef enum { 
	OP_NOP,	
	OP_READ 
} op_t;

UBYTE clockport_read(UBYTE reg);
void clockport_write(UBYTE reg, UBYTE val);
void pca9564_isr(void);
void pca9564_dump_state(void);
void pca9564_send_start(void);
void pca9564_send_stop(void);
void pca9564_read_1(UBYTE);

static UBYTE *cp = CLOCKPORT_BASE;

static UBYTE slave_addr = 0; 

static BYTE sig_intr = -1;
static LONG sigmask_intr;
static struct Task *MainTask = NULL;

static op_t cur_op = OP_NOP;
static UBYTE bytes_left = 0;

static const BOOL debug = TRUE;
static int isr_called = 0; /* how may times ISR was called */

UBYTE
clockport_read(UBYTE reg) 
{
	UBYTE v;
	UBYTE *ptr;

	ptr = cp + (reg * CLOCKPORT_STRIDE);
	v = *ptr;
	if (debug)
		printf("DEBUG: read %x from %p\n", (int) v, (void*) ptr);

	return v;
}

void
clockport_write(UBYTE reg, UBYTE value)
{
	UBYTE *ptr;

	ptr = cp + (reg * CLOCKPORT_STRIDE);
	if (debug)
		printf("DEBUG: write %x to %p\n", (int) value, (void*) ptr);
	*ptr = value;
}

int main(void)
{
	struct Interrupt *int6;
	UBYTE ctrl;
	UBYTE void_data;

	if ((sig_intr = AllocSignal(-1)) == -1) {
		printf("Couldn't allocate signal\n");
		return 1;
	}
	sigmask_intr = 1L << sig_intr;

	MainTask = FindTask(NULL);

	if (int6 = AllocMem(sizeof(struct Interrupt), MEMF_PUBLIC|MEMF_CLEAR)) {
		int6->is_Node.ln_Type = NT_INTERRUPT;
		int6->is_Node.ln_Pri = -60;
		int6->is_Node.ln_Name = "PCA9564";
		int6->is_Data = (APTR)&void_data;
		int6->is_Code = pca9564_isr;

		AddIntServer(INTB_EXTER, int6); 
	} else {
		printf("Can't allocate memory for interrupt node\n");
		FreeSignal(sig_intr);
		return 1;
	}

	/* init the host controller */
	ctrl = I2CCON_CR_59KHZ | I2CCON_ENSIO;
	clockport_write(I2CCON, ctrl);
	Delay(50);

	pca9564_read_1(0x48); 	/* XXX */

	ctrl = 0;
	clockport_write(I2CCON, ctrl);

	RemIntServer(INTB_EXTER, int6);
	FreeMem(int6, sizeof(struct Interrupt));
	FreeSignal(sig_intr);

	printf("ISR was called %d times\n", isr_called);
    
    return 0;
}

void
pca9564_read_1(UBYTE address)
{
	/*assert(cur_op == OP_NOP);
	assert(address > 1);*/

	cur_op = OP_READ;
	slave_addr = address;
	bytes_left = 1;

	printf("gonna send start\n");
	pca9564_send_start();

	Wait(sigmask_intr);

	Delay(10);
	pca9564_dump_state();
/*
	printf("gonna send stop\n");
	pca9564_send_stop();
	pca9564_dump_state();
*/

	slave_addr = 0;
	cur_op = OP_NOP;
}

void
pca9564_send_stop(void)
{
	UBYTE c;

	c = clockport_read(I2CCON);
	c |= I2CCON_STO;
	c &= (I2CCON_STA);
	clockport_write(I2CCON, c);	/* send STOP condition */
}

void
pca9564_send_start(void) 
{
	UBYTE c;

	c = clockport_read(I2CCON);
	c |= I2CCON_STA|I2CCON_AA;
	clockport_write(I2CCON, c);	/* send START condition */

}

void
pca9564_dump_state(void)
{
	UBYTE c, s, d;

	c = clockport_read(I2CCON);
	s = clockport_read(I2CSTA);
	d = clockport_read(I2CDAT);
	printf("I2CCON: %x, I2CSTA: %x, I2CDAT: %x\n", c, s, d);

}

/* Interrupt service routine. */
void
pca9564_isr(void) 
{
	UBYTE *conptr, *datptr, *staptr;
	UBYTE v;

	isr_called++; 

	conptr = cp + (I2CCON * CLOCKPORT_STRIDE);
	datptr = cp + (I2CDAT * CLOCKPORT_STRIDE);
	staptr = cp + (I2CSTA * CLOCKPORT_STRIDE);

	if (! (*conptr & I2CCON_SI)) 
		return;

	switch (cur_op) {
	case OP_READ:
		switch (*staptr) {
		case I2CSTA_START_SENT:		/* 0x08 */
			*datptr = (slave_addr << 1) | 1;
			*conptr &= ~(I2CCON_SI|I2CCON_STA); /*|I2CCON_AA);*/
			break;
		case I2CSTA_SLAR_TX_ACK_RX:	/* 0x40 */
			v = *conptr;
			v &= ~(I2CCON_SI|I2CCON_AA); /*XXX: last byte */
			*conptr = v;
			break;
		case I2CSTA_DATA_RX_NACK_TX:	/* 0x58 */
			v = *conptr;
			v &= ~(I2CCON_SI);
			v |= (I2CCON_AA|I2CCON_STO);
			*conptr = v;
			Signal(MainTask, sigmask_intr);
			break;
		default:
			*conptr = 0;
			break;
		}
		break;
	case OP_NOP:
		*conptr = 0;
		break;
	}


}

