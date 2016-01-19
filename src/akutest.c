#include <stdio.h>
#include <stdlib.h>

#include <proto/exec.h>

#include <exec/types.h>
#include <exec/memory.h>
#include <exec/interrupts.h>

#include <hardware/intbits.h>

#define DEBUG			1

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

#define I2CSTA_SLAW_TX_ACK_RX	0x18
#define I2CSTA_SLAW_TX_NACK_RX	0x20
#define I2CSTA_DATA_TX_ACK_RX	0x28
#define I2CSTA_DATA_TX_NACK_RX	0x30

#define I2CSTA_IDLE		0xF8

#pragma dontwarn 113

typedef enum { 
	OP_NOP,	
	OP_READ,
	OP_WRITE
} op_t;

typedef enum {
	RESULT_OK,
	RESULT_ERR
} result_t;

/* glorious god object that holds the state of everything in this program; tldr */
typedef struct {
	op_t cur_op; 
	result_t cur_result;

	UBYTE *cp;

	BYTE sig_intr;
	LONG sigmask_intr;
	struct Task *MainTask;

	UBYTE *buf;
	ULONG buf_size;
	ULONG bytes_count;

	UBYTE slave_addr; 
#ifdef DEBUG
	int isr_called; /* how may times ISR was called */
	BOOL in_isr;
#endif /* DEBUG */
} pca9564_state_t;

UBYTE clockport_read(pca9564_state_t *, UBYTE);
void clockport_write(pca9564_state_t *, UBYTE, UBYTE);
__amigainterrupt void pca9564_isr(pca9564_state_t *);
void pca9564_dump_state(pca9564_state_t *);
void pca9564_send_start(pca9564_state_t *);
void pca9564_read(pca9564_state_t *, UBYTE, ULONG, UBYTE **);
void pca9564_write(pca9564_state_t *, UBYTE, ULONG, UBYTE **);
void pca9564_exec(pca9564_state_t *, UBYTE, ULONG, UBYTE **);

UBYTE
clockport_read(pca9564_state_t *sp, UBYTE reg) 
{
	UBYTE v;
	UBYTE *ptr;

	ptr = sp->cp + (reg * CLOCKPORT_STRIDE);
	v = *ptr;
#ifdef DEBUG 
	if (!(sp->in_isr))
		printf("DEBUG: read %x from %p\n", (int) v, (void*) ptr);
#endif /* DEBUG */

	return v;
}

void
clockport_write(pca9564_state_t *sp, UBYTE reg, UBYTE value)
{
	UBYTE *ptr;

	ptr = (sp->cp) + (reg * CLOCKPORT_STRIDE);
#ifdef DEBUG 
	if (!(sp->in_isr))
		printf("DEBUG: write %x to %p\n", (int) value, (void*) ptr);
#endif /* DEBUG */

	*ptr = value;
}

int main(void)
{
	pca9564_state_t sc;
	struct Interrupt *int6;

	UBYTE ctrl;
	UBYTE *buf;
	UBYTE size;

	size = 2;

#ifdef DEBUG
	sc.in_isr = FALSE;
	sc.isr_called = 0;
#endif /* DEBUG */
	sc.cp = CLOCKPORT_BASE;
	sc.cur_op = OP_NOP;

	sc.sig_intr = -1;
	if ((sc.sig_intr = AllocSignal(-1)) == -1) {
		printf("Couldn't allocate signal\n");
		return 1;
	}
	sc.sigmask_intr = 1L << sc.sig_intr;

	sc.MainTask = FindTask(NULL);

	if (int6 = AllocMem(sizeof(struct Interrupt), MEMF_PUBLIC|MEMF_CLEAR)) {
		int6->is_Node.ln_Type = NT_INTERRUPT;
		int6->is_Node.ln_Pri = -60;
		int6->is_Node.ln_Name = "PCA9564";
		int6->is_Data = (APTR)&sc;
		int6->is_Code = pca9564_isr;

		AddIntServer(INTB_EXTER, int6); 
	} else {
		printf("Can't allocate memory for interrupt node\n");
		FreeSignal(sc.sig_intr);
		return 1;
	}

	if (!(buf = AllocMem(size, MEMF_PUBLIC|MEMF_CLEAR))) {
		printf("Not enough memory to allocate the buffer\n");
		/* XXX: clean up */
	} 

	/* init the host controller */
	ctrl = I2CCON_CR_59KHZ | I2CCON_ENSIO;
	clockport_write(&sc, I2CCON, ctrl);
	Delay(50);

	buf[0] = 0x2;
	pca9564_write(&sc, 0x48, 1, &buf); 	/* XXX */
	/* read 2 bytes from 0x48 */
	pca9564_read(&sc, 0x48, size, &buf); 	/* XXX */

	printf("read result: %u, %u\n", buf[0], buf[1]);

	ctrl = 0;
	clockport_write(&sc, I2CCON, ctrl);

	FreeMem(buf, size);

	RemIntServer(INTB_EXTER, int6);
	FreeMem(int6, sizeof(struct Interrupt));
	FreeSignal(sc.sig_intr);

#ifdef DEBUG 
	printf("ISR was called %d times\n", sc.isr_called);
#endif /* DEBUG */
    
	return 0;
}

void
pca9564_write(pca9564_state_t *sp, UBYTE address, ULONG size, UBYTE **buf)
{
	sp->cur_op = OP_WRITE;
	pca9564_exec(sp, address, size, buf);
	sp->cur_op = OP_NOP;
}

void
pca9564_read(pca9564_state_t *sp, UBYTE address, ULONG size, UBYTE **buf)
{
	sp->cur_op = OP_READ;
	pca9564_exec(sp, address, size, buf);
	sp->cur_op = OP_NOP;
}

void
pca9564_exec(pca9564_state_t *sp, UBYTE address, ULONG size, UBYTE **buf)
{
	sp->slave_addr = address;
	sp->buf_size = size;
	sp->bytes_count = 0;
	sp->buf = *buf;

	pca9564_send_start(sp);

	Wait(sp->sigmask_intr);

	if (sp->cur_result != RESULT_OK) {
		printf("OP: failed!\n");	
		pca9564_dump_state(sp);
	}
#ifdef DEBUG
	else {
		printf("OP: successful!\n");
		pca9564_dump_state(sp);
	}
#endif /* DEBUG */

	sp->buf_size = 0;
	sp->slave_addr = 0;
}

void
pca9564_send_start(pca9564_state_t *sp) 
{
	UBYTE c;

	c = clockport_read(sp, I2CCON);
	c |= I2CCON_STA|I2CCON_AA;
	clockport_write(sp, I2CCON, c);	/* send START condition */

}

void
pca9564_dump_state(pca9564_state_t *sp)
{
	UBYTE c, s, d;

	c = clockport_read(sp, I2CCON);
	s = clockport_read(sp, I2CSTA);
	d = clockport_read(sp, I2CDAT);
	printf("I2CCON: %x, I2CSTA: %x, I2CDAT: %x\n", c, s, d);

}

/* Interrupt service routine. */
__amigainterrupt void
pca9564_isr(__reg("a1") pca9564_state_t *sp) 
{
	UBYTE v;

	sp->in_isr = TRUE;
	sp->isr_called++; 

	if (!(clockport_read(sp, I2CCON) & I2CCON_SI)) {
		sp->in_isr = FALSE;
		return;
	}

	switch (sp->cur_op) {
	case OP_READ:
		switch (clockport_read(sp, I2CSTA)) {
		case I2CSTA_START_SENT:		/* 0x08 */
			clockport_write(sp, I2CDAT, (sp->slave_addr << 1) | 1);
			v = clockport_read(sp, I2CCON);
			v &= ~(I2CCON_SI|I2CCON_STA); 
			clockport_write(sp, I2CCON, v);
			break;
		case I2CSTA_SLAR_TX_ACK_RX:	/* 0x40 */
			v = clockport_read(sp, I2CCON);
			v &= ~(I2CCON_SI);

			if ((sp->bytes_count+1) < sp->buf_size) 
				v |= (I2CCON_AA);
			else
				v &= ~(I2CCON_AA); /* last byte */

			clockport_write(sp, I2CCON, v);
			break;
		case I2CSTA_DATA_RX_ACK_TX:	/* 0x50 */
			sp->buf[sp->bytes_count] = clockport_read(sp, I2CDAT);
			(sp->bytes_count)++;

			v = clockport_read(sp, I2CCON);
			v &= ~(I2CCON_SI);

			if ((sp->bytes_count+1) < sp->buf_size) 
				v |= (I2CCON_AA);
			else
				v &= ~(I2CCON_AA); /* last byte */

			clockport_write(sp, I2CCON, v);
			break;
		case I2CSTA_DATA_RX_NACK_TX:	/* 0x58 */
			sp->buf[sp->bytes_count] = clockport_read(sp, I2CDAT);
			(sp->bytes_count)++;

			v = clockport_read(sp, I2CCON);
			v &= ~(I2CCON_SI);
			v |= (I2CCON_AA|I2CCON_STO);	/* send stop */

			clockport_write(sp, I2CCON, v);

			sp->cur_result = RESULT_OK;
			Signal(sp->MainTask, sp->sigmask_intr);
			break;
		default:
			clockport_write(sp, I2CCON, 0);
			sp->cur_result = RESULT_ERR;
			Signal(sp->MainTask, sp->sigmask_intr);
			break;
		}
		break;
	case OP_WRITE:
		switch (clockport_read(sp, I2CSTA)) {
		case I2CSTA_START_SENT:		/* 0x08 */
			clockport_write(sp, I2CDAT, (sp->slave_addr << 1) | 0);
			v = clockport_read(sp, I2CCON);
			v &= ~(I2CCON_SI|I2CCON_STA); 
			clockport_write(sp, I2CCON, v);
			break;
		case I2CSTA_SLAW_TX_ACK_RX:	/* 0x18 */
			clockport_write(sp, I2CDAT, sp->buf[sp->bytes_count]);
			v = clockport_read(sp, I2CCON);
			v &= ~(I2CCON_SI);
			clockport_write(sp, I2CCON, v);
			break;
		case I2CSTA_DATA_TX_ACK_RX:	/* 0x28 */
			v = clockport_read(sp, I2CCON);

			if (sp->bytes_count+1 < sp->buf_size) {
				clockport_write(sp, I2CDAT, sp->buf[sp->bytes_count]);

			} else {
				v |= (I2CCON_STO);
			}
			(sp->bytes_count)++;

			v &= ~(I2CCON_SI);
			clockport_write(sp, I2CCON, v);

			if (sp->bytes_count == sp->buf_size) {
				sp->cur_result = RESULT_OK;
				Signal(sp->MainTask, sp->sigmask_intr);
			}

			break;
		default:
			clockport_write(sp, I2CCON, 0);
			sp->cur_result = RESULT_ERR;
			Signal(sp->MainTask, sp->sigmask_intr);
			break;
		}
		break;
	case OP_NOP:
		clockport_write(sp, I2CCON, 0);
		sp->cur_result = RESULT_ERR;
		Signal(sp->MainTask, sp->sigmask_intr);
		break;
	}

	sp->in_isr = FALSE;
}

