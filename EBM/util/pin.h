#ifndef PIN_H_
#define PIN_H_

/*
examples

#define Pin1 A,2,H
#define Pin2 B,3,L
             ^ ^ ^
          port | |
             bit \active level

void main(void)
{
	DRIVER(Pin1,OUT);	//as output
	DRIVER(Pin2,IN);		//as input
	DRIVER(Pin2,PULLUP);	// with pullup
	ON(Pin1);
	if (ACTIVE(Pin2)) OFF(Pin1);
	if (LATCH(Pin1)) { //if active level presents on Pin1 for drive something
		//do something
	}
}
*/

#define PM_BITNUM(port,bit,val) (bit)
#define BITNUM(x) PM_BITNUM(x)
#define BITMASK(x) (1<<PM_BITNUM(x))

#define PM_SETOUT(port,bit) (DDR##port |= (1<<(bit)))
#define PM_SETIN(port,bit) (DDR##port &= ~(1<<(bit)))
#define PM_SETPULLUP(port,bit) (PORT##port |= (1<<(bit)))
#define PM_SETHIZ(port,bit) (PORT##port &= ~(1<<(bit)))
#define PM_DRIVER(port,bit,val,mode) PM_SET##mode(port,bit)
/* dmode = OUT or IN, PULLUP or HIZ */
#define DRIVER(x,mode) PM_DRIVER(x,mode)

#define PM_SETL(port,bit,dummy) (PORT##port &= ~(1<<(bit)))
#define PM_SETH(port,bit,dummy) (PORT##port |= (1<<(bit)))
#define PM_SET(port,bit,val) PM_SET##val(port,bit,dummy)
#define ON(x)  PM_SET(x)
#define SET(x) PM_SETH(x)
#define CLR(x) PM_SETL(x)

#define PM_CLRL(port,bit,dummy) PM_SETH(port,bit,dummy)
#define PM_CLRH(port,bit,dummy) PM_SETL(port,bit,dummy)
#define PM_CLR(port,bit,val) PM_CLR##val(port,bit,dummy)
#define OFF(x) PM_CLR(x)

#define PM_PINH(port,bit,dummy) (PIN##port & (1<<(bit)))
#define PM_PINL(port,bit,dummy) !PM_PINH(port,bit,dummy)
#define PM_PIN(port,bit,val) PM_PIN##val(port,bit,dummy)
#define ACTIVE(x) PM_PIN(x)
#define PIN_H(x) PM_PINH(x)
#define PIN_L(x) PM_PINL(x)

#define PM_LATCHH(port,bit,dummy) (PORT##port & (1<<(bit)))
#define PM_LATCHL(port,bit,dummy) !PM_LATCHH(port,bit,dummy)
#define PM_LATCH(port,bit,val) PM_LATCH##val(port,bit,dummy)
#define LATCH(x) PM_LATCH(x)
#define LATCH_H(x) PM_LATCHH(x)
#define LATCH_L(x) PM_LATCHL(x)

#define PM_CPL(port,bit,val)      (PIN##port ^= (1 << (bit)))

#define CPL(x) PM_CPL(x)
#define TOGGLE(x) PM_CPL(x)


#endif /* PIN_H_ */