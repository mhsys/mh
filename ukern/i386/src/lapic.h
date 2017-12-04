#ifndef __lapic_h
#define __lapic_h

unsigned lapic_getcurrent(void);
void lapic_init(paddr_t, unsigned);
void lapic_add(uint16_t, uint16_t);
void lapic_add_nmi(uint8_t, int);
void lapic_platform_done(void);
void lapic_eoi(void);

#endif
