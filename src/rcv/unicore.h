#ifndef RTKLIB_RCV_UNICORE_H
#define RTKLIB_RCV_UNICORE_H

#include "../rtklib.h"

int init_unicore_b2b(raw_t *raw);
void free_unicore_b2b(raw_t *raw);
const B2bmask_t *unicore_b2b_mask(const raw_t *raw);
int input_unicore(raw_t *raw, uint8_t data);
int input_unicoref(raw_t *raw, FILE *fp);

#endif
