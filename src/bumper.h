#ifndef BUMPER_H
#define BUMPER_H

#include <stdint.h>
#include "tools/variablesAccess.h"
#include <tools/variablesAccess.h>
#include <stdint.h>

void bumper_init();

uint8_t bumper_getContacts();

bitset8_t bumper_getBumpers();

#endif /* BUMPER_H */