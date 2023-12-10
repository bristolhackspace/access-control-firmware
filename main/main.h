#pragma once


#include <stdbool.h>

#define CARD_ID_MAX_LEN 8

bool is_unlocked(void);
bool is_update_pending(void);
bool is_in_use(void);