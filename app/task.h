#ifndef __TASK_H
#define __TASK_H

#include "CoOS.h"

/* ================= Task Stack Size ================= */

#define TASK_STK_SIZE   256

/* ================= Public API ================= */

void task_create(void);

#endif
