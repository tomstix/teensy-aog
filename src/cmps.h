#pragma once

#include "main.h"

void initCMPS();
void cmpsWorker();
int16_t requestTwoSigned(int, int);
int8_t requestSingle(int, int);
int requestBytes(int, int, int);