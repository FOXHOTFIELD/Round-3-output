#ifndef MYHEADER_H
#define MYHEADER_H

#include "stm32f10x.h"                  // Device header
#include "stdint.h"

/* FreeRTOS core header must precede other kernel headers */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Project headers */
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "myClaim.h"
#include "PID.h"
#include "Motor.h"
#include "Menu.h"

#endif
