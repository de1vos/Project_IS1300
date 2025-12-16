/*
 * Traffic_Logic.h
 *
 *  Created on: 09 Dec 2025
 *      Author: augus
 */

#ifndef TRAFFIC_LOGIC_H
#define TRAFFIC_LOGIC_H

#include <stdint.h>
#include "cmsis_os.h" // For OS types

// --- 1. Bitmasks for Shift Registers (Based on Schematic [cite: 199]) ---
// U3 (First Byte sent, furthest chip) - Controls TL3, TL4
#define TL3_RED    0x01
#define TL3_YELLOW 0x02
#define TL3_GREEN  0x04
#define TL4_RED    0x08
#define TL4_YELLOW 0x10
#define TL4_GREEN  0x20

// U2 (Second Byte sent) - Controls TL2, PL2
#define TL2_RED    0x01
#define TL2_YELLOW 0x02
#define TL2_GREEN  0x04
#define PL2_RED    0x08
#define PL2_GREEN  0x10
#define PL2_BLUE   0x20

// U1 (Third Byte sent, closest chip) - Controls TL1, PL1
#define TL1_RED    0x01
#define TL1_YELLOW 0x02
#define TL1_GREEN  0x04
#define PL1_RED    0x08
#define PL1_GREEN  0x10
#define PL1_BLUE   0x20

// --- 2. Queue Event Definitions ---
typedef enum {
    EVENT_NO_EVENT = 0,
    EVENT_PED1_BTN_PRESS,    // Task 1: Button 1 Pressed
    EVENT_PED2_BTN_PRESS,    // Task 3: Button 2 Pressed
    EVENT_CAR_DETECTED_TL1,  // Task 2: Car Sensors
    EVENT_CAR_DETECTED_TL2,
    EVENT_CAR_DETECTED_TL3,
    EVENT_CAR_DETECTED_TL4
} InputEvent_t;

// --- 3. System States (FSM) ---
typedef enum {
    STATE_INIT,

    // --- Vertical Lane (TL1 & TL3) ---
    STATE_VERTICAL_PREPARE,   // Red -> Orange (Req R1.6/R2.3)
    STATE_VERTICAL_GREEN,     // Green
    STATE_VERTICAL_STOPPING,  // Green -> Orange

    // --- Horizontal Lane (TL2 & TL4) ---
    STATE_HORIZONTAL_PREPARE, // Red -> Orange (Req R1.6/R2.3)
    STATE_HORIZONTAL_GREEN,   // Green
    STATE_HORIZONTAL_STOPPING,// Green -> Orange

    // --- Pedestrians ---
    STATE_PED1_WALK,          // PL1 Green
    STATE_PED2_WALK,          // PL2 Green

    // --- Safety ---
    STATE_ALL_RED             // Buffer state
} TrafficState_t;

typedef struct {
    uint32_t toggleFreqVal;      // Blink frequency (ms)
    uint32_t pedestrianDelay;    // Delay before stopping cars (ms)
    uint32_t walkingDelay;       // Walk duration (ms)
    uint32_t orangeDelay;        // Yellow light duration (ms)
    uint32_t greenDelay;
    uint32_t redDelayMax;
} TrafficConfig_t;

// Declare that this variable exists (defined in main.c)
extern TrafficConfig_t sysConfig;

// Function Prototypes
void TL_init(void);
void toggleFreq(void);
void togglePed1(void);
void togglePed2(void);
void walkingDelay(void);
void readTrafficState(void);
void writeTrafficState(void);

// Helper delays strictly for Logic
void trafficDelay(uint32_t delay_ms);

#endif
