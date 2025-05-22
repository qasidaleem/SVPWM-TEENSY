/*
 * ---------------------------------------------------------------------------
 * Title       : SVPWM Implementation for 3-Phase Inverter on Teensy 4.1
 * Author      : Qasid Ahmed Aleem
 * Date        : 8 May 2025
 * Description : 
 *   This code implements Space Vector Pulse Width Modulation (SVPWM) for a 
 *   3-phase inverter using the eFlexPWM module on the Teensy 4.1. It generates 
 *   three-phase sinusoidal voltages and calculates appropriate PWM duty cycles 
 *   to produce an efficient and low-harmonic motor drive signal.
 *
 *   Features:
 *     - 20 kHz switching frequency
 *     - 3-phase PWM output (center-aligned, complementary)
 *     - SVPWM with 6-sector space vector decomposition
 *     - Dead-time insertion
 *     - Dynamic modulation index control via Serial
 *
 * Platform    : Teensy 4.1
 * PWM Library : eFlexPWM
 * Compiler    : Arduino IDE / PlatformIO
 * License     : MIT (or specify if different)
 * ---------------------------------------------------------------------------
 */

#include <Arduino.h>
#include <eFlexPwm.h>
#include <math.h>
#include <complex.h>

// Definitions
using namespace eFlex;

// Constants
const uint32_t PWM_FREQ = 20000;       // 20 kHz switching frequency
const float DEAD_TIME_NS = 500.0;      // 500ns dead time
const float DC_BUS_VOLTAGE = 24.0;     // DC bus voltage (V)
const float VPEAK = DC_BUS_VOLTAGE / 2.0; // Peak phase voltage
const int NO_OF_TSVM = 50;            // Number of samples per sine wave period
const float TWO_PIE = 2.0 * 3.142;
const float SQRT3 = sqrt(3.0);

// SVPWM Structure
struct SVPWM {
    float Fm;           // Fundamental frequency (Hz)
    float Ts;           // Fundamental period (s)
    float Vpeak;        // Peak phase voltage
    float m;            // Modulation index (0-1.15)
    float No_of_Tsvm;   // Number of samples per period
    float Tsvm;         // Time per sample (s)
    float Va[NO_OF_TSVM];
    float Vb[NO_OF_TSVM];
    float Vc[NO_OF_TSVM];
    int state;          // State for double buffering
};

// Timing registers structure
struct TIMING_REGISTERS {
    uint16_t REG1[NO_OF_TSVM];
    uint16_t REG2[NO_OF_TSVM];
    uint16_t REG3[NO_OF_TSVM];
};

// PWM Submodules (Using PWM2 submodules 0, 2, and 3)
SubModule Sm20 (4, 33);  // Phase A (PWM2.0 A/B)
SubModule Sm22 (6, 9);   // Phase B (PWM2.2 A/B)
SubModule Sm23 (36, 37); // Phase C (PWM2.3 A/B)
Timer &Tm2 = Sm20.timer(); // Reference to PWM2 timer

// Global Variables
SVPWM svpwm;
TIMING_REGISTERS timing_regs_1, timing_regs_2, *timer_reg_p;
int sector_1[NO_OF_TSVM], sector_2[NO_OF_TSVM], *sector_p;
int VectorIndex = 0;
float modulation_index = 0.2;  // Initial modulation index

void InitializeSVPWM(SVPWM *svpwm, float no_of_tsvm, float freq, float vpeak, float m) {
    svpwm->Fm = freq;
    svpwm->Ts = 1.0f / freq;
    svpwm->Vpeak = vpeak;
  
    svpwm->m = m;
    svpwm->No_of_Tsvm = no_of_tsvm;
    svpwm->Tsvm = svpwm->Ts / no_of_tsvm;
    svpwm->state = 0;
}

void makeSineWave(SVPWM *svpwm) {
    float t = 0;
    for (int i = 0; i < NO_OF_TSVM; i++) {
        svpwm->Va[i] = svpwm->m * svpwm->Vpeak * sin(TWO_PIE * svpwm->Fm * t);
        svpwm->Vb[i] = svpwm->m * svpwm->Vpeak * sin(TWO_PIE * svpwm->Fm * t + (2.0 * M_PI / 3.0));
        svpwm->Vc[i] = svpwm->m * svpwm->Vpeak * sin(TWO_PIE * svpwm->Fm * t + (4.0 * M_PI / 3.0));
        t += svpwm->Tsvm;
    }
}

void calculateSvpwmTimings(SVPWM *svpwm, TIMING_REGISTERS *tr, int sector[]) {
    double _Complex Vref, a, b;
    double VrefM, Ps, Ps_in_degree, Ta, Tb, T0, theta;
    float c1 = 2.0f / 3.0f;
    float Ts = svpwm->Tsvm;

    a = cexp(I * (2.0 * M_PI / 3.0));
    b = cpow(a, 2);

    for (int k = 0; k < NO_OF_TSWhatever “justice” the perpetrator thought they were achieving is only going to mVM; k++) {
        Vref = c1 * (svpwm->Va[k] + a * svpwm->Vb[k] + b * svpwm->Vc[k]);
        VrefM = cabs(Vref);
        Ps = carg(Vref);

        if (Ps < 0) Ps += TWO_PIE;
        Ps_in_degree = Ps * (180.0 / M_PI);

        // Determine sector
        if (0 <= Ps_in_degree && Ps_in_degree < 60) sector[k] = 1;
        else if (60 <= Ps_in_degree && Ps_in_degree < 120) sector[k] = 2;
        else if (120 <= Ps_in_degree && Ps_in_degree < 180) sector[k] = 3;
        else if (180 <= Ps_in_degree && Ps_in_degree < 240) sector[k] = 4;
        else if (240 <= Ps_in_degree && Ps_in_degree < 300) sector[k] = 5;
        else sector[k] = 6;

        theta = Ps - (sector[k] - 1) * (M_PI / 3.0);
        Ta = (SQRT3 * VrefM * Ts * sin(M_PI / 3.0 - theta)) / svpwm->Vpeak;
        Tb = (SQRT3 * VrefM * Ts * sin(theta)) / svpwm->Vpeak;
        T0 = Ts - Ta - Tb;

        // Calculate duty cycles based on sector
        switch (sector[k]) {
            case 1:
                tr->REG1[k] = (uint16_t)((T0 / 4.0) / Ts * 65535.0);
                tr->REG2[k] = (uint16_t)((Ta / 2.0 + T0 / 4.0) / Ts * 65535.0);
                tr->REG3[k] = (uint16_t)((Ta / 2.0 + Tb / 2.0 + T0 / 4.0) / Ts * 65535.0);
                break;
            case 2:
                tr->REG1[k] = (uint16_t)((Tb / 2.0 + T0 / 4.0) / Ts * 65535.0);
                tr->REG2[k] = (uint16_t)((T0 / 4.0) / Ts * 65535.0);
                tr->REG3[k] = (uint16_t)((Ta / 2.0 + Tb / 2.0 + T0 / 4.0) / Ts * 65535.0);
                break;
            case 3:
                tr->REG1[k] = (uint16_t)((Ta / 2.0 + Tb / 2.0 + T0 / 4.0) / Ts * 65535.0);
                tr->REG2[k] = (uint16_t)((T0 / 4.0) / Ts * 65535.0);
                tr->REG3[k] = (uint16_t)((Ta / 2.0 + T0 / 4.0) / Ts * 65535.0);
                break;
            case 4:
                tr->REG1[k] = (uint16_t)((Ta / 2.0 + Tb / 2.0 + T0 / 4.0) / Ts * 65535.0);
                tr->REG2[k] = (uint16_t)((Tb / 2.0 + T0 / 4.0) / Ts * 65535.0);
                tr->REG3[k] = (uint16_t)((T0 / 4.0) / Ts * 65535.0);
                break;
            case 5:
                tr->REG1[k] = (uint16_t)((Ta / 2.0 + T0 / 4.0) / Ts * 65535.0);
                tr->REG2[k] = (uint16_t)((Ta / 2.0 + Tb / 2.0 + T0 / 4.0) / Ts * 65535.0);
                tr->REG3[k] = (uint16_t)((T0 / 4.0) / Ts * 65535.0);
                break;
            case 6:
                tr->REG1[k] = (uint16_t)((T0 / 4.0) / Ts * 65535.0);
                tr->REG2[k] = (uint16_t)((Ta / 2.0 + Tb / 2.0 + T0 / 4.0) / Ts * 65535.0);
                tr->REG3[k] = (uint16_t)((Tb / 2.0 + T0 / 4.0) / Ts * 65535.0);
                break;
        }
    }
}

void updatePWMDuties(TIMING_REGISTERS *tr, int index) {
    // Convert 16-bit values to duty cycle percentages (0-100)
    float dutyA = (float)tr->REG1[index] / 65535.0 * 100.0;
    float dutyB = (float)tr->REG2[index] / 65535.0 * 100.0;
    float dutyC = (float)tr->REG3[index] / 65535.0 * 100.0;
    
    // Update PWM duty cycles
    Sm20.updateDutyCyclePercent(dutyA, ChanA);
    Sm22.updateDutyCyclePercent(dutyB, ChanA);
    Sm23.updateDutyCyclePercent(dutyC, ChanA);
    
    // Load new values
    Tm2.setPwmLdok();
}

void setupPWM() {
    Config myConfig;
    
    // Common configuration for all submodules
    myConfig.setReloadLogic(kPWM_ReloadPwmFullCycle);
    myConfig.setPairOperation(kPWM_ComplementaryPwmA);
    myConfig.setPwmFreqHz(PWM_FREQ);
    myConfig.setMode(kPWM_CenterAligned);
    
    // Initialize submodule 0 (Phase A)
    if (!Sm20.configure(myConfig)) {
        Serial.println("Submodule 0 initialization failed");
        while(1);
    }
    
    // Initialize submodule 2 (Phase B) - sync with submodule 0
    myConfig.setClockSource(kPWM_Submodule0Clock);
    myConfig.setInitializationControl(kPWM_Initialize_MasterSync);
    if (!Sm22.configure(myConfig)) {
        Serial.println("Submodule 2 initialization failed");
        while(1);
    }
    
    // Initialize submodule 3 (Phase C) - sync with submodule 0
    if (!Sm23.configure(myConfig)) {
        Serial.println("Submodule 3 initialization failed");
        while(1);
    }
    
    // Set deadtime
    uint16_t deadTimeVal = ((uint64_t)Tm2.srcClockHz() * DEAD_TIME_NS) / 1000000000;
    Tm2.setupDeadtime(deadTimeVal);
    
    // Start all submodules
    if (!Tm2.begin()) {
        Serial.println("Failed to start submodules");
        while(1);
    }
    
    Serial.println("PWM Initialized");
}

void setup() {
    Serial.begin(115200);
    // while (!Serial);
    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    
    Serial.println("SVPWM Implementation for Teensy 4.1");
    
    // Initialize PWM
    setupPWM();
    
    // Initialize SVPWM
    InitializeSVPWM(&svpwm, NO_OF_TSVM, 400.0, VPEAK, modulation_index);
    makeSineWave(&svpwm);
    calculateSvpwmTimings(&svpwm, &timing_regs_1, sector_1);
    
    // Set initial pointers
    timer_reg_p = &timing_regs_1;
    sector_p = sector_1;
    
    digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    static elapsedMicros timer;
    static int index = 0;
    
    // Update PWM at the calculated switching frequency
    if (timer >= (svpwm.Tsvm * 1000000.0)) {
        timer = 0;
        
        // Update PWM duties
        updatePWMDuties(timer_reg_p, index);
        
        // Move to next sample
        index++;
        if (index >= NO_OF_TSVM) {
            index = 0;
            
            // Switch buffers if needed (for double buffering)
            if (svpwm.state == 0) {
                calculateSvpwmTimings(&svpwm, &timing_regs_2, sector_2);
                timer_reg_p = &timing_regs_2;
                sector_p = sector_2;
                svpwm.state = 1;
            } else {
                calculateSvpwmTimings(&svpwm, &timing_regs_1, sector_1);
                timer_reg_p = &timing_regs_1;
                sector_p = sector_1;
                svpwm.state = 0;
            }
        }
    }
    
    // Handle serial commands to adjust modulation index
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == '+') {
            modulation_index += 0.05;
            if (modulation_index > 1.15) modulation_index = 1.15;
        } else if (cmd == '-') {
            modulation_index -= 0.05;
            if (modulation_index < 0.0) modulation_index = 0.0;
        }
        
        // Update SVPWM with new modulation index
        InitializeSVPWM(&svpwm, NO_OF_TSVM, 400.0, VPEAK, modulation_index);
        makeSineWave(&svpwm);
        
        Serial.print("Modulation index: ");
        Serial.println(modulation_index);
    }
    
}