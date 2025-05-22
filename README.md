# SVPWM-TEENSY
This code implements Space Vector Pulse Width Modulation (SVPWM) for a 3-phase inverter using a Teensy 4.1 microcontroller and the eFlexPWM library. SVPWM is a sophisticated technique used to generate 3-phase sinusoidal voltages for motor control applications, offering better utilization of the DC bus voltage and reduced harmonic distortion.
Key Features:

    Target Platform: Teensy 4.1 (with eFlexPWM hardware support)

    PWM Frequency: 20 kHz

    SVPWM Fundamentals:

        Uses 50 discrete time steps per sine wave period.

        Generates 3-phase voltages (Va, Vb, Vc) using sine wave functions.

        Converts these into a space vector and determines the appropriate sector (1–6) for each sample.

        Calculates duty cycles (Ta, Tb, T0) for each sector and converts them into 16-bit PWM values.

Core Components:

    SVPWM Structure: Holds parameters like frequency, modulation index, and per-phase voltage samples.

    Timing Registers: Stores calculated duty cycle values for each PWM phase per time step.

    PWM SubModules: Three PWM submodules control the three motor phases (A, B, C).

    Double Buffering: Alternates between two timing buffers to maintain smooth PWM updates.

    Serial Interface: Allows real-time adjustment of the modulation index using + and - commands.

Flow:

    Initialization (setup):

        Configures PWM submodules with proper frequency, mode, and dead time.

        Generates the SVPWM sine waves.

        Computes initial timing values.

    Loop Execution:

        Updates PWM duty cycles at regular intervals based on SVPWM timing tables.

        Cycles through samples and recomputes timings periodically.

        Allows dynamic modulation index tuning over serial.

Use Cases:

    Motor control for BLDC/PMSM motors.

    Inverter simulations or educational demonstrations.

    Real-time control systems where high-efficiency sinusoidal output is needed.
