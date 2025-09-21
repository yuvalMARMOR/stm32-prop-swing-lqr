# STM32 Propeller Swing LQR

Propeller swing control on STM32 using model-matching with LQR and a reference filter.  
Closed-loop stabilization and control of a pendulum-like swing driven by propellers.

## Features
- LQR controller with reference filter (model-matching)
- Motor control via PWM (TIM3 PC6/PC7) and DAC (PA4/PA5)
- Position feedback via encoder (TIM4 PB6/PB7) and potentiometer (ADC1 PA1)
- UART logging/telemetry (USART2 PA2 TX)
- Clean structure (src/include/docs) with report & Simulink model

## Project Structure
    .
     src/        # Source files (.c)
     include/    # Header files (.h)
     docs/       # Final report & Simulink model
     .gitignore  # Ignore build/temp files
     README.md   # Project overview

## Requirements
- STM32F10x MCU  
- ST-LINK programmer/debugger  
- STM32 toolchain (e.g., STM32CubeIDE or PlatformIO)  
- MATLAB/Simulink (optional, for the provided model)

## Quick Start
    git clone https://github.com/yuvalMARMOR/stm32-prop-swing-lqr.git
    cd stm32-prop-swing-lqr
    # Open with STM32CubeIDE or PlatformIO and build for STM32F10x

## Documentation
- [Final Report (PDF)](docs/Final_Report.pdf)  
- [Simulink Model](docs/read_data_SWING.slx)

## Authors
- Yuval Marmor  
- Rabea Hawa

## License
MIT License
