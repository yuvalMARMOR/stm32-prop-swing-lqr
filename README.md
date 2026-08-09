<div align="center">

# Propeller Swing Control on STM32

**Embedded angle control for a two-propeller rotary arm**

Model-reference dynamics, LQR-derived state feedback, dual PWM/DAC actuation,
and serial telemetry implemented on an STM32F10x microcontroller.

[Open the full engineering notebook](notebooks/stm32_propeller_swing_lqr.ipynb) &nbsp;&middot;&nbsp;
[Simulink monitor](models/read_data_SWING.slx)

</div>

## Full demonstration

https://github.com/user-attachments/assets/b8affc33-cc30-4c7a-bc1a-70199a0b5af0

The operator initializes the horizontal arm, changes the requested angle with
the rotary potentiometer, and demonstrates closed-loop tracking while the host
display updates in real time.

## The system

Two opposing propellers rotate an instrumented arm around a central pivot. A
potentiometer sets the requested angle, an encoder measures the arm position,
and the STM32 closes the loop at a nominal 20 ms sample interval.

<p align="center">
  <img src="assets/figures/07-controller-flowchart.png" alt="Closed-loop control architecture" width="900">
</p>

The firmware performs the following sequence on every control update:

1. Read the potentiometer command and encoder position.
2. Estimate angular velocity by finite difference.
3. Advance the second-order reference model.
4. Evaluate state feedback and feedforward.
5. Convert the signed control effort into complementary motor commands.
6. Transmit command and position data over UART.

## Quick reference

| Parameter | Implemented value |
|:--|:--|
| Firmware API | STM32F10x Standard Peripheral Library |
| Control interrupt | TIM7 |
| Nominal control period | 20 ms |
| Reference integration | Forward Euler, $T_s=0.02$ s |
| Command mapping | 12-bit ADC centered at 2048, approximately $\pm450$ counts |
| Feedback gain | $K=[2.0,\ 1.25,\ -3.75,\ -0.6]$ |
| Feedforward gain | $K_{ff}=4$ |
| PWM command range | 0 to 300 timer counts per motor |
| Analog command range | 0 to 3 V per motor |
| Telemetry | USART2, 9600 baud, 8N1, nominally every 20 ms |

## Controller

The measured open-loop response was approximated by

$$
G(s)=\frac{83.01}{s^2+13.88s+83.01}.
$$

The requested motion is shaped by the critically damped reference model

$$
F(s)=\frac{400}{s^2+40s+400}=\frac{20^2}{(s+20)^2}.
$$

The implemented controller uses

$$
u[k]=-K
\begin{bmatrix}x_1&x_2&x_{r1}&x_{r2}\end{bmatrix}^{T}
+K_{ff}\theta_d,
$$

with

$$
K=\begin{bmatrix}2.0&1.25&-3.75&-0.6\end{bmatrix},
\qquad K_{ff}=4.
$$

The gain is LQR-derived and was tuned on the physical apparatus. The complete
derivation, state-space models, discretization, calibration, actuator mappings,
and source-code correspondence are documented in the
[engineering notebook](notebooks/stm32_propeller_swing_lqr.ipynb).

## Experimental response

<p align="center">
  <img src="assets/figures/04-analog-closed-loop-response.jpg" alt="Analog closed-loop response" width="48%">
  <img src="assets/figures/06-pwm-closed-loop-response.jpg" alt="PWM closed-loop response" width="48%">
</p>

| Motor interface | Rise time | Peak time | Steady-state error |
|:--|--:|--:|--:|
| Analog | 2.610 s | 1.580 s | 24.618 counts (about $1.114^\circ$) |
| PWM | 2.065 s | 1.435 s | 68.322 counts (about $3.075^\circ$) |

These are reported experimental measurements. Raw samples are not available in
the repository, so the figures and metrics are preserved as evidence rather
than presented as newly reproduced results.

## Hardware interface

| Signal | STM32 peripheral | Pin(s) |
|:--|:--|:--|
| Motor PWM | TIM3 | PC6, PC7 |
| Analog motor command | DAC | PA4, PA5 |
| Quadrature encoder | TIM4 | PB6, PB7 |
| Command potentiometer | ADC1 | PA1 |
| Serial telemetry | USART2 TX | PA2 |

## Repository guide

| Path | Contents |
|:--|:--|
| [`src/`](src) | STM32 firmware implementation |
| [`include/`](include) | Firmware headers |
| [`notebooks/`](notebooks) | Primary engineering documentation and analysis |
| [`models/`](models) | MATLAB/Simulink serial telemetry viewer |
| [`assets/figures/`](assets/figures) | System, calibration, and response figures |

## Firmware map

| File | Responsibility |
|:--|:--|
| [`src/main.c`](src/main.c) | Peripheral initialization, zeroing sequence, and TIM7 setup |
| [`src/stm32f10x_it.c`](src/stm32f10x_it.c) | State updates, controller evaluation, motor mapping, and interrupt handlers |
| [`src/Motor.c`](src/Motor.c) | TIM3 PWM generation on PC6 and PC7 |
| [`src/DAC.c`](src/DAC.c) | Dual analog motor commands on PA4 and PA5 |
| [`src/encoder.c`](src/encoder.c) | TIM4 quadrature encoder interface on PB6 and PB7 |
| [`src/ADC.c`](src/ADC.c) | Potentiometer acquisition through ADC1 channel 1 on PA1 |
| [`src/uart.c`](src/uart.c) | USART2 and periodic TIM2 telemetry setup |
| [`src/timeclock.c`](src/timeclock.c) | TIM6-based elapsed-time counter |

## Requirements and integration

### Required for firmware integration

- An STM32F10x target project and a compatible C toolchain
- STM32F10x Standard Peripheral Library headers and sources
- Board-specific startup code, interrupt vector, linker script, and system-clock setup
- SWD/JTAG programmer and hardware wired to the interface table above

### Optional host visualization

- MATLAB and Simulink R2017a or a compatible release
- Instrument Control Toolbox
- A serial adapter connected to USART2 TX

### Bring-up sequence

1. Create a project for the exact STM32 target and add `src/` and `include/`.
2. Add the required STM32F10x peripheral-library modules and device headers.
3. Supply startup, linker, clock, and interrupt-vector configuration for the board.
4. Resolve the PA1 ADC/UART configuration conflict and verify all pin assignments.
5. Confirm the timer clock before relying on the nominal 20 ms control and telemetry periods.
6. Build and flash, hold the arm horizontal, and press the PA0 user button to zero the encoder.
7. Verify which motor interface is physically active; the interrupt currently updates both PWM and DAC outputs.
8. For host monitoring, open [`models/read_data_SWING.slx`](models/read_data_SWING.slx) and select the correct serial port.

## Telemetry

The controller emits one 10-byte frame per nominal telemetry interval:

```text
LF | u1 low | u1 high | u2 low | u2 high | angle low | angle high |
desired low | desired high | NUL
```

The four payload channels are transmitted as little-endian signed 16-bit
values in this order:

1. PWM motor command `u1`
2. PWM motor command `u2`
3. Encoder position `x[0]`
4. Requested position `theta_desired`

The supplied Simulink model displays these channels using its recorded COM1,
9600 baud, 8 data bits, no parity, one stop bit, little-endian configuration.
Change COM1 to the port assigned by the host operating system.

## Build status

> [!IMPORTANT]
> This repository contains the application firmware, not a complete standalone
> STM32 build project. Startup code, linker configuration, clock configuration,
> target project files, and the full STM32F10x vendor library must be supplied
> by the integrating project.

No CI build badge is shown because the repository does not contain enough
target-specific material to produce a meaningful standalone build.

## Known limitations

- PA1 is configured both as the potentiometer ADC input and as the USART2 RX
  GPIO, even though USART2 is initialized in transmit-only mode.
- The pinout comment in `main.c` does not fully match the peripheral setup in
  the implementation; the tables in this README follow the executed code.
- TIM2 telemetry sends all ten bytes using blocking flag polling inside an
  interrupt configured at the same priority as the TIM7 control interrupt.
- Passing exactly 3 V to `DAC_Motor_Drive()` converts 65536 to `uint16_t` and
  wraps the full-scale command to zero.
- Both PWM and DAC drive functions run during every control update; the active
  physical motor interface must be selected and verified externally.
- The nominal timing depends on a clock configuration that is not included in
  the repository.
- The final LQR synthesis script and complete $Q$ and $R$ matrices are absent,
  so the implemented gain can be inspected but not regenerated exactly.

The historical C implementation is intentionally preserved unchanged. See the
[engineering notebook](notebooks/stm32_propeller_swing_lqr.ipynb) for the full
derivation, evidence trail, and source-level analysis.

## License

This project is released under the [MIT License](LICENSE).
