# STM32 Embedded Games & Utilities Project

## 🧩 Overview

This embedded systems project for the STM32 microcontroller implements a suite of interactive mini-games and utilities. It leverages several hardware peripherals: timers, ADC, UART, DAC, GPIOs, an ultrasonic sensor, and an IR remote receiver using the NEC protocol. Communication and control are provided through buttons, serial UART interface, and an IR remote.

---

## 🎮 Features

### ⚡ Game 1: Reaction Time

- A two-player reflex challenge.
- After a random delay, an LED lights up.
- Players press a button as quickly as possible.
- The winner is the fastest responder.
- Time is captured using **TIM4 input capture**.
- The winner’s time is printed via **UART**.

---

### ⏱️ Game 2: Countdown Challenge

- Players must wait until a countdown reaches zero before pressing a button.
- Countdown speed depends on **potentiometer** value read using **ADC**:
  - Easy (2s): ADC < 1330
  - Normal (1s): 1330 ≤ ADC < 2600
  - Hard (0.5s): ADC ≥ 2600
- Players are penalized for pressing early (before zero).
- Melody feedback is generated based on performance using **PWM signals** via **TIM2 and TIM3**.
- Player reaction times (positive or negative) are printed via **UART**.

---

### 🪙 Game 3: Greed Island (2D Console Game)

- A text-based adventure on a 20x20 grid, displayed via UART.
- Player moves using **IR remote control**.
- Collect gold coins and avoid traps or obstacles.
- Features simple interaction mechanics like keys, padlocks, HP system, and victory/loss messages.

---

### 📏 Ultrasonic Distance Trigger

- Game selection is activated by moving your hand near an **ultrasonic sensor**.
- Distance range: **1 cm to 7 cm**.
- Implements timing capture using **TIM2** for echo pulse analysis.

### 👇 How It Works

Using **Timer 2 (TIM2)** and **GPIOs**:

- The sensor emits a 10μs pulse via **PA1**.
- The **echo pin (PB10)** captures the reflected signal.
- **TIM2 Channel 3** is configured for input capture on rising and falling edges.
- Time between edges gives the pulse duration.

\[
\text{Distance} = \frac{\text{Pulse Duration} \times \text{Speed of Sound}}{2}
\]

- The speed of sound used: `0.0343 cm/μs`.

---

## 📡 NEC Protocol — IR Remote Control

The **NEC protocol** is used for decoding commands from a standard IR remote control.

### Key Concepts:
- NEC sends a **32-bit frame**:
  - 8 bits: address
  - 8 bits: logical inverse of address
  - 8 bits: command
  - 8 bits: logical inverse of command
- Each bit is identified using pulse length:
  - Logic `1`: 562.5μs pulse + 1687.5μs space
  - Logic `0`: 562.5μs pulse + 562.5μs space

### STM32 Integration:
- An IR receiver is connected to **PB11 (EXTI line)**.
- **TIM11** is used to measure pulse intervals.
- Falling edges trigger an interrupt, and pulse duration is checked:
  - >1700μs → logic `1`
  - 1000–1700μs → logic `0`
- After 32 bits are received:
  - If command and its logical inverse are valid, the decoded **`cmd`** is saved.
  - Mapped to directional movement in **Game 3**.

---

## 🧭 UART Communication

- UART (USART2) is used for:
  - Printing game menus and instructions.
  - Displaying real-time game states (coordinates, map, results).
  - Showing ADC values, distance measurements, and reaction times.

### Configuration:
- Baud rate: **115200**
- Transmission via `HAL_UART_Transmit`
- Reception handled with **interrupts (`HAL_UART_RxCpltCallback`)** for responsiveness.

### Special Commands:
- Sending `X` over UART resets the system using `NVIC_SystemReset()`.

---

## 🧪 I2C (Prepared for Future Use)

Although I2C is initialized (`MX_I2C1_Init()`), it is **not actively used** in the current game logic. It is configured for:
- 400kHz Fast Mode
- 7-bit addressing
- Ready for expansion (e.g., adding an OLED display, EEPROM, or additional sensors).

This provides a base for future improvements or hardware extensions.

---

## 🔧 Hardware Peripherals Used

| Peripheral | Description |
|------------|-------------|
| **GPIO**   | LEDs, push buttons, ultrasonic trigger/echo, IR input |
| **TIM2**   | PWM melody generation, ultrasound timing, delays |
| **TIM3**   | Tone timing (for melody note durations) |
| **TIM4**   | Input capture for reaction time game |
| **TIM9**   | Delay timing (software delay routine) |
| **TIM11**  | IR timing base for NEC decoding |
| **ADC**    | Potentiometer input to set countdown difficulty |
| **DAC**    | Initialized, but not used in current version |
| **UART2**  | Serial communication with PC |
| **I2C1**   | Configured, ready for future peripheral support |
| **EXTI**   | Used for PC13 (user button) and PB11 (IR receiver) interrupts |

---

## ▶️ Getting Started

1. Flash the code to your STM32 board using STM32CubeIDE or ST-Link.
2. Connect a UART terminal (e.g., Tera Term, PuTTY) at **115200 baud**.
3. Place your hand over the ultrasonic sensor to trigger the menu.
4. Use:
   - On-board push buttons for Games 1 & 2.
   - IR remote control for Game 3.

---

## 📌 Notes

- UART is the main interface for feedback and debugging.
- Button presses are interrupt-driven for accuracy.
- The IR and ultrasonic systems run with **precise timing via timers and interrupts**.
- Code is modular and ready for further extensions (I2C, DAC, OLED, etc.).

---
