# Shuttle Car Gripper Control on STM32F103C8T6 MCU

# Clock Configuration
* SYSCLK: 72 MHz
* Use 8 MHz High Speed External (HSE) clock


# GPIO Configuration
* PC13:
  * User Label: LED
  * GPIO output level: Low
  * GPIO mode: Output Push Pull
  * GPIO Pull-up/Pull-down: No pull-up and pull-down
  * Maximum output speed: Low
* PC14:
  * User Label: Left_Direction
  * GPIO output level: Low
  * GPIO mode: Output Push Pull
  * GPIO Pull-up/Pull-down: No pull-up and pull-down
  * Maximum output speed: Low
* PC15:
  * User Label: Left_Direction
  * GPIO output level: Low
  * GPIO mode: Output Push Pull
  * GPIO Pull-up/Pull-down: No pull-up and pull-down
  * Maximum output speed: Low

# Timer Configuration
* TIM2
  * PWM Channel 1
  * PWM Channel 3
  * Frequency: 20 kHz
* TIM3
  * PWM Channel 1: Input Capture
  * PWM Channel 2: Input Capture
* TIM4
  * Frequency: 50 Hz


# UART Configuration
* USART1
  * Mode: Asynchronous
  * Pins
    * PA9: USART1_TX
    * PA10: USART1_RX
  * Configuration
    * Baud Rate: 9600 Bits/s
    * Word Length: 8 Bits (including Parity)
    * Parity: None
    * Stop Bits: 1
    * Data Direction: Receive and Transmit

 
# Independent Watchdog Configuration
* Use 40 kHz LSI RC clock 
* Timeout: 1 second
