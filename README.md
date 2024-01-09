# Shuttle Car Gripper Control on STM32F103C8T6 MCU

# Clock Configuration
* SYSCLK: 72 MHz
* Use 8 MHz High Speed External (HSE) clock


# GPIO
* PA1:
  * User Label: Left_Direction
  * GPIO output level: Low
  * GPIO mode: Output Push Pull
  * GPIO Pull-up/Pull-down: No pull-up and pull-down
  * Maximum output speed: Low
* PA7:
  * User Label: Right_Direction
  * GPIO output level: Low
  * GPIO mode: Output Push Pull
  * GPIO Pull-up/Pull-down: No pull-up and pull-down
  * Maximum output speed: Low
* PB0:
  * User Label: Left_Feedback
  * GPIO mode: External Interrupt Mode with Rising edge trigger detection
  * GPIO Pull-up/Pull-down: No pull-up and pull-down
* PB1:
  * User Label: Right_Feedback
  * GPIO mode: External Interrupt Mode with Rising edge trigger detection
  * GPIO Pull-up/Pull-down: No pull-up and pull-down
* PB12:
  * User Label: Forward_Right
  * GPIO mode: Input mode
  * GPIO Pull-up/Pull-down: Pull-up
* PB13:
  * User Label: Forward_Left
  * GPIO mode: Input mode
  * GPIO Pull-up/Pull-down: Pull-up
* PB14:
  * User Label: Through_Sensor
  * GPIO mode: Input mode
  * GPIO Pull-up/Pull-down: Pull-up
* PC13:
  * User Label: LED
  * GPIO output level: Low
  * GPIO mode: Output Push Pull
  * GPIO Pull-up/Pull-down: No pull-up and pull-down
  * Maximum output speed: Low


# Timer Configuration
* TIM2
  * PWM Channel 1
  * Frequency: 20 kHz
* TIM3
  * PWM Channel 1
  * Frequency: 20 kHz
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
