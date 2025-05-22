# free_rtos_stm32f0_public
example using free rtos on the stm32 f091RC
v 0.0.1
user IRS to read button input and convert to moris code. sends message to task which read it from a circular buffer and sends it to the computer over uart.
uart settings:
baud: 115200, size: 8 bit, stop bit: 1, no parity bit.
core/main.c
