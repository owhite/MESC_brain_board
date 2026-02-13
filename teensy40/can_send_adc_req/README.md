## Teensy CAN Test Programs

**Teensy sketches** that can be loaded to send commands over CAN to the MESC-based ESC.  

---

### Debugging in STM32CubeIDE
If you want to trace the command as it flows through the ESC firmware, good debug breakpoints are:  

- **`CAN1_RX0_IRQHandler()`** → Confirms the packet arrived in the ISR and was queued.  
- **`TASK_CAN_packet_cb()`** → Confirms the packet was dispatched to the right handler (ADC1_2_REQ or IQREQ).  
- **`MESCinput_Collect()`** → Confirms the chosen input value (CAN vs UART vs ADC) is being forwarded into the motor control loop.  

