Programming
===========
1. Plug in aqn ST-Link V2 programmer. Wire 1:1 to 3v, SWD, Clk and GND.
2. Place both boot jumpers on the G side. They can stay one that side.
3. Click the upload button in vscode.
4. Click the reset button or power cycle to run the program.
5. Connect a TTL USB/Serial adapter if you want to see debugging text.

FluidNC UART Conection
======================
PA2 (TX2)
PA3 (RX2)

Configuration
=============
uart1:
  txd_pin: gpio.4
  rxd_pin: gpio.26
  baud: 921600
  mode: 8N1

uart_channel1:
  uart_num: 1
  report_interval_ms: 75
  message_level: None