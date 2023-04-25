# Qorvo_UWB_Linux
Qorvo UWB Module + RPi 4 (Any Linux based Host)





# UWB Module Example Code

## Project Description

The main goal of this open-source project is to create an example code for using the Qorvo DWM3000EVB Ultra-Wideband (UWB) module with a Linux-based host system, specifically the Raspberry Pi 4. The project aims to provide a clear and concise example of how to interface with the DWM3000EVB module using the Raspberry Pi 4's SPI interface, and how to perform basic UWB operations such as ranging and positioning.

The project will be implemented in C programming language and will be open-sourced under the MIT License. The example code will be divided into several modules, each responsible for a specific task, such as initializing the DWM3000EVB module, configuring UWB parameters, and performing UWB ranging operations.

This project aims to provide a valuable resource for developers who are interested in using UWB technology for ranging and positioning applications, and who are looking for a clear and concise example of how to interface with the DWM3000EVB module using a Linux-based host system. The project is open to contributions from the community, and we encourage developers to share their feedback, suggestions, and code contributions to help improve the project.

## Getting Started

To get started with this project, you will need the following hardware and software:

- Qorvo DWM3000EVB module
- Raspberry Pi 4 or compatible Linux-based host system
- Compatible SPI interface and wiring connections between the DWM3000EVB module and the Raspberry Pi 4
- C compiler and development environment for the Raspberry Pi 4

Once you have the necessary hardware and software, you can clone the project repository and build the example code using the provided Makefile. The example code can be run on the Raspberry Pi 4, and should provide a basic demonstration of how to use the DWM3000EVB module for UWB ranging and positioning.

# UWB Module Example Code

| Example Code | Description | Status |
|--------------|-------------|--------|
| ex_uwb_reading_dev_id | Read device ID from the DWM3000EVB module | Working and tested |
| ex_uwb_simple_tx | Simple transmission using the DWM3000EVB module | Under development |
| ex_uwb_tx_sleep | Transmission with sleep mode enabled | Under development |
| ex_uwb_tx_sleep_auto | Transmission with automatic sleep mode enabled | Under development |
| ex_uwb_tx_timed_sleep | Transmission with timed sleep mode enabled | Under development |
| ex_uwb_tx_with_cca | Transmission with Clear Channel Assessment (CCA) enabled | Under development |
| ex_uwb_simple_tx_sts_sdc | Simple transmission with STS/SDC enabled | Under development |
| ex_uwb_simple_tx_pdoa | Simple transmission with PDOA enabled | Under development |
| ex_uwb_simple_tx_aes | Simple transmission with AES encryption enabled | Under development |
| ex_uwb_simple_rx | Simple reception using the DWM3000EVB module | Under development |
| ex_uwb_rx_diagnostics | Reception with diagnostic information enabled | Under development |
| ex_uwb_rx_sniff | Reception with sniff mode enabled | Under development |
| ex_uwb_rx_with_crystal_trim | Reception with crystal trimming enabled | Under development |
| ex_uwb_simple_rx_sts_sdc | Simple reception with STS/SDC enabled | Under development |
| ex_uwb_simple_rx_pdoa | Simple reception with PDOA enabled | Under development |
| ex_uwb_simple_rx_aes | Simple reception with AES decryption enabled





The main goal of this open-source project is to provide an example code for using the Qorvo DWM3000EVB module with a Linux-based host system. Specifically, we are using a Raspberry Pi 4 as the host system and connecting it to the DWM3000EVB module via the SPI interface.


## Wiring Connections

The following table shows the wiring connections between the DWM3000EVB module and the Raspberry Pi 4:

| DWM3000 Pin | DWM3000 Function | Raspberry Pi 4 Pin | Raspberry Pi 4 Function |
|-------------|-----------------|--------------------|--------------------------|
| 1           | GND             | 6                  | Ground                   |
| 2           | VDD             | 1                  | 3.3V Power               |
| 3           | MOSI            | 19 (GPIO 10)       | SPI MOSI                  |
| 4           | MISO            | 21 (GPIO 9)        | SPI MISO                  |
| 5           | SCLK            | 23 (GPIO 11)       | SPI Clock                 |



## License

This project is open-sourced under the MIT License. See the LICENSE file for more information.
