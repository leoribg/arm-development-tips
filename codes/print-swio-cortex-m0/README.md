# Printf Cortex M0

## Table of Contents

- [About](#about)
- [Getting Started](#getting_started)
- [Usage](#usage)
- [Contributing](../CONTRIBUTING.md)

## About <a name = "about"></a>

Print debug messages over OpenOCD Debuuger (SWIO).

## Steps <a name = "steps"></a>

 - Go to project debug configuration

<img src=".\images\print_openocd_1.png" width="512">

- Change the Debug probe to OpenOCD

<img src=".\images\print_openocd_2.png" width="512">

- Mention the semi-hosting run command on Startup tab

```
monitor arm semihosting enable
```

<img src=".\images\print_openocd_3.png" width="512">

- Go to project Properties>C/C++ Build>Settings>MCU GCC Linker>Miscellaneous and mention the linked arguments:

```
-specs=rdimon.specs -lc -lrdimon
```

<img src=".\images\print_openocd_4.png" width="512">

- Remove syscalls.c file from debug configuration

<img src=".\images\print_openocd_5.png" width="512">

- Call the initialise_monitor_handles() function before any printf, as shown in [main.c](./main.c) file.

- Do not forget "\n" 

<img src=".\images\print_openocd_6.png" width="700">