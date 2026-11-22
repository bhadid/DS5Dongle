# Diagnostic firmware

> [!WARNING]
> **Serial logs from this firmware contain Bluetooth pairing keys and device addresses.**
> Don't post or commit them. Don't use this firmware for everyday play.

Build with `-DDIAG_LOG=ON -DENABLE_SERIAL=ON`, flash it, then log with:

```powershell
powershell -ExecutionPolicy Bypass -File tools\latency\serial-log.ps1 COMx logs\pico.log 600
```

Boot waits up to 20 seconds for the logger to attach. To send a radio command, write it to `<logfile>.cmd`.
The commands are listed above `diag_radio_command` in `src/bt.cpp`.
