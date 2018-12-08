
* SiW Mon

---------------------------------------------------------------------------------------------------

- This driver supports the monitoring for bus, event data of SiW Touch Driver

---------------------------------------------------------------------------------------------------
[Build(module build)]
- Make mon folder under SiW Touch Driver folder
- Build SiW Touch Driver first under 'CONFIG_TOUCHSCREEN_SIWMON = y' condition
  : See siw_touch.h
  : Module.symvers of Touch Driver rquired
- Build mon driver
  : Check siwmon.ko

---------------------------------------------------------------------------------------------------
[Execution order]

$ insmod sxxxxxx.ko	//Touch driver shall be loaded first
$ insmod siwmon.ko

$ cat /sys/kernel/debug/siwmon/all	//Show all mon events
$ cat /sys/kernel/debug/siwmon/bus	//Show bus(I2C, SPI, ...) data
$ cat /sys/kernel/debug/siwmon/evt	//Show touch input report data
$ cat /sys/kernel/debug/siwmon/ops	//Show operation step info
(ctrl+c for stop)

* [Caution]
  For cat command, use SSH terminal rather than UART terminal

  Because!!! - I
  Even after stop cat via ctrl+c, the access count for file(all, bus, evt, ops) still remains not-cleared state
  and the lock state will be cleared just after terminal diesconnection.
  
  Clear(X) -> rmmod siwmon.ko failed -> rmmod sxxxxxx.ko failed(due to module dependency)

  Because!!! - II
  The amount for print log is big because it shows all data
  and UART terminal has low speed and it can cause considerable system load.
  So the SSH terminal is recommended for its faster comm. speed.
  In addition, it's difficult to parse the mon log because it's shuffled with kernel log
  under UART terminal.

---------------------------------------------------------------------------------------------------

---------------------------------------------------------------------------------------------------

---------------------------------------------------------------------------------------------------

---------------------------------------------------------------------------------------------------

---------------------------------------------------------------------------------------------------

