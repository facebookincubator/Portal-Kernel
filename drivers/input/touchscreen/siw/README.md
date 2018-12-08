# touch
SiW Touch Driver

This driver supports Silicon-works Touch Device.

Test platform : Odroid-xu4 (Exynos5422, Android 4.4.4, Kernel v3.10.9)

Author:

kimhh@siliconworks.co.kr

parksy5@siliconworks.co.kr

# Recommended base folder

: {kernel top}/input/touchscreen/siw


List of supported devices

: LG4894, LG4895, LG4946, LG4951, SW1828, SW49407, SW42101


# for Built-in

{kernel top}/input/touchscreen/siw $ mv Kconfig_builtin Kconfig

{kernel top}/input/touchscreen/siw $ mv Makefile_builtin Makefile


Add this 1 line into {kernel top}/input/touchscreen/Kconfig

: source "drivers/input/touchscreen/siw/Kconfig"

Add this 1 line into {kernel top}/input/touchscreen/Makefile

: obj-$(CONFIG_TOUCHSCREEN_SIW) += siw/


# for Module test

{kernel top}/input/touchscreen/siw $ mv Makefile_module Makefile

{kernel top}/input/touchscreen/siw $ make


You can choose the device type in this Makefile(Makefile_module)

CONFIG_TOUCHSCREEN_SIW_LG4894=y   //LG4894 selected

CONFIG_TOUCHSCREEN_SIW_LG4895=n

CONFIG_TOUCHSCREEN_SIW_LG4946=n

CONFIG_TOUCHSCREEN_SIW_LG4951=n

CONFIG_TOUCHSCREEN_SIW_SW1828=n

CONFIG_TOUCHSCREEN_SIW_SW49407=n

CONFIG_TOUCHSCREEN_SIW_SW42101=n

# for DTS

See '_reference / device_tree'


# git

[clone]

$ git clone https://github.com/siw-touch/touch.git .

[push (id/pw required)]

$ git push origin master

