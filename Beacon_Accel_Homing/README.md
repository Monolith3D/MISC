# Beacon Accelerometer Homing

This plugin has moved. It is now maintained by FrankRamm as
[Accelerometer Homing](https://github.com/FrankRamm/accel_homing), which adds
support for other accelerometers, calibration, accuracy checks and Moonraker
updates.

This folder is no longer maintained and will not receive updates. New installs
should use the [new repository](https://github.com/FrankRamm/accel_homing).

## Existing installs

Follow the
[migration instructions](https://github.com/FrankRamm/accel_homing/blob/main/docs/INSTALLATION.md#migrate-from-the-misc-plugin).
They rely on this folder's uninstaller, so the old plugin stays here:

```
~/Monolith_MISC/Beacon_Accel_Homing/install.sh -u
```

Before that, comment out the `beacon_accel` endstop lines, restore your old
endstop and sensorless homing lines, and remove the `[beacon_accel_homing]`
section. Afterwards, remove the `[update_manager beacon_accel_homing]` section
from `moonraker.conf`, delete `~/Monolith_MISC`, and restart Klipper and
Moonraker.

## License

GPLv3, like Klipper. See [LICENSE](LICENSE).
