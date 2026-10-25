# Beacon Accelerometer Homing

Klipper and Kalico extension that homes X and Y by detecting the bump of the
toolhead against the stop with the Beacon accelerometer, instead of
sensorless homing. One touch per home, no stall tuning.

> [!WARNING]
> **Experimental.** This plugin was completely vibe-coded (written by AI
> coding agents). It works on three printers so far. Use it at your own risk.
> We take no responsibility for any damage to your printer or anything else.
> Keep a hand on the emergency stop for your first homes.

## Status

- Working on three printers so far, including those of FrankRamm and
  Ingstronmentalist.
- Requires a Beacon RevH with [beacon_klipper](https://github.com/beacon3d/beacon_klipper).
- If the toolhead does not move when a home starts, the plugin takes it as
  already against the stop. A jammed axis away from the stop would then home
  in the wrong place.
- Stepper drivers on more than one MCU are untested.
- Homing speeds below about 50 mm/s are too soft to detect reliably.

## Installation

Over SSH on the printer:

```
cd ~
git clone --filter=blob:none --sparse https://github.com/Monolith3D/MISC.git Monolith_MISC
git -C Monolith_MISC sparse-checkout set Beacon_Accel_Homing
./Monolith_MISC/Beacon_Accel_Homing/install.sh
```

This downloads only this folder of the MISC repository and links the plugin
into Klipper. For updates, add this to `moonraker.conf` (the install script
offers to do it):

```ini
[update_manager beacon_accel_homing]
type: git_repo
path: ~/Monolith_MISC
origin: https://github.com/Monolith3D/MISC.git
primary_branch: main
managed_services: klipper
```

## Configuration

Add the section. Nothing in it is required. [example.cfg](examples/beacon_accel_homing.cfg)
explains the options.

```ini
[beacon_accel_homing]
#x_threshold: 8.0
#y_threshold: 8.0
```

`x_threshold` and `y_threshold` (8 g by default) set how hard the toolhead
has to hit the X or Y stop before homing triggers:

- **Higher g** needs a stronger hit. Movement vibration is less likely to
  trigger it, but a soft hit can be missed. The toolhead then keeps pushing
  until the travel limit stops homing with an error.
- **Lower g** reacts to softer hits, but vibration during the move can
  trigger it early and home in the wrong place.

Leave them commented out unless `BEACON_ACCEL_HOMING_STATUS` shows a reason
to change them. It lists the last homes with the impact and the movement
noise per axis.

Select the plugin as the X and Y endstop, commenting out the old lines:

```ini
[stepper_x]
#endstop_pin: <your old X endstop pin>
endstop_pin: beacon_accel:x
homing_speed: 150
homing_retract_dist: 0

[stepper_y]
#endstop_pin: <your old Y endstop pin>
endstop_pin: beacon_accel:y
homing_speed: 150
homing_retract_dist: 0
```

Comment out the sensorless homing leftovers: `diag0_pin` / `diag1_pin` and
`home_current` in the X/Y TMC sections, `homing_accel` and `min_home_dist` on
Kalico, and any current or move changes in a sensorless `homing_override`.
Restart Klipper.

## Uninstall

Comment out the `beacon_accel` endstop lines and uncomment your old endstop
and sensorless homing lines (`diag` pins, `home_current`, homing macro).
Remove the `[beacon_accel_homing]` section, then run:

```
~/Monolith_MISC/Beacon_Accel_Homing/install.sh -u
```

Remove the `[update_manager beacon_accel_homing]` section from
`moonraker.conf`, delete `~/Monolith_MISC`, and restart Klipper and Moonraker.

## Credits

Tested by FrankRamm and Ingstronmentalist.

## License

GPLv3, like Klipper. See [LICENSE](LICENSE).
