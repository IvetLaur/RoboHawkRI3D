# BIOBUZZ Command-Based TeleOp — Mecanum + Intake + Nectar & Pollen Shooters

Command-based robot code for the 2026-27 FTC game **BIOBUZZ**, with a PIDF +
battery-voltage-compensated flywheel controller, a live **Shooter Tune** mode, and a
Limelight 3A vision subsystem that targets **HIVE CELL AprilTag clusters**.

**No outside libraries.** The command framework (`command/` folder) is our own ~250 lines,
using the same WPILib-style pattern FRC and many FTC teams use. Nothing to add to Gradle.

## 1. Install

Copy the `TeamCode/.../teamcode/` folders into your FtcRobotController project (SDK 12.0+
recommended for BIOBUZZ) and build. That's it.

## 2. Robot Configuration (Driver Hub)

| Name in config | Device | Port idea |
|---|---|---|
| `frontLeft` `frontRight` `backLeft` `backRight` | drive motors | Control Hub motor 0–3 |
| `intake` | intake motor | Expansion Hub motor 0 |
| `nectarShooter` | flywheel motor **with encoder cable** | Expansion Hub motor 1 |
| `pollenShooter` | flywheel motor **with encoder cable** | Expansion Hub motor 2 |
| `imu` | built-in IMU | (already there) |
| `limelight` | Limelight 3A (Ethernet device) | optional — code runs without it |

Names are case-sensitive. Change them in `Constants.Hardware` if yours differ.

## 3. Files

```
teamcode/
├── Constants.java                    every name and number, one place
├── command/                          OUR command framework (no library)
│   ├── Command.java                  initialize / execute / isFinished / end
│   ├── CommandBase.java              extend this for new commands
│   ├── InstantCommand.java           do one thing once
│   ├── RunCommand.java               do one thing every loop
│   ├── Subsystem.java, SubsystemBase.java
│   ├── CommandScheduler.java         the loop: buttons → subsystems → commands → defaults
│   ├── Trigger.java                  onTrue / whileTrue / toggleOnTrue bindings
│   └── CommandOpMode.java            base OpMode, always stops everything at STOP
├── util/
│   ├── Alliance.java                 RED / BLUE
│   └── LookupTable.java              distance → RPM interpolation
├── subsystems/                       "the robot's parts" — own the hardware
│   ├── MecanumDriveSubsystem.java    mecanum math, field-centric, IMU
│   ├── IntakeSubsystem.java          in / out / stop
│   ├── ShooterSubsystem.java         PIDF + voltage comp (used twice)
│   ├── ShooterGains.java             kV kS kP kI kD + default RPM
│   └── VisionSubsystem.java          Limelight → HIVE CELL clusters
├── commands/                         "actions" — use subsystems
│   ├── DriveCommand.java             joystick driving (default command)
│   ├── IntakeCommand.java            run intake while held
│   ├── SpinShooterCommand.java       flywheel on until cancelled
│   ├── VisionRpmShooterCommand.java  RPM from distance to our upright CELL
│   └── AimAtCellCommand.java         auto-turn to our upright CELL
└── opmodes/
    ├── MainTeleOp.java               "BioBuzz TeleOp" (competition)
    └── ShooterTuneOpMode.java        "Shooter Tune" (live PIDF tuning)
```

## 4. How command-based works (the 2-minute version)

Think of a restaurant.

- **Subsystems are the kitchen stations** (grill, fryer). Each owns its equipment. Only
  the grill touches the grill. In code: only `IntakeSubsystem` touches the intake motor.
- **Commands are the orders** ("spin the nectar shooter"). An order says which stations
  it needs (`addRequirements(...)`).
- **The CommandScheduler is the head chef.** Every loop it checks the buttons, starts new
  orders, runs the active ones, and makes sure two orders never fight over the same
  station. If a new order needs a busy station, the old order is stopped first.
- **A default command** is what a station does when nobody ordered anything. The
  drivetrain's default is `DriveCommand`, so the sticks always work — until you hold
  auto-aim, which takes over the drivetrain, then hands it back when you let go.

Every command has the same life cycle:

| Method | When it runs | Example |
|---|---|---|
| `initialize()` | once, at start | `shooter.spinUp()` |
| `execute()` | every loop | read sticks, set drive power |
| `isFinished()` | every loop — return `true` to stop | (ours run until cancelled) |
| `end(interrupted)` | once, at stop | `shooter.stop()` |

Button bindings use `Trigger` (names match WPILib, so FRC docs and videos apply):

| Binding | Meaning | Example |
|---|---|---|
| `onTrue` | start on press | `new Trigger(() -> gamepad1.y).onTrue(new InstantCommand(drive::toggleFieldCentric))` |
| `whileTrue` | run while held, `end()` on release | intake, auto-aim |
| `toggleOnTrue` | press = start, press again = cancel | shooters |

The scheduler checks each Trigger once per loop and compares with last loop, so a
press only counts once — no `wasPressed` booleans scattered through the code.

## 5. Controls

**During INIT:** gamepad 1 d-pad ← BLUE / → RED alliance.

| Gamepad 1 (driver) | | Gamepad 2 (operator) | |
|---|---|---|---|
| Left stick | drive + strafe | Right trigger | intake in |
| Right stick X | turn | Left trigger | intake out |
| Right bumper (hold) | slow mode | A | nectar on/off |
| Left bumper (hold) | auto-aim at our upright CELL | B | pollen on/off |
| Y | field / robot-centric | Right bumper | nectar on/off, RPM from cell distance |
| Back | reset heading | Left bumper | pollen on/off, RPM from cell distance |
| | | X | stop both shooters |

## 6. The flywheel controller

Runs inside `ShooterSubsystem.periodic()` every loop:

```
power = kV·target + kS + kP·error + kI·∫error + kD·Δerror
power = power × (12 V ÷ battery voltage)        ← voltage compensation
```

- **Feedforward (kV, kS)** does most of the work: "this RPM needs about this much power."
- **Feedback (kP, kI, kD)** fixes what's left — mostly the speed drop after each shot.
- **Voltage compensation** keeps shots the same on a fresh 13.5 V battery and a tired 12 V one.
- **Integral zone**: kI only acts within 300 RPM of target, so it doesn't wind up during spin-up.
- **Filter**: measured RPM is smoothed because raw encoder velocity is noisy.

## 7. Shooter Tune mode

Run **"Shooter Tune"** (group *Tuning*). Everything is on gamepad 1:

| Button | Does |
|---|---|
| D-pad ← / → | pick nectar / pollen |
| A | wheel on/off |
| Y | next setting (RPM → kV → kS → kP → kI → kD) |
| D-pad ↑ / ↓ | change setting |
| X | step size ×1 → ×10 → ×0.1 |
| Right trigger | feed with intake (test shot) |
| Back | clear shot stats |
| Sticks | drive (always slow) |

**Recipe:**
1. kP = kI = kD = 0. Spin up.
2. Adjust **kV** until actual ≈ target on its own. The **kV guess** line computes it for you.
3. Add **kS** only if low speeds struggle.
4. Raise **kP** while firing test shots. Watch **dip** and **recovery ms** — you want short
   recovery without RPM bouncing above target. If it oscillates, back off ~30%.
5. Tiny **kI** only for a stubborn steady error. kD rarely needed.
6. Copy the **"Paste into Constants"** line into `Constants.Shooter.NECTAR` / `POLLEN`.
   Tune-mode changes are lost when the robot restarts.

**Distance table:** with an upright HIVE CELL in view, drive to 5–6 spots, find the RPM
that lands in the CELL, note the shown distance, and fill `LUT_DISTANCE_IN` / `*_LUT_RPM`.

## 8. Vision — BIOBUZZ AprilTag clusters (not last season's single tags)

**What changed.** In DECODE each goal had one AprilTag and never moved: find tag N, aim.
In BIOBUZZ:

- Each HIVE has two CELLS. Each CELL has a **cluster of 4 tags** (36h11, 3.25 in) on its
  underside, facing the tiles. Any one of the 4 identifies the CELL.
- The HIVE is **bi-stable and flips** when it tips, so which CELL you can score in changes
  during the match. You can't hard-code a target tag.
- Only the **upright** CELL (opening facing up) is scorable.

**What the code does every loop** (`VisionSubsystem.periodic()`):

1. Read every tag the Limelight sees.
2. Group them into the 4 CELL clusters by ID.
3. Average the visible tags per CELL → steadier aim angle (`tx`), distance and roll.
4. Keep our alliance's CELLS (`RED…` / `BLUE…`).
5. Keep the **upright** one (|roll| < 90°) — FIRST's recommended check.

This mirrors the FTC SDK 12 `AprilTagClusterDetection` (alliance check on the cluster name,
upright check on roll), done by hand because the Limelight reports tags individually.

| CELL cluster | Tag IDs |
|---|---|
| RED SCORING | 30–33 |
| RED AUDIENCE | 34–37 |
| BLUE AUDIENCE | 38–41 (confirmed in manual text) |
| BLUE SCORING | 42–45 |

**Before it's competition-ready:**
- [ ] Check every ID against **manual Figure 9-17**. Only 38–41 was readable in the text I
      could access; the rest follow the SDK's 30–45 numbering.
- [ ] Limelight web UI (`http://limelight.local:5801`): pipeline 0 = AprilTag, family
      **36h11**, tag size **82.55 mm** (3.25 in). Without the size there's no distance or roll.
- [ ] The tags face **down** — mount the camera so it can look **up** at the CELL undersides
      from where you shoot.
- [ ] **Calibrate roll:** point at an upright CELL, read its roll on telemetry, and set
      `ROLL_OFFSET_DEG` so upright reads near 0. Then check a flipped CELL reads above 90.
- [ ] Tune `AIM_KP` / `AIM_MIN_TURN`, then fill the distance → RPM table.

Telemetry lists every visible CELL with % of tags found, UP/down, angle, distance and roll,
plus raw tag IDs — so you can see exactly what the camera thinks.

## 9. First-run checklist

1. Robot on blocks. Push left stick forward — all four wheels should spin "forward".
   Wrong? Change the `setDirection` lines in `MecanumDriveSubsystem`.
2. Strafe right — check the robot actually goes right on the floor.
3. Flywheel spins the wrong way? Pass `true` for `reversed` when creating that shooter.
4. Shooter RPM reads 0 while spinning? The encoder cable isn't plugged in, or
   `TICKS_PER_REV` is wrong.
5. Field-centric drifts? Check the IMU orientation (logo/USB direction) in `MecanumDriveSubsystem`.
