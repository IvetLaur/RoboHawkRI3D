# RoboHawk RI3D — Code Strategy (BIOBUZZ 2026-27)

Built from the `biobuzz-teleop.zip` command-based TeleOp. The full original guide is in
`BIOBUZZ_README.md` next to this file.

## 1. What we have

A command-based TeleOp with **no outside libraries**, written for SDK 12.0 (our project is on 12.0):

| Layer | Folder | Job |
|---|---|---|
| Framework | `command/` | Scheduler, Trigger (button bindings), Command/Subsystem base classes |
| Settings | `Constants.java` | Every hardware name and tuning number, in one place |
| Robot parts | `subsystems/` | Mecanum drive (field-centric + IMU), intake, 2× PIDF flywheel shooters, Limelight vision |
| Actions | `commands/` | Drive, intake, spin shooter, distance-based RPM, auto-aim |
| Programs | `opmodes/` | **RoboHawk TeleOp** (competition), **RoboHawk Shooter Tune** (live PIDF tuning) |

Rule of thumb: **only a subsystem touches its own motors. Commands ask subsystems to act.
The OpMode only creates subsystems and binds buttons.**

## 2. RI3D game plan: the order we bring things up

RI3D is short, so bring the robot up in layers, testing each one before adding the next.

**Day 1: drive**
1. Wire the drive motors. Name them on the Driver Hub: `frontLeft frontRight backLeft backRight`, `imu`.
2. Robot on blocks → run RoboHawk TeleOp → push left stick forward. All 4 wheels must spin forward.
   Fix directions in `MecanumDriveSubsystem` (`setDirection` lines).
3. On the floor: check strafe right goes right. Then check the IMU orientation (Logo/USB direction)
   and test field-centric (gp1 **Y** toggles, **Back** resets heading).
4. Not using a subsystem yet? The OpMode will crash if a configured name is missing.
   Temporarily comment out that subsystem's `new ...Subsystem(...)` line and its bindings in `MainTeleOp`.

**Day 2: mechanisms**
1. Intake (`intake`): check the direction with gp2 triggers.
2. Shooters (`nectarShooter`, `pollenShooter`): **encoder cables required**.
   Confirm `Constants.Shooter.TICKS_PER_REV` matches your motor and gearing (28 = bare goBILDA 6000rpm / REV HD Hex).
3. Run **RoboHawk Shooter Tune** and follow its recipe: kV first, then kP, kS/kI only if needed.
   Paste the printed `new ShooterGains(...)` line into `Constants.Shooter.NECTAR` / `POLLEN`.

**Day 3: vision + polish**
1. Limelight 3A named `limelight`. Pipeline 0 = AprilTag 36h11, tag size **82.55 mm**.
2. Mount the camera to look **up** at the HIVE CELL undersides from the shooting spot.
3. Calibrate `Constants.Vision.ROLL_OFFSET_DEG` (upright cell → roll ≈ 0, flipped cell → >90).
4. Tune `AIM_KP` / `AIM_MIN_TURN` with gp1 **left bumper** (auto-aim).
5. Fill the distance→RPM table (`LUT_DISTANCE_IN`, `*_LUT_RPM`) from 5–6 real test shots.
6. The code runs fine **without** a Limelight, so vision can wait if build time runs out.

## 3. What to change for OUR robot

Fill in these items as the robot gets built:

- [ ] **Motor names** — in `Constants.Hardware`, match whatever we name things on the Driver Hub.
- [ ] **Drive directions** — `MecanumDriveSubsystem` constructor.
- [ ] **Control Hub mounting** — `RevHubOrientationOnRobot` in `MecanumDriveSubsystem`.
- [ ] **Shooter direction** — `false` → `true` (reversed) in `MainTeleOp` / `ShooterTuneOpMode` if a wheel spins backwards.
- [ ] **Ticks per rev** — `Constants.Shooter.TICKS_PER_REV` (multiply by gear/belt ratio).
- [ ] **Shooter gains + default RPM** — from Shooter Tune.
- [ ] **AprilTag IDs** — verify `Constants.Vision.CELL_TAG_IDS` against manual Figure 9-17
      (only 38–41 BLUE AUDIENCE was confirmed by the original author).
- [ ] **Mechanism differs from the template?** (e.g. one shooter, a servo gate/feeder, an arm)
      → see section 4.

## 4. How to add or change a mechanism

Example: a servo **feeder gate** that pushes pieces into the flywheel.

1. `Constants.Hardware`: add `public static final String FEEDER = "feeder";` plus positions in a new `Constants.Feeder` class.
2. `subsystems/FeederSubsystem.java`: `extends SubsystemBase`, gets the servo in its constructor,
   has simple methods like `open()` / `close()`.
3. `commands/FeedCommand.java`: `extends CommandBase`, `addRequirements(feeder)`,
   `initialize()` → `open()`, `end()` → `close()`.
   (Or skip the class: `new InstantCommand(feeder::open, feeder)`.)
4. `MainTeleOp.initialize()`: create the subsystem, bind a button:
   `new Trigger(() -> gamepad2.dpad_up).whileTrue(new FeedCommand(feeder));`
   Want "feed only when shooter is at speed"? `new Trigger(() -> gamepad2.dpad_up && nectarShooter.isAtSpeed())`.

Only one shooter? Delete the pollen lines in both OpModes and in `Constants`. Nothing else depends on them.

## 5. Controls (RoboHawk TeleOp)

**INIT:** gp1 d-pad ← BLUE / → RED.

| Gamepad 1 (driver) | | Gamepad 2 (operator) | |
|---|---|---|---|
| Left stick | drive + strafe | Right trigger | intake in |
| Right stick X | turn | Left trigger | intake out |
| Right bumper (hold) | slow mode | A / B | nectar / pollen on-off |
| Left bumper (hold) | auto-aim at our upright CELL | Right / Left bumper | nectar / pollen, RPM from distance |
| Y | field / robot-centric | X | stop both shooters |
| Back | reset heading | | |

## 6. Next steps after TeleOp works

- **Autonomous:** reuse the same subsystems in a `CommandOpMode`. `AimAtCellCommand.isAimed()` and
  `ShooterSubsystem.isAtSpeed()` already exist for "aim → wait for speed → feed".
  Add encoders or odometry to the drive for accurate movement.
- **Driver feedback:** rumble the operator gamepad when the shooter is `isAtSpeed()`.
- **Live tuning:** `ShooterGains` fields are public/non-final, ready for FTC Dashboard / Panels if added later.
