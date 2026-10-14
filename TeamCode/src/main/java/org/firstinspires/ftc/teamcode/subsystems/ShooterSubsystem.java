package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * SUBSYSTEM: one flywheel shooter with our own PIDF + voltage compensation.
 *
 * We have TWO shooters (nectar and pollen) that work the same way, so we write
 * this class ONCE and create two objects from it.
 *
 * HOW THE CONTROLLER WORKS (runs every loop in periodic()):
 *
 *   power = kV * target            <- feedforward: "this speed needs about this much power"
 *         + kS * sign(target)      <- overcome friction
 *         + kP * error             <- push harder if too slow, ease off if too fast
 *         + kI * sum(error)        <- fix small leftover error (only near target)
 *         + kD * change(error)     <- calm down overshoot
 *
 *   then:  power *= 12V / batteryVoltage     <- VOLTAGE COMPENSATION
 *
 * Why compensate? A fresh battery (~13.5 V) spins the wheel faster than a tired
 * one (~12 V) at the same power. Scaling by voltage makes "0.5 power" mean the
 * same thing in the first match and the last, so shots stay consistent.
 *
 * Why not just use the motor's built-in setVelocity()? It works, but you can't
 * see or tune feedforward, voltage comp or the integral zone. Writing it here
 * makes every piece visible — and tunable in Shooter Tune mode.
 */
public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx motor;
    private final VoltageSensor batterySensor;
    private final ShooterGains gains;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private boolean enabled = false;
    private double targetRpm = 0;
    private double filteredRpm = 0;
    private double integral = 0;
    private double lastError = 0;
    private double batteryVoltage = Constants.Shooter.NOMINAL_VOLTAGE;
    private double lastPower = 0;

    /**
     * @param hardwareMap the robot's hardware map
     * @param motorName   name from the Robot Configuration
     * @param gains       tuning numbers (Constants.Shooter.NECTAR / POLLEN)
     * @param reversed    true if the wheel spins the wrong way
     */
    public ShooterSubsystem(HardwareMap hardwareMap, String motorName,
                            ShooterGains gains, boolean reversed) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE
                                    : DcMotorSimple.Direction.FORWARD);
        // FLOAT so the heavy flywheel coasts down instead of slamming to a stop.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // WE do the control math, so the hub must NOT run its own velocity loop.
        // The encoder still counts, so getVelocity() keeps working.
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        batterySensor = hardwareMap.voltageSensor.iterator().next();
        batteryVoltage = batterySensor.getVoltage();

        this.gains = gains;
        setName(motorName);
    }

    // ------------------------------------------------------------------ control

    /** Spin to this shooter's default speed. */
    public void spinUp() {
        setTargetRpm(gains.defaultRpm);
    }

    /** Spin to any speed (used by vision distance table and tune mode). */
    public void setTargetRpm(double rpm) {
        if (!enabled) {
            integral = 0;
            lastError = 0;
            loopTimer.reset();
        }
        targetRpm = rpm;
        enabled = rpm > 0;
    }

    public void stop() {
        enabled = false;
        targetRpm = 0;
        integral = 0;
        lastPower = 0;
        motor.setPower(0);
    }

    /** Runs automatically every scheduler loop. This is the PIDF loop. */
    @Override
    public void periodic() {
        // 1. Measure speed (smoothed — raw encoder velocity is noisy).
        double rawRpm = motor.getVelocity() / Constants.Shooter.TICKS_PER_REV * 60.0;
        double a = Constants.Shooter.VELOCITY_FILTER;
        filteredRpm = a * filteredRpm + (1 - a) * rawRpm;

        // 2. Reading the battery is slow-ish, so only do it 10x per second.
        if (voltageTimer.milliseconds() > 100) {
            double v = batterySensor.getVoltage();
            if (v > 6) batteryVoltage = v; // ignore glitch readings
            voltageTimer.reset();
        }

        if (!enabled) return;

        // 3. PIDF math.
        double dt = Math.max(loopTimer.seconds(), 1e-3);
        loopTimer.reset();

        double error = targetRpm - filteredRpm;

        if (Math.abs(error) < Constants.Shooter.INTEGRAL_ZONE_RPM) {
            integral += error * dt;
        } else {
            integral = 0; // far from target (spinning up) -> don't wind up
        }
        double derivative = (error - lastError) / dt;
        lastError = error;

        double feedforward = gains.kV * targetRpm + gains.kS * Math.signum(targetRpm);
        double feedback = gains.kP * error + gains.kI * integral + gains.kD * derivative;

        // 4. Voltage compensation.
        double power = (feedforward + feedback) * (Constants.Shooter.NOMINAL_VOLTAGE / batteryVoltage);

        // Flywheels never need to spin backwards — braking with reverse power is hard on gears.
        power = Math.max(0, Math.min(1, power));
        lastPower = power;
        motor.setPower(power);
    }

    // ------------------------------------------------------------------ info

    public boolean isRunning() { return enabled; }

    public double getTargetRpm() { return targetRpm; }

    public double getCurrentRpm() { return filteredRpm; }

    public double getError() { return targetRpm - filteredRpm; }

    public double getPower() { return lastPower; }

    public double getBatteryVoltage() { return batteryVoltage; }

    public ShooterGains getGains() { return gains; }

    /** True when the flywheel is close enough to target speed to shoot. */
    public boolean isAtSpeed() {
        return enabled && Math.abs(getError()) < Constants.Shooter.READY_TOLERANCE_RPM;
    }
}
