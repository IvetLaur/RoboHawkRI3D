package org.firstinspires.ftc.teamcode.subsystems;

/**
 * The tuning numbers for one flywheel.
 *
 * Fields are public and NOT final on purpose: the Shooter Tune OpMode changes
 * them live while the wheel spins. (If you add FTC Dashboard/Panels later, it
 * can edit them too.)
 *
 *  kV  feedforward: power per RPM. Does ~90% of the work. Tune FIRST.
 *  kS  static: small constant push to overcome friction.
 *  kP  proportional: fixes leftover error, recovers speed after a shot.
 *  kI  integral: removes tiny steady error. Usually 0.
 *  kD  derivative: damps overshoot. Usually tiny or 0.
 */
public class ShooterGains {
    public double defaultRpm;
    public double kV, kS, kP, kI, kD;

    public ShooterGains(double defaultRpm, double kV, double kS, double kP, double kI, double kD) {
        this.defaultRpm = defaultRpm;
        this.kV = kV;
        this.kS = kS;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /** Ready-to-paste Java for Constants.java. */
    @Override
    public String toString() {
        return String.format("new ShooterGains(%.0f, %.6f, %.3f, %.6f, %.6f, %.6f)",
                defaultRpm, kV, kS, kP, kI, kD);
    }
}
