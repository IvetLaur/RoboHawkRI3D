package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * SUBSYSTEM: the mecanum drivetrain.
 *
 * A subsystem owns its hardware. Nothing else in the code touches these four
 * motors directly — other code asks the subsystem to do things ("drive this
 * way"). That way only one place can ever be setting drive power.
 */
public class MecanumDriveSubsystem extends SubsystemBase {

    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private final IMU imu;

    private boolean fieldCentric = false;

    public MecanumDriveSubsystem(HardwareMap hardwareMap) {
        frontLeft  = hardwareMap.get(DcMotorEx.class, Constants.Hardware.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotorEx.class, Constants.Hardware.FRONT_RIGHT);
        backLeft   = hardwareMap.get(DcMotorEx.class, Constants.Hardware.BACK_LEFT);
        backRight  = hardwareMap.get(DcMotorEx.class, Constants.Hardware.BACK_RIGHT);

        // The motors on the left side face the opposite way from the right side,
        // so we reverse the left side so "positive power" means "forward" for all four.
        // If your robot drives backwards or spins when you push forward, fix it HERE.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            // BRAKE = robot stops quickly when you let go of the stick.
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // The IMU (gyro) in the Control Hub tells us which way the robot faces.
        // Change these two values to match how YOUR Control Hub is mounted.
        imu = hardwareMap.get(IMU.class, Constants.Hardware.IMU);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
    }

    /**
     * Drive the robot.
     *
     * @param forward  +1 = full forward,  -1 = full backward
     * @param strafe   +1 = full right,    -1 = full left
     * @param turn     +1 = turn right,    -1 = turn left
     * @param scale    0..1 multiplier (used for slow mode)
     */
    public void drive(double forward, double strafe, double turn, double scale) {
        forward = deadband(forward);
        strafe  = deadband(strafe) * Constants.Drive.STRAFE_MULTIPLIER;
        turn    = deadband(turn);

        // Field-centric: rotate the stick direction by the robot's heading, so
        // "stick up" always means "away from the driver", no matter where the robot faces.
        if (fieldCentric) {
            double heading = getHeadingRadians();
            double rotStrafe  = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
            double rotForward = strafe * Math.sin(-heading) + forward * Math.cos(-heading);
            strafe = rotStrafe;
            forward = rotForward;
        }

        // The mecanum formula: each wheel gets a mix of forward, strafe and turn.
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // If any wheel is asked for more than 100%, shrink ALL of them by the
        // same amount so the robot still moves in the direction you asked.
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                                            Math.max(Math.abs(bl), Math.abs(br))));

        frontLeft.setPower(fl / max * scale);
        frontRight.setPower(fr / max * scale);
        backLeft.setPower(bl / max * scale);
        backRight.setPower(br / max * scale);
    }

    public void stop() {
        drive(0, 0, 0, 0);
    }

    public void toggleFieldCentric() {
        fieldCentric = !fieldCentric;
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    /** Call when the robot is pointing straight away from the driver. */
    public void resetHeading() {
        imu.resetYaw();
    }

    public double getHeadingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    private static double deadband(double value) {
        return Math.abs(value) < Constants.Drive.DEADBAND ? 0 : value;
    }
}
