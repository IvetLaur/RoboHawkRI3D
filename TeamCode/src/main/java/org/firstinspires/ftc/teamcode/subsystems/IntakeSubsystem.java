package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * SUBSYSTEM: the intake roller.
 *
 * Three simple actions: pull in, push out, stop. Commands decide WHEN;
 * this class only knows HOW.
 */
public class IntakeSubsystem extends SubsystemBase {

    private final DcMotorEx motor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, Constants.Hardware.INTAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD); // flip if it spins the wrong way
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intake() { motor.setPower(Constants.Intake.IN_POWER); }

    public void outtake() { motor.setPower(Constants.Intake.OUT_POWER); }

    public void stop() { motor.setPower(0); }

    public double getPower() { return motor.getPower(); }
}
