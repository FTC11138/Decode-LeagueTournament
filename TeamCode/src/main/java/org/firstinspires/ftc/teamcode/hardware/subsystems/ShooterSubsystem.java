package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

public class ShooterSubsystem extends RE_SubsystemBase {

    private final DcMotorEx shooterMotor;

    public enum ShooterState {
        LOWERPOWER,
        SHOOT,
        STOP
    }

    public ShooterState shooterState;

    private double ticksPerRev;
    private static final double MAX_RPM = 6000.0;
    private double maxTicksPerSecond;

    private double targetVelocity = 0;

    public ShooterSubsystem(HardwareMap hardwareMap, String motorName) {

        shooterMotor = hardwareMap.get(DcMotorEx.class, motorName);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticksPerRev = shooterMotor.getMotorType().getTicksPerRev();
        maxTicksPerSecond = (ticksPerRev * MAX_RPM) / 60.0;

        shooterMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(Constants.kP, Constants.kI, Constants.kD, Constants.kF)
        );

        shooterState = ShooterState.STOP;

        Robot.getInstance().subsystems.add(this);
    }

    @Override
    public void updateData() {
//        Robot.getInstance().data.shootState = shootState;
//        Robot.getInstance().data.shootVelocity = shootMotor.getVelocity();
//        Robot.getInstance().data.shootTargetVelocity = targetVelocity;
    }

    public void updateShooterState(ShooterState newState) {
        shooterState = newState;
    }

    @Override
    public void periodic() {

        shooterMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(Constants.kP, Constants.kI, Constants.kD, Constants.kF)
        );

        switch (shooterState) {

            case LOWERPOWER:
                targetVelocity = Constants.lowerShootPower * maxTicksPerSecond;
                shooterMotor.setVelocity(targetVelocity);
                break;

            case SHOOT:
                targetVelocity = Constants.shootPower * maxTicksPerSecond;
                shooterMotor.setVelocity(targetVelocity);
                break;

            case STOP:
                targetVelocity = 0.0;
                shooterMotor.setPower(0.0);
                break;
        }
    }

    public double getCurrentVelocity() {
        return shooterMotor.getVelocity();
    }

    public double getCurrentRPM() {
        return (shooterMotor.getVelocity() / ticksPerRev) * 60.0;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void stopShooter() {
        targetVelocity = 0;
        shooterState = ShooterState.STOP;
        shooterMotor.setPower(0);
    }
}