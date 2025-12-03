package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

public class ShooterSubsystem extends RE_SubsystemBase {

    private final DcMotorEx shootMotor;

    public enum ShootState {
        LOWERPOWER,
        SHOOT,
        STOP
    }

    public ShootState shootState;

    private double ticksPerRev;
    private static final double MAX_RPM = 6000.0;
    private double maxTicksPerSecond;

    private double targetVelocity = 0;

    public ShooterSubsystem(HardwareMap hardwareMap, String motorName) {

        shootMotor = hardwareMap.get(DcMotorEx.class, motorName);

        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticksPerRev = shootMotor.getMotorType().getTicksPerRev();
        maxTicksPerSecond = (ticksPerRev * MAX_RPM) / 60.0;

        shootMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(Constants.kP, Constants.kI, Constants.kD, Constants.kF)
        );

        shootState = ShootState.STOP;

        Robot.getInstance().subsystems.add(this);
    }

    @Override
    public void updateData() {
//        Robot.getInstance().data.shootState = shootState;
//        Robot.getInstance().data.shootVelocity = shootMotor.getVelocity();
//        Robot.getInstance().data.shootTargetVelocity = targetVelocity;
    }

    public void updateShootState(ShootState newState) {
        shootState = newState;
    }

    @Override
    public void periodic() {

        shootMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(Constants.kP, Constants.kI, Constants.kD, Constants.kF)
        );

        switch (shootState) {

            case LOWERPOWER:
                targetVelocity = Constants.lowerShootPower * maxTicksPerSecond;
                shootMotor.setVelocity(targetVelocity);
                break;

            case SHOOT:
                targetVelocity = Constants.shootPower * maxTicksPerSecond;
                shootMotor.setVelocity(targetVelocity);
                break;

            case STOP:
                targetVelocity = 0.0;
                shootMotor.setPower(0.0);
                break;
        }
    }

    public double getCurrentVelocity() {
        return shootMotor.getVelocity();
    }

    public double getCurrentRPM() {
        return (shootMotor.getVelocity() / ticksPerRev) * 60.0;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void stopShooter() {
        targetVelocity = 0;
        shootState = ShootState.STOP;
        shootMotor.setPower(0);
    }
}