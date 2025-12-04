package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

public class ShooterSubsystem extends RE_SubsystemBase {

    private final DcMotorEx shooterMotor1; 
    private final DcMotorEx shooterMotor2;

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

    public ShooterSubsystem(HardwareMap hardwareMap, String motorName1, String motorName2) {

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, motorName1);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, motorName2);

        initMotor(shooterMotor1);
        initMotor(shooterMotor2);

        ticksPerRev = shooterMotor1.getMotorType().getTicksPerRev();
        maxTicksPerSecond = (ticksPerRev * MAX_RPM) / 60.0;

        shooterState = ShooterState.STOP;

        Robot.getInstance().subsystems.add(this);
    }

    private void initMotor(DcMotorEx motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(Constants.kP, Constants.kI, Constants.kD, Constants.kF)
        );
    }

    public void updateShooterState(ShooterState newState) {
        shooterState = newState;
    }

    @Override
    public void periodic() {

        switch (shooterState) {
            case LOWERPOWER:
                targetVelocity = Constants.lowerShootPower * maxTicksPerSecond;
                shooterMotor1.setVelocity(targetVelocity);
                shooterMotor2.setVelocity(targetVelocity);
                break;

            case SHOOT:
                targetVelocity = Constants.shootPower * maxTicksPerSecond;
                shooterMotor1.setVelocity(targetVelocity);
                shooterMotor2.setVelocity(targetVelocity);
                break;

            case STOP:
                targetVelocity = 0.0;
                shooterMotor1.setPower(0.0);
                shooterMotor2.setPower(0.0);
                break;
        }
    }

    public double getCurrentVelocity() {
        return shooterMotor1.getVelocity();
    }

    public double getCurrentRPM() {
        return (shooterMotor1.getVelocity() / ticksPerRev) * 60.0;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void stopShooter() {
        targetVelocity = 0;
        shooterState = ShooterState.STOP;
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
    }
}