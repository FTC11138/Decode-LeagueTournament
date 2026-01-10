package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Const;
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

    private static final double MAX_RPM = 6000.0;
    private static final double TICKS_PER_REV = 28;
    private static final double MAX_TICKS_PER_SECOND = (TICKS_PER_REV * MAX_RPM) / 60.0;

    private double targetVelocity = 0;
    private double currentVelocity1 = 0;
    private double currentVelocity2 = 0;
    private double currentRPM1 = 0;
    private double currentRPM2 = 0;

    public ShooterSubsystem(HardwareMap hardwareMap, String motorName1, String motorName2) {

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, motorName1);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, motorName2);

        initMotor(shooterMotor1);
        initMotor(shooterMotor2);

        shooterState = ShooterState.STOP;

        Robot.getInstance().subsystems.add(this);
    }

    @Override
    public void updateData() {
        Robot robot = Robot.getInstance();

        robot.data.shooterState = shooterState;
        robot.data.shooterTargetVelocity = targetVelocity;
        robot.data.shooterCurrentVelocity1 = currentVelocity1;
        robot.data.shooterCurrentVelocity2 = currentVelocity2;
        robot.data.shooterCurrentRPM1 = currentRPM1;
        robot.data.shooterCurrentRPM2 = currentRPM2;
    }

    private void initMotor(DcMotorEx motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor.setPIDFCoefficients(
//                DcMotor.RunMode.RUN_USING_ENCODER,
//                new PIDFCoefficients(Constants.kP, Constants.kI, Constants.kD, Constants.kF)
//        );
    }

    public void updateShooterState(ShooterState newState) {
        shooterState = newState;
    }

    @Override
    public void periodic() {

        switch (shooterState) {
            case LOWERPOWER:
                targetVelocity = Constants.lowerShootPower * MAX_TICKS_PER_SECOND;
                shooterMotor1.setVelocity(targetVelocity);
                shooterMotor2.setVelocity(targetVelocity);
                break;

            case SHOOT:
                shooterMotor1.setPower(Constants.shootPower);
                shooterMotor2.setPower(Constants.shootPower);
                break;

            case STOP:
                shooterMotor1.setPower(0.0);
                shooterMotor2.setPower(0.0);
                break;
        }

        currentVelocity1 = shooterMotor1.getVelocity();
        currentVelocity1 = shooterMotor2.getVelocity();
        currentRPM1 = (currentVelocity1 / TICKS_PER_REV) * 60.0;
        currentRPM2 = (currentVelocity2 / TICKS_PER_REV) * 60.0;
    }

    public double getCurrentVelocity1() {
        return currentVelocity1;
    }

    public double getCurrentVelocity2() {
        return currentVelocity2;
    }

    public double getCurrentRPM1() {
        return currentRPM1;
    }

    public double getCurrentRPM2() {
        return currentRPM2;
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