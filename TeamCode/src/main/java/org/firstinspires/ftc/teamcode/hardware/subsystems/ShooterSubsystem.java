package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

public class ShooterSubsystem extends RE_SubsystemBase {

    private final DcMotorEx shooterMotor1;
    private final DcMotorEx shooterMotor2;
    private final Servo adjHood;

    public enum ShooterState {
        MANUAL,
        AUTO,
        STOP
    }

    // ✅ keep same names
    public enum AdjHoodState {
        MANUAL,
        AUTO,
        NONE,
    }

    Robot robot = Robot.getInstance();

    public ShooterState shooterState;
    public AdjHoodState adjHoodState;

    private static final double MAX_RPM = 6000.0;
    private static final double TICKS_PER_REV = 28;
    private static final double MAX_TICKS_PER_SECOND =
            (TICKS_PER_REV * MAX_RPM) / 60.0;

    private double targetVelocity = 0;
    private double currentVelocity1 = 0;
    private double currentVelocity2 = 0;
    private double currentRPM1 = 0;
    private double currentRPM2 = 0;

    private double dist = 0.0;

    private double hoodPos = 0;


    public static double flywheelSpeed(double goalDist) {
        return MathFunctions.clamp(
                -0.0662677 * Math.pow(goalDist, 2)
                        + 0.630762 * goalDist
                        - 1139.28191,
                -2240,
                0
        );
    }

    public static double adjHoodPos(double goalDist) {
        if (goalDist < 35) return 0.57;
        if (goalDist < 90) return 0.47;
        return 0.3;
    }

    public ShooterSubsystem(HardwareMap hardwareMap,
                            String motorName1,
                            String motorName2,
                            String servoName1) {

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, motorName1);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, motorName2);
        adjHood = hardwareMap.get(Servo.class, servoName1);

        initMotor(shooterMotor1);
        initMotor(shooterMotor2);

        shooterState = ShooterState.STOP;
        adjHoodState = AdjHoodState.NONE;
        hoodPos = Constants.adjHoodMin;

        Robot.getInstance().subsystems.add(this);
    }

    private void initMotor(DcMotorEx motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void updateShooterState(ShooterState newState) {
        shooterState = newState;
    }

    public void updateAdjHoodState(AdjHoodState newState) {
        adjHoodState = newState;
    }

    public void setHoodPos(double pos) {
        adjHoodState = AdjHoodState.MANUAL;
        hoodPos = pos;
    }

    public void setShooterSpeed(double speed) {
        shooterState = ShooterState.MANUAL;
        targetVelocity = speed;
    }

    private double clamp01(double x) {
        return Math.max(0.0, Math.min(1.0, x));
    }

    @Override
    public void updateData() {
        Robot robot = Robot.getInstance();

        robot.data.shooterState = shooterState;
        robot.data.hoodState = adjHoodState;
        robot.data.shooterTargetVelocity = getTargetVelocity();
        robot.data.shooterCurrentVelocity1 = getCurrentVelocity1();
        robot.data.shooterCurrentVelocity2 = getCurrentVelocity2();
        robot.data.shooterCurrentRPM1 = getCurrentRPM1();
        robot.data.shooterCurrentRPM2 = getCurrentRPM2();
        robot.data.hoodPos = getHoodPos();
    }

    @Override
    public void periodic() {

        dist = Robot.getInstance()
                .turretOdometrySubsystem
                .getDist();

        // Shooter control (UNCHANGED)
        switch (shooterState) {
            case MANUAL:
                shooterMotor1.setVelocity(targetVelocity);
                shooterMotor2.setVelocity(targetVelocity);
                break;
            case AUTO:
                shooterMotor1.setVelocity(flywheelSpeed(dist));
                shooterMotor2.setVelocity(flywheelSpeed(dist));
            case STOP:
                shooterMotor1.setPower(0);
                shooterMotor2.setPower(0);
                break;
        }

        // Hood control (one writer)
        switch (adjHoodState) {
            case MANUAL:
                adjHood.setPosition(MathFunctions.clamp(hoodPos, Constants.adjHoodMax, Constants.adjHoodMin));
                break;
            case AUTO:
                adjHood.setPosition(adjHoodPos(dist));
                break;
            case NONE:
                adjHood.setPosition(Constants.adjHoodMin);
                break;
        }


        currentVelocity1 = shooterMotor1.getVelocity();
        currentVelocity2 = shooterMotor2.getVelocity();
        currentRPM1 = (currentVelocity1 / TICKS_PER_REV) * 60.0;
        currentRPM2 = (currentVelocity2 / TICKS_PER_REV) * 60.0;
    }


    public double getCurrentVelocity1() { return currentVelocity1; }
    public double getCurrentVelocity2() { return currentVelocity2; }
    public double getCurrentRPM1() { return currentRPM1; }
    public double getCurrentRPM2() { return currentRPM2; }
    public double getTargetVelocity() { return targetVelocity; }
    public double getHoodPos() { return hoodPos; }

}
