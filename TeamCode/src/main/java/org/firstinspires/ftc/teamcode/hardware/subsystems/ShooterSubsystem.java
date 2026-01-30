package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

import java.util.NavigableMap;
import java.util.TreeMap;

@Configurable
public class ShooterSubsystem extends RE_SubsystemBase {

    private final DcMotorEx shooterMotor1;
    private final DcMotorEx shooterMotor2;
    private final Servo adjHood;
    private final Servo led;

    public enum ShooterState {
        MANUAL,
        AUTO,
        FAR,
        STOP,
        AUTONOMOUS,
        FRONT
    }

    // ✅ keep same names
    public enum AdjHoodState {
        MANUAL,
        AUTO,
        FAR,
        NONE,
        AUTONOMOUS
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

    private static class ShotPoint {
        final double hood;
        final double power; // or velocity target

        ShotPoint(double hood, double power) {
            this.hood = hood;
            this.power = power;
        }
    }

    private static final NavigableMap<Double, ShotPoint> SHOT_TABLE = new TreeMap<>();
    static {
        SHOT_TABLE.put(33.8, new ShotPoint(0.47, -1275));
        SHOT_TABLE.put(44.8, new ShotPoint(0.43, -1325));
        SHOT_TABLE.put(50.8, new ShotPoint(0.37, -1425));
        SHOT_TABLE.put(57.1, new ShotPoint(0.37, -1450));
        SHOT_TABLE.put(69.9, new ShotPoint(0.33, -1500));
        SHOT_TABLE.put(80.7, new ShotPoint(0.31, -1600));
    }


    private static ShotPoint nearestShotPoint(double goalDist) {
        if (SHOT_TABLE.isEmpty()) return new ShotPoint(Constants.adjHoodMin, 0);

        Double low = SHOT_TABLE.floorKey(goalDist);
        Double high = SHOT_TABLE.ceilingKey(goalDist);

        if (low == null) return SHOT_TABLE.get(high);
        if (high == null) return SHOT_TABLE.get(low);

        return (Math.abs(goalDist - low) <= Math.abs(high - goalDist))
                ? SHOT_TABLE.get(low)
                : SHOT_TABLE.get(high);
    }

    // Nearest-neighbor selection using midpoints between your measured x_1 values.
// Data (x_1 -> H, P):
// 36 -> 0.54, -1275
// 47 -> 0.34, -1325
// 60 -> 0.38, -1425
// 73 -> 0.36, -1600
// 92 -> 0.36, -1625
// 104 -> 0.39, -1725
// 126 -> 0.38, -1850

    /*

    public static double adjHoodPos(double goalDist) {

        // Midpoints: 41.5, 53.5, 66.5, 82.5, 98.0, 115.0
        if (goalDist < 30) return 0.48;
        else if (goalDist < 41.5) return 0.54;      // nearest to 36
        else if (goalDist < 53.5) return 0.34; // nearest to 47
        else if (goalDist < 66.5) return 0.38; // nearest to 60
        else if (goalDist < 82.5) return 0.36; // nearest to 73
        else if (goalDist < 98.0) return 0.36; // nearest to 92
        else if (goalDist < 115.0) return 0.39;// nearest to 104
        else return 0.38;                       // nearest to 126
    }

    public static double flywheelSpeed(double goalDist) {
        // Optional special-case if you want one for super-close shots:
        // if (goalDist < 35) return -1200; // <-- set whatever you want, or delete this

        // Midpoints: 41.5, 53.5, 66.5, 82.5, 98.0, 115.0
        if (goalDist < 41.5) return -1275;       // nearest to 36
        else if (goalDist < 53.5) return -1325;  // nearest to 47
        else if (goalDist < 66.5) return -1425;  // nearest to 60
        else if (goalDist < 82.5) return -1600;  // nearest to 73
        else if (goalDist < 98.0) return -1625;  // nearest to 92
        else if (goalDist < 115.0) return -1725; // nearest to 104
        else return -1850;                       // nearest to 126
    }


     */

    public ShooterSubsystem(HardwareMap hardwareMap,
                            String motorName1,
                            String motorName2,
                            String servoName1) {

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, motorName1);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, motorName2);
        adjHood = hardwareMap.get(Servo.class, servoName1);
        led = hardwareMap.get(Servo.class, "led");

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
        hoodPos = pos;
    }

    public void setShooterSpeed(double speed) {
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

        switch (shooterState) {
            case MANUAL:
                break;
            case AUTO:
                targetVelocity = nearestShotPoint(dist).power;
                break;
            case FAR:
                targetVelocity = Constants.shootVelFar;
                break;
            case STOP:
                shooterMotor1.setPower(0);
                shooterMotor2.setPower(0);
                targetVelocity = 0;
                break;
            case AUTONOMOUS:
                targetVelocity = -1325;
                break;
            case FRONT:
                targetVelocity = -1400;
                break;
        }

        switch (adjHoodState) {
            case MANUAL:
                adjHood.setPosition(MathFunctions.clamp(hoodPos, Constants.adjHoodMax, Constants.adjHoodMin));
                break;
            case AUTO:
                adjHood.setPosition(nearestShotPoint(dist).hood);
                break;
            case FAR:
                adjHood.setPosition(Constants.shootHoodFar);
                break;
            case NONE:
                adjHood.setPosition(Constants.adjHoodMin);
                break;
            case AUTONOMOUS:
                adjHood.setPosition(0.37);
                break;
        }

        if (Math.abs(currentVelocity1 - targetVelocity) < 40) {
            led.setPosition(0.444);
        } else {
            if (shooterState == ShooterState.FAR) {
                led.setPosition(.65);
            } else {
                led.setPosition(1);
            }
        }



        shooterMotor1.setVelocity(targetVelocity);
        shooterMotor2.setVelocity(targetVelocity);
//        shooterMotor1.setVelocityPIDFCoefficients(Constants.kP_velo, Constants.kI_velo, Constants.kD_velo, Constants.kF_velo);
//        shooterMotor2.setVelocityPIDFCoefficients(Constants.kP_velo, Constants.kI_velo, Constants.kD_velo, Constants.kF_velo);



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
