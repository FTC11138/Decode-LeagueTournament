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
        LOWERPOWER,
        SHOOT,
        STOP
    }

    // ✅ keep same names
    public enum AdjHoodState {
        NOTMOVING,
        SHOOTING,
        MANUAL
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

    // Manual hood target
    private double manualHoodPos = 0.5;

    // Manual shooter target (TPS)
    private boolean manualShooterEnabled = false;
    private double manualShooterTPS = 0.0;

    // ✅ HARD LOCK for tuning (prevents other commands from overriding)
    private boolean tuningLock = false;

    /* =====================
       EXISTING METHODS (UNCHANGED)
       ===================== */

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
        adjHoodState = AdjHoodState.NOTMOVING;

        Robot.getInstance().subsystems.add(this);
    }

    private void initMotor(DcMotorEx motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ✅ If tuning lock is enabled, ignore state updates from other commands
    public void updateShooterState(ShooterState newState) {
        if (tuningLock) return;
        shooterState = newState;
    }

    public void updateAdjHoodState(AdjHoodState newState) {
        if (tuningLock) return;
        adjHoodState = newState;
    }

    /* =====================
       ✅ TUNING LOCK API
       ===================== */

    public void enableTuningLock() {
        tuningLock = true;
        // force safe defaults while locked
        adjHoodState = AdjHoodState.MANUAL;
        shooterState = ShooterState.STOP;
    }

    public void disableTuningLock() {
        tuningLock = false;
    }

    public boolean isTuningLockEnabled() {
        return tuningLock;
    }

    /* =====================
       ✅ MANUAL HOOD API
       ===================== */

    public void setManualHoodPos(double pos) {
        manualHoodPos = clamp01(pos);
    }

    public double getManualHoodPos() {
        return manualHoodPos;
    }

    /* =====================
       ✅ MANUAL SHOOTER API (TPS)
       ===================== */

    public void setManualShooterTPS(double tps) {
        manualShooterEnabled = true;
        manualShooterTPS = tps;
    }

    public void disableManualShooterTPS() {
        manualShooterEnabled = false;
    }

    public boolean isManualShooterEnabled() {
        return manualShooterEnabled;
    }

    public double getManualShooterTPS() {
        return manualShooterTPS;
    }

    private double clamp01(double x) {
        return Math.max(0.0, Math.min(1.0, x));
    }

    /* =====================
       PERIODIC
       ===================== */

    @Override
    public void periodic() {

        dist = Robot.getInstance()
                .turretOdometrySubsystem
                .getDist();

        // ✅ If tuning lock is on, FORCE manual behavior here.
        if (tuningLock) {
            // Shooter: only runs if you put state to SHOOT in tuning opmode
            if (shooterState == ShooterState.SHOOT) {
                targetVelocity = manualShooterEnabled ? manualShooterTPS : 0.0;
                shooterMotor1.setVelocity(targetVelocity);
                shooterMotor2.setVelocity(targetVelocity);
            } else {
                shooterMotor1.setPower(0);
                shooterMotor2.setPower(0);
                targetVelocity = 0;
            }

            // Hood: always manual when locked
            adjHood.setPosition(manualHoodPos);

            currentVelocity1 = shooterMotor1.getVelocity();
            currentVelocity2 = shooterMotor2.getVelocity();
            currentRPM1 = (currentVelocity1 / TICKS_PER_REV) * 60.0;
            currentRPM2 = (currentVelocity2 / TICKS_PER_REV) * 60.0;
            return;
        }

        // Shooter control (UNCHANGED)
        switch (shooterState) {
            case LOWERPOWER:
                targetVelocity =
                        Constants.lowerShootPower * MAX_TICKS_PER_SECOND;
                shooterMotor1.setVelocity(targetVelocity);
                shooterMotor2.setVelocity(targetVelocity);
                break;

            case SHOOT:
                targetVelocity = manualShooterEnabled ? manualShooterTPS : flywheelSpeed(dist);
                shooterMotor1.setVelocity(targetVelocity);
                shooterMotor2.setVelocity(targetVelocity);
                break;

            case STOP:
                shooterMotor1.setPower(0);
                shooterMotor2.setPower(0);
                break;
        }

        // Hood control (one writer)
        switch (adjHoodState) {
            case MANUAL:
                adjHood.setPosition(manualHoodPos);
                break;

            case SHOOTING:
                adjHood.setPosition(adjHoodPos(this.dist));
                break;

            case NOTMOVING:
            default:
                adjHood.setPosition(0.0);
                break;
        }

        currentVelocity1 = shooterMotor1.getVelocity();
        currentVelocity2 = shooterMotor2.getVelocity();
        currentRPM1 = (currentVelocity1 / TICKS_PER_REV) * 60.0;
        currentRPM2 = (currentVelocity2 / TICKS_PER_REV) * 60.0;
    }

    /* =====================
       GETTERS (UNCHANGED)
       ===================== */

    public double getCurrentVelocity1() { return currentVelocity1; }
    public double getCurrentVelocity2() { return currentVelocity2; }
    public double getCurrentRPM1() { return currentRPM1; }
    public double getCurrentRPM2() { return currentRPM2; }
    public double getTargetVelocity() { return targetVelocity; }

    public void stopShooter() {
        targetVelocity = 0;
        shooterState = ShooterState.STOP;
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
    }
}
