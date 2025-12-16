package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

public class TurretSubsystem extends RE_SubsystemBase {

    private final ServoEx turretServo;
    private final CameraSubsystem camera;

    private double integral = 0.0;
    private double lastErrDeg = 0.0;
    private double errFiltDeg = 0.0;
    private long lastNanos = 0L;

    public enum TurretState {
        MANUAL,
        AUTO_AIM
    }

    private TurretState turretState;

    private static final double MIN_DT = 1e-3;  // 1 ms
    private static final double MAX_DT = 0.05;  // 50 ms

    // Axon servo range configuration
    private static final double SERVO_MIN_ANGLE = 0.0;      // degrees
    private static final double SERVO_MAX_ANGLE = 355.0;    // degrees (Axon max range)

    // Turret mechanical limits (adjust these to your actual turret range)
    private static final double LEFT_LIMIT_DEG = 0.0;
    private static final double RIGHT_LIMIT_DEG = 180.0;

    // Current target angle for manual control
    private double targetAngleDeg = 90.0;  // Start centered

    public TurretSubsystem(HardwareMap hw, String servoName, CameraSubsystem camera) {
        // Initialize Axon servo with FTCLib
        this.turretServo = new SimpleServo(hw, servoName, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        this.camera = camera;

        turretState = TurretState.MANUAL;

        // Set initial position to center
        turretServo.turnToAngle(targetAngleDeg);

        Robot.getInstance().subsystems.add(this);
        lastNanos = System.nanoTime();
    }

    public void setTurretState(TurretState newstate) {
        this.turretState = newstate;
        if (newstate == TurretState.MANUAL) {
            // When switching to manual, hold current position
            targetAngleDeg = getTurretAngleDeg();
        } else {
            resetPID();
        }
    }

    public TurretState getTurretState() {
        return turretState;
    }

    public void enableAutoAim(boolean enable) {
        setTurretState(enable ? TurretState.AUTO_AIM : TurretState.MANUAL);
    }

    /**
     * Set turret angle directly (for manual control)
     * @param angleDeg Target angle in degrees
     */
    public void setTurretAngle(double angleDeg) {
        targetAngleDeg = clamp(angleDeg, LEFT_LIMIT_DEG, RIGHT_LIMIT_DEG);
        turretServo.turnToAngle(targetAngleDeg);
    }

    /**
     * Move turret by a relative amount (for manual control)
     * @param deltaDeg Change in angle (positive = right, negative = left)
     */
    public void moveTurretRelative(double deltaDeg) {
        targetAngleDeg = clamp(targetAngleDeg + deltaDeg, LEFT_LIMIT_DEG, RIGHT_LIMIT_DEG);
        turretServo.turnToAngle(targetAngleDeg);
    }

    /**
     * Get current turret angle from servo
     * @return Current angle in degrees
     */
    public double getTurretAngleDeg() {
        return turretServo.getAngle();
    }

    /**
     * Get target angle (useful for telemetry)
     * @return Target angle in degrees
     */
    public double getTargetAngleDeg() {
        return targetAngleDeg;
    }

    /**
     * Check if turret is at target position
     * @param toleranceDeg Tolerance in degrees
     * @return True if within tolerance
     */
    public boolean isAtTarget(double toleranceDeg) {
        return Math.abs(getTurretAngleDeg() - targetAngleDeg) < toleranceDeg;
    }

    @Override
    public void updateData() {
        // Uncomment and update your Robot data structure as needed
        // Robot.getInstance().data.turretState = turretState.name();
        // Robot.getInstance().data.turretAngleDeg = getTurretAngleDeg();
        // Robot.getInstance().data.turretTargetAngleDeg = targetAngleDeg;
    }

    @Override
    public void periodic() {
        if (turretState == TurretState.AUTO_AIM) {
            runAutoAimPID();
        }
    }

    private void runAutoAimPID() {
        long now = System.nanoTime();
        double dt = (now - lastNanos) / 1e9;
        lastNanos = now;

        if (dt < MIN_DT) dt = MIN_DT;
        if (dt > MAX_DT) dt = MAX_DT;

        // If no target detected, hold current position
        if (!camera.hasBasket()) {
            targetAngleDeg = getTurretAngleDeg();
            lastErrDeg = 0.0;
            return;
        }

        // Get yaw error from Limelight (negative = target is left, positive = target is right)
        double yawErrDeg = camera.getBasketYawDeg();

        // Apply deadband
        if (Math.abs(yawErrDeg) < Constants.deadbandDeg) yawErrDeg = 0.0;

        // Low-pass filter the error
        errFiltDeg = Constants.errAlpha * yawErrDeg + (1.0 - Constants.errAlpha) * errFiltDeg;

        // Integral term with anti-windup
        integral += errFiltDeg * dt;
        if (yawErrDeg == 0.0) integral *= 0.5;
        integral = clamp(integral, -Constants.maxIntegral, Constants.maxIntegral);

        // Derivative term
        double deriv = (errFiltDeg - lastErrDeg) / dt;
        deriv = clamp(deriv, -Constants.maxDeriv, Constants.maxDeriv);
        lastErrDeg = errFiltDeg;

        // Calculate correction angle using PID
        double correctionDeg =
                Constants.kP_v * errFiltDeg +
                        Constants.kI_v * integral +
                        Constants.kD_v * deriv;

        // Calculate new target angle
        // Current angle + correction (negative yaw error means turn left/decrease angle)
        double currentAngle = getTurretAngleDeg();
        targetAngleDeg = currentAngle - correctionDeg;  // Subtract because camera yaw is opposite

        // Clamp to mechanical limits
        targetAngleDeg = clamp(targetAngleDeg, LEFT_LIMIT_DEG, RIGHT_LIMIT_DEG);

        // Anti-windup: reduce integral if at limits
        if ((targetAngleDeg <= LEFT_LIMIT_DEG || targetAngleDeg >= RIGHT_LIMIT_DEG)) {
            integral *= 0.95;
        }

        // Command servo to target angle
        turretServo.turnToAngle(targetAngleDeg);
    }

    private void resetPID() {
        integral = 0.0;
        lastErrDeg = 0.0;
        errFiltDeg = 0.0;
        lastNanos = System.nanoTime();
        targetAngleDeg = getTurretAngleDeg();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}