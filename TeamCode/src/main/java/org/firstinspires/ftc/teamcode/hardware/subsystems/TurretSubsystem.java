package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

public class TurretSubsystem extends RE_SubsystemBase {

    private final CRServo turretServo;
    private final DcMotorEx turretEncoder;
    private final CameraSubsystem camera;

    private double integral = 0.0;
    private double lastErrDeg = 0.0;
    private double errFiltDeg = 0.0;
    private long lastNanos = 0L;
    private boolean firstMeasurement = true;

    // Center PD state
    private double lastCenterAngle = 0.0;
    private boolean firstCenterMeasurement = true;

    public enum TurretState {
        MANUAL,
        TRACK,
        CENTER  // Return to center position
    }

    private TurretState turretState = TurretState.MANUAL;

    private static final double MIN_DT = 1e-3;
    private static final double MAX_DT = 0.05;

    private static final double ENCODER_TICKS_PER_REV = 8192.0;
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_DEGREE =
            (ENCODER_TICKS_PER_REV * GEAR_RATIO) / 360.0;

    private static final double LEFT_LIMIT_DEG = -90.0;
    private static final double RIGHT_LIMIT_DEG = 90.0;

    private static final double HOLD_POWER = 0.09; // tune 0.03–0.06
    private static final double CENTER_POWER = 0.25; // max power for centering
    private static final double CENTER_TOLERANCE = 2.5; // degrees tolerance for center
    private static final double CENTER_KP = 0.012; // Proportional gain (lower = smoother)
    private static final double CENTER_KD = 0.008; // Derivative gain (damping)

    public TurretSubsystem(HardwareMap hw, String servoName, String encoderName, CameraSubsystem camera) {
        turretServo = hw.crservo.get(servoName);

        turretEncoder = hw.get(DcMotorEx.class, encoderName);
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.camera = camera;

        Robot.getInstance().subsystems.add(this);
        lastNanos = System.nanoTime();
    }

    /* ================= State ================= */

    public void setTurretState(TurretState newState) {
        turretState = newState;
        resetPID();
        if (newState == TurretState.MANUAL) {
            setTurretPower(0.0);
        }
    }

    public TurretState getTurretState() {
        return turretState;
    }

    public boolean isTracking() {
        return turretState == TurretState.TRACK;
    }

    public boolean isCentered() {
        return Math.abs(getTurretAngleDeg()) < CENTER_TOLERANCE;
    }

    /* ================= Encoder ================= */

    public double getTurretAngleDeg() {
        return turretEncoder.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public void resetEncoder() {
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetPID();
    }

    /* ================= Output ================= */

    public void setTurretPower(double power) {
        double angle = getTurretAngleDeg();

        if (angle <= LEFT_LIMIT_DEG && power < 0) power = 0.0;
        if (angle >= RIGHT_LIMIT_DEG && power > 0) power = 0.0;

        turretServo.setPower(clamp(power, -1.0, 1.0));
    }

    /* ================= Loop ================= */

    @Override
    public void periodic() {
        camera.setCurrentCameraYaw(getTurretAngleDeg());

        if (turretState == TurretState.TRACK) {
            runTrackingPID();
        } else if (turretState == TurretState.CENTER) {
            centerTurret();
        }
    }

    /* ================= PID ================= */

    private void runTrackingPID() {
        long now = System.nanoTime();
        double dt = clamp((now - lastNanos) / 1e9, MIN_DT, MAX_DT);
        lastNanos = now;

        if (!camera.hasBasket()) {
            // No target - center the turret instead of scanning
            centerTurret();
            resetPID();
            return;
        }

        double yawErrDeg = camera.getBasketYawDeg();

        if (Math.abs(yawErrDeg) < Constants.deadbandDeg) {
            setTurretPower(Math.signum(lastErrDeg) * HOLD_POWER);
            integral = 0.0;
            return;
        }

        errFiltDeg =
                Constants.errAlpha * yawErrDeg +
                        (1.0 - Constants.errAlpha) * errFiltDeg;

        integral += errFiltDeg * dt;
        integral = clamp(integral,
                -Constants.maxIntegral,
                Constants.maxIntegral);

        // Calculate derivative, but skip on first measurement to prevent spike
        double deriv = 0.0;
        if (!firstMeasurement) {
            deriv = (errFiltDeg - lastErrDeg) / dt;
            deriv = clamp(deriv, -Constants.maxDeriv, Constants.maxDeriv);
        }
        firstMeasurement = false;
        lastErrDeg = errFiltDeg;

        double power =
                Constants.kP_v * errFiltDeg +
                        Constants.kI_v * integral +
                        Constants.kD_v * deriv;

        if (Constants.kS > 0) {
            power += Math.signum(errFiltDeg) * Constants.kS;
        }

        power = clamp(power,
                -Constants.maxPower,
                Constants.maxPower);

        if (Math.abs(power) >= Constants.maxPower) {
            integral *= 0.9;
        }

        setTurretPower(power);
    }

    /**
     * Center the turret to 0 degrees using PD control
     */
    private void centerTurret() {
        long now = System.nanoTime();
        double dt = clamp((now - lastNanos) / 1e9, MIN_DT, MAX_DT);
        lastNanos = now;

        double currentAngle = getTurretAngleDeg();
        double error = 0.0 - currentAngle; // Target is 0 degrees

        // Check if we're close enough to center
        if (Math.abs(error) < CENTER_TOLERANCE) {
            setTurretPower(0.0);
            firstCenterMeasurement = true; // Reset for next time
            return;
        }

        // Calculate derivative (velocity damping)
        double derivative = 0.0;
        if (!firstCenterMeasurement) {
            derivative = (currentAngle - lastCenterAngle) / dt;
        }
        firstCenterMeasurement = false;
        lastCenterAngle = currentAngle;

        // PD control: P moves toward target, D prevents overshoot
        double power = (Constants.CENTER_KP * error) - (Constants.CENTER_KD * derivative);

        // Clamp to max centering power
        power = clamp(power, -CENTER_POWER, CENTER_POWER);

        setTurretPower(power);
    }

    /* ================= Utils ================= */

    private void resetPID() {
        integral = 0.0;
        lastErrDeg = 0.0;
        errFiltDeg = 0.0;
        firstMeasurement = true;
        firstCenterMeasurement = true;
        lastNanos = System.nanoTime();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}