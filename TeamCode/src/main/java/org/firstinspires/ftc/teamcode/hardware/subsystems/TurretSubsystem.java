package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

/**
 * TurretSubsystem:
 *  - If camera sees the april tag target -> use camera yaw PID (UNCHANGED behavior)
 *  - If camera does NOT see the target -> use TurretOdometrySubsystem's desired angle and hold to it
 *  - If odometry pose is unavailable -> center
 *
 * IMPORTANT:
 *  - TurretOdometrySubsystem must NOT drive the servo at the same time.
 *    (With the version I gave you, outputsEnabled defaults to false, so you're safe.)
 */
public class TurretSubsystem extends RE_SubsystemBase {

    private final CRServo turretServo;
    private final DcMotorEx turretEncoder;
    private final CameraSubsystem camera;

    // NEW: used only as a desired-angle provider when the tag is not visible
    private final TurretOdometrySubsystem turretOdom;

    private double integral = 0.0;
    private double lastErrDeg = 0.0;
    private double errFiltDeg = 0.0;
    private long lastNanos = 0L;
    private boolean firstMeasurement = true;

    // Center PD state
    private double lastCenterAngle = 0.0;
    private boolean firstCenterMeasurement = true;

    // Track source switching to avoid derivative/integral spikes when we swap sources
    private enum TrackSource { VISION, ODOM, NONE }
    private TrackSource lastSource = TrackSource.NONE;

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

    private static final double LEFT_LIMIT_DEG = -60.0;
    private static final double RIGHT_LIMIT_DEG = 60.0;

    private static final double HOLD_POWER = 0.1; // tune 0.03–0.06 if needed
    private static final double CENTER_POWER = 0.25; // max power for centering
    private static final double CENTER_TOLERANCE = 2.5; // degrees tolerance for center
    private static final double CENTER_KP = 0.012; // Proportional gain
    private static final double CENTER_KD = 0.008; // Derivative gain

    public TurretSubsystem(
            HardwareMap hw,
            String servoName,
            String encoderName,
            CameraSubsystem camera,
            TurretOdometrySubsystem turretOdom
    ) {
        turretServo = hw.crservo.get(servoName);

        turretEncoder = hw.get(DcMotorEx.class, encoderName);
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.camera = camera;
        this.turretOdom = turretOdom;

        // Safety: ensure odom subsystem doesn't fight for servo output (works with the version I gave you)
        if (this.turretOdom != null) {
            this.turretOdom.setOutputsEnabled(false);
        }

        Robot.getInstance().subsystems.add(this);
        lastNanos = System.nanoTime();
    }

    /* ================= State ================= */

    public void setTurretState(TurretState newState) {
        turretState = newState;
        resetPID();
        lastSource = TrackSource.NONE;

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
        // keep this exactly as you had it
        camera.setCurrentCameraYaw(getTurretAngleDeg());

        if (turretState == TurretState.TRACK) {
            runHybridTrackingPID();
        } else if (turretState == TurretState.CENTER) {
            lastSource = TrackSource.NONE;
            centerTurret();
        }
    }

    /* ================= Hybrid Tracking ================= */

    private void runHybridTrackingPID() {
        long now = System.nanoTime();
        double dt = clamp((now - lastNanos) / 1e9, MIN_DT, MAX_DT);
        lastNanos = now;

        // 1) If camera target is visible, do your original camera tracking behavior
        if (camera.hasBasket()) {
            if (lastSource != TrackSource.VISION) {
                resetPID(); // prevents spikes when switching sources
                lastSource = TrackSource.VISION;
            }
            runCameraTrackingPID(dt);
            return;
        }

        // 2) If camera target isn't visible, use odometry desired angle
        if (turretOdom != null && turretOdom.hasPose()) {
            if (lastSource != TrackSource.ODOM) {
                resetPID();
                lastSource = TrackSource.ODOM;
            }
            double desiredDeg = turretOdom.getDesiredTurretAngleDeg(); // relative-to-robot
            runOdomAngleHoldPID(dt, desiredDeg);
            return;
        }

        // 3) If no vision and no odom pose, center
        if (lastSource != TrackSource.NONE) {
            resetPID();
            lastSource = TrackSource.NONE;
        }
        centerTurret();
    }

    /* ================= Camera PID (UNCHANGED behavior) ================= */

    private void runCameraTrackingPID(double dt) {
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

        // derivative (skip on first measurement)
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

    /* ================= Odom fallback PID (new) ================= */

    private void runOdomAngleHoldPID(double dt, double desiredTurretAngleDeg) {
        double currentDeg = getTurretAngleDeg();

        // Use wrapped error so we choose the shortest direction
        double errDeg = normalizeAngle(desiredTurretAngleDeg - currentDeg);

        // CHANGE TO THIS IF DOESNT WORK: double errDeg = normalizeAngle(currentDeg - desiredTurretAngleDeg);

        if (Math.abs(errDeg) < Constants.deadbandDeg) {
            setTurretPower(0.0);
            integral = 0.0;
            return;
        }

        errFiltDeg =
                Constants.errAlpha * errDeg +
                        (1.0 - Constants.errAlpha) * errFiltDeg;

        integral += errFiltDeg * dt;
        integral = clamp(integral,
                -Constants.maxIntegral,
                Constants.maxIntegral);

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

        if (Math.abs(error) < CENTER_TOLERANCE) {
            setTurretPower(0.0);
            firstCenterMeasurement = true;
            return;
        }

        double derivative = 0.0;
        if (!firstCenterMeasurement) {
            derivative = (currentAngle - lastCenterAngle) / dt;
        }
        firstCenterMeasurement = false;
        lastCenterAngle = currentAngle;

        double power = (CENTER_KP * error) - (CENTER_KD * derivative);
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

    private static double normalizeAngle(double angleDeg) {
        angleDeg = angleDeg % 360.0;
        if (angleDeg > 180.0) {
            angleDeg -= 360.0;
        } else if (angleDeg < -180.0) {
            angleDeg += 360.0;
        }
        return angleDeg;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
