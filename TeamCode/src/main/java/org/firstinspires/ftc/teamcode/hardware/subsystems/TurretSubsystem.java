package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
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

    public enum TurretState {
        MANUAL,
        AUTO_AIM
    }

    private TurretState turretState;

    private static final double MIN_DT = 1e-3;  // 1 ms
    private static final double MAX_DT = 0.05;  // 50 ms

    private static final double ENCODER_TICKS_PER_REV = 8192.0;

    private static final double GEAR_RATIO = 1.0;

    private static final double TICKS_PER_DEGREE = (ENCODER_TICKS_PER_REV * GEAR_RATIO) / 360.0;

    private static final double LEFT_LIMIT_DEG = 0.0;
    private static final double RIGHT_LIMIT_DEG = 355.0;

    // Alternative options based on your needs:
    // Full coverage with some wraparound: -30° to 390°
    // Safe with lots of slack: -90° to 450°
    // Conservative 270° range: 45° to 315°

    private static final double POSITION_TOLERANCE_DEG = 2.0;

    private double targetAngleDeg = 180.0;  // Start centered

    public TurretSubsystem(HardwareMap hw, String servoName, String encoderName, CameraSubsystem camera) {
        this.turretServo = hw.crservo.get(servoName);

        this.turretEncoder = hw.get(DcMotorEx.class, encoderName);


        this.camera = camera;

        turretState = TurretState.MANUAL;

        // Set initial target to current position
        targetAngleDeg = getTurretAngleDeg();

        Robot.getInstance().subsystems.add(this);
        lastNanos = System.nanoTime();
    }

    public void setTurretState(TurretState newstate) {
        this.turretState = newstate;
        if (newstate == TurretState.MANUAL) {
            setTurretPower(0.0);
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

    public void setTurretPower(double power) {
        double currentAngle = getTurretAngleDeg();

        // Enforce hard limits to protect cables
        if (currentAngle <= LEFT_LIMIT_DEG && power < 0) {
            power = 0.0;  // At left limit, prevent further left rotation
        } else if (currentAngle >= RIGHT_LIMIT_DEG && power > 0) {
            power = 0.0;  // At right limit, prevent further right rotation
        }

        turretServo.setPower(clamp(power, -1.0, 1.0));
    }

    public boolean isWithinLimits() {
        double angle = getTurretAngleDeg();
        return angle >= LEFT_LIMIT_DEG && angle <= RIGHT_LIMIT_DEG;
    }

    public boolean isNearLimit(double thresholdDeg) {
        double angle = getTurretAngleDeg();
        return (angle - LEFT_LIMIT_DEG < thresholdDeg) ||
                (RIGHT_LIMIT_DEG - angle < thresholdDeg);
    }

    private double getShortestAngleDistance(double from, double to) {
        double diff = to - from;

        // Normalize to -180 to +180 range
        while (diff > 180.0) diff -= 360.0;
        while (diff < -180.0) diff += 360.0;

        return diff;
    }

    private double normalizeAngle(double angleDeg) {
        double normalized = angleDeg % 360.0;
        if (normalized < 0) normalized += 360.0;
        return normalized;
    }

    public double getTurretAngleDeg() {
        return turretEncoder.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public double getTargetAngleDeg() {
        return targetAngleDeg;
    }

    public void setTargetAngle(double angleDeg) {
        targetAngleDeg = clamp(angleDeg, LEFT_LIMIT_DEG, RIGHT_LIMIT_DEG);
    }

    public double getLeftLimitDeg() {
        return LEFT_LIMIT_DEG;
    }

    public double getRightLimitDeg() {
        return RIGHT_LIMIT_DEG;
    }

    public boolean isAtTarget(double toleranceDeg) {
        return Math.abs(getTurretAngleDeg() - targetAngleDeg) < toleranceDeg;
    }


    public boolean isAtTarget() {
        return isAtTarget(POSITION_TOLERANCE_DEG);
    }


    public void resetEncoder() {

    }

    @Override
    public void updateData() {
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

        if (!camera.hasBasket()) {
            setTurretPower(0.0);
            targetAngleDeg = getTurretAngleDeg();
            lastErrDeg = 0.0;
            return;
        }

        double currentAngle = getTurretAngleDeg();

        double yawErrDeg = camera.getBasketYawDeg();

        boolean atRightLimit = currentAngle >= RIGHT_LIMIT_DEG - 5.0;
        boolean atLeftLimit = currentAngle <= LEFT_LIMIT_DEG + 5.0;

        if (atRightLimit && yawErrDeg > 0) {

            yawErrDeg = -(360.0 - Math.abs(yawErrDeg));
        } else if (atLeftLimit && yawErrDeg < 0) {

            yawErrDeg = 360.0 - Math.abs(yawErrDeg);
        }

        if (Math.abs(yawErrDeg) < Constants.deadbandDeg) {
            yawErrDeg = 0.0;
            setTurretPower(0.0);
            return;
        }

        errFiltDeg = Constants.errAlpha * yawErrDeg + (1.0 - Constants.errAlpha) * errFiltDeg;

        integral += errFiltDeg * dt;
        if (yawErrDeg == 0.0) integral *= 0.5;
        integral = clamp(integral, -Constants.maxIntegral, Constants.maxIntegral);

        double deriv = (errFiltDeg - lastErrDeg) / dt;
        deriv = clamp(deriv, -Constants.maxDeriv, Constants.maxDeriv);
        lastErrDeg = errFiltDeg;

        double rawPower =
                Constants.kP_v * errFiltDeg +
                        Constants.kI_v * integral +
                        Constants.kD_v * deriv;

        if (yawErrDeg != 0.0 && Constants.kS > 0) {
            rawPower += Math.signum(errFiltDeg) * Constants.kS;
        }

        double maxPower = clamp(Constants.maxPower, 0.0, 1.0);
        double power = clamp(rawPower, -maxPower, maxPower);

        if (Math.abs(power) >= maxPower - 1e-6 && Math.signum(power) == Math.signum(rawPower)) {
            integral *= 0.95;
        }

        targetAngleDeg = currentAngle - errFiltDeg;

        setTurretPower(power);
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