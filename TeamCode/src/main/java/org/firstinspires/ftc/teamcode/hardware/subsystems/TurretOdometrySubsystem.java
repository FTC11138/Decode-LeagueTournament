package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.util.Globals.ALLIANCE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

/**
 * TurretOdometrySubsystem can do two things:
 *  1) Provide desired turret angle from odometry: getDesiredTurretAngleDeg()
 *  2) (Optionally) drive the turret itself with its own PID states.
 *
 * To prevent "servo fighting" with TurretSubsystem:
 *  - outputsEnabled defaults to FALSE (so this subsystem will not command the servo unless you enable it).
 *  - When using TurretSubsystem as the actuator, just call turretOdom.setOutputsEnabled(false) once.
 */
public class TurretOdometrySubsystem extends RE_SubsystemBase {

    private final CRServo turretServo;
    private final DcMotorEx turretEncoder;
    private final Follower follower;

    private double targetX;
    private double targetY;

    // Offset from robot center to turret pivot, in inches (your original)
    private final double turretOffsetX = -55.603 / 25.4;
    private final double turretOffsetY = 0;

    // PID state (only used if outputsEnabled == true AND you set a tracking state)
    private double integral = 0.0;
    private double lastErrDeg = 0.0;
    private double errFiltDeg = 0.0;
    private long lastNanos = 0L;
    private double lastSetPower = 0.0;

    // IMPORTANT: prevents this subsystem from commanding the servo unless you explicitly enable it
    private boolean outputsEnabled = false;

    public enum TurretState {
        MANUAL,
        TRACK_POINT,
        CENTER,
        AUTOANGLE
    }

    private TurretState turretState;

    private static final double MIN_DT = 1e-3;
    private static final double MAX_DT = 0.05;

    private static final double ENCODER_TICKS_PER_REV = 8192.0;     // REV Through Bore
    private static final double ENCODER_REV_PER_TURRET_REV = 102.0 / 45.0;

    private static final double ticksPerDeg =
            (ENCODER_TICKS_PER_REV * ENCODER_REV_PER_TURRET_REV) / 360.0;

    // These limits only matter if THIS subsystem is driving the servo.
    // If TurretSubsystem drives the servo, TurretSubsystem limits are the ones that matter.
    private static final double leftlim = -90;
    private static final double rightlim = 90;

    public TurretOdometrySubsystem(HardwareMap hw, String servoName, String encoderName, Follower follower) {
        this.turretServo = hw.get(CRServo.class, servoName);
        this.turretEncoder = hw.get(DcMotorEx.class, encoderName);
        this.follower = follower;

        if (Globals.IS_AUTO) {
            turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretState = TurretState.MANUAL;
        setTurretPower(0.0);

        // Default target point
        if (ALLIANCE == Globals.COLORS.BLUE) {
            targetX = 0;
            targetY = 144;
        } else {
            targetX = 137;
            targetY = 137;
        }

        Robot.getInstance().subsystems.add(this);
        lastNanos = System.nanoTime();
    }

    /* ================= Output gate (prevents servo fighting) ================= */

    /** If false, periodic() will not command the servo and will force power to 0. */
    public void setOutputsEnabled(boolean enabled) {
        outputsEnabled = enabled;
        if (!enabled) {
            turretState = TurretState.MANUAL;
            setTurretPower(0.0);
            resetPID();
        }
    }

    public boolean getOutputsEnabled() {
        return outputsEnabled;
    }

    /* ================= State ================= */

    public void setTurretState(TurretState state) {
        this.turretState = state;
        if (state == TurretState.MANUAL) {
            setTurretPower(0.0);
        } else {
            resetPID();
        }
    }

    public TurretState getTurretState() {
        return turretState;
    }

    /* ================= Pose validity ================= */

    public boolean hasPose() {
        return follower.getPose() != null;
    }

    /* ================= Encoder ================= */

    public double getRawTicks() {
        // Negate for belt drive (motor and turret rotate same direction)
        return -(double) turretEncoder.getCurrentPosition();
    }

    public double getTurretAngleDeg() {
        return getRawTicks() / ticksPerDeg;
    }

    public void zeroTurretEncoder() {
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetPID();
    }

    /* ================= Targeting ================= */

    public void setTargetPoint(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    public double getDist() {
        Pose pose = follower.getPose();
        if (pose == null) return 0.0;

        double x = pose.getX();
        double y = pose.getY();
        double tx, ty;

        if (Globals.ALLIANCE == Globals.COLORS.BLUE) {
            tx = 17;
            ty = 132.5;
        } else {
            tx = 127;
            ty = 132.5;
        }

        return Math.abs(Math.sqrt(Math.pow((tx - x), 2) + Math.pow((ty - y), 2)));
    }

    /**
     * The important method for your hybrid logic:
     * Desired turret angle relative to the robot (degrees), using follower pose + turret offset.
     */
    public double getDesiredTurretAngleDeg() {
        Pose pose = follower.getPose();
        if (pose == null) return 0.0;

        double robotX = pose.getX();
        double robotY = pose.getY();
        double heading = pose.getHeading(); // radians

        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        // turret pivot in field coords
        double turretX = robotX + turretOffsetX * cosH - turretOffsetY * sinH;
        double turretY = robotY + turretOffsetX * sinH + turretOffsetY * cosH;

        double angleToTargetField = Math.atan2(targetY - turretY, targetX - turretX);

        // desired turret angle relative to robot heading
        double desiredDeg = Math.toDegrees(angleToTargetField - heading);
        return normalizeAngle(desiredDeg);
    }

    public double getTurretErrorDeg() {
        double desired = getDesiredTurretAngleDeg();
        double current = getTurretAngleDeg();
        return calculateSmartError(desired, current);
    }

    /* ================= Servo output ================= */

    public void setTurretPower(double pwr) {
        double angleDeg = getTurretAngleDeg();

        if (angleDeg <= leftlim && pwr < 0) {
            pwr = 0;
        } else if (angleDeg >= rightlim && pwr > 0) {
            pwr = 0;
        }

        // Negate power for belt drive
        lastSetPower = clamp(-pwr, -1.0, 1.0);
        turretServo.setPower(lastSetPower);
    }

    public double getTurretPower() {
        return lastSetPower;
    }

    /* ================= Telemetry ================= */

    @Override
    public void updateData() {
        Robot robot = Robot.getInstance();

        robot.data.turretState = turretState;
        robot.data.turretTargetX = targetX;
        robot.data.turretTargetY = targetY;

        robot.data.turretAngleDeg = getTurretAngleDeg();
        robot.data.turretServoPower = lastSetPower;

        Pose pose = follower.getPose();
        if (pose == null) return;

        double robotX = pose.getX();
        double robotY = pose.getY();
        double heading = pose.getHeading();

        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        double turretX = robotX + turretOffsetX * cosH - turretOffsetY * sinH;
        double turretY = robotY + turretOffsetX * sinH + turretOffsetY * cosH;

        robot.data.turretPivotX = turretX;
        robot.data.turretPivotY = turretY;

        robot.data.robotHeadingDeg = Math.toDegrees(heading);

        double angleToTargetFieldRad = Math.atan2(targetY - turretY, targetX - turretX);
        robot.data.angleToTargetFieldDeg = Math.toDegrees(angleToTargetFieldRad);

        robot.data.turretRawRelDeg =
                normalizeAngle(robot.data.angleToTargetFieldDeg - robot.data.robotHeadingDeg);

        robot.data.turretDesiredDeg = getDesiredTurretAngleDeg();
        robot.data.turretErrorDeg =
                calculateSmartError(robot.data.turretDesiredDeg, robot.data.turretAngleDeg);

        robot.data.dist = getDist();
    }

    /* ================= Loop ================= */

    @Override
    public void periodic() {
        // If TurretSubsystem is the one driving the servo, leave outputs disabled.
        if (!outputsEnabled) {
            // make sure we never accidentally keep a nonzero output around
            if (turretState != TurretState.MANUAL || Math.abs(lastSetPower) > 1e-6) {
                turretState = TurretState.MANUAL;
                setTurretPower(0.0);
            }
            return;
        }

        if (turretState == TurretState.TRACK_POINT) {
            runTrackPointPID();
        } else if (turretState == TurretState.CENTER) {
            runCenterPID();
        } else if (turretState == TurretState.AUTOANGLE) {
            runAutoAnglePID();
        }
    }

    /* ================= PID (only when outputsEnabled == true) ================= */

    private void runTrackPointPID() {
        long now = System.nanoTime();
        double dt = clamp((now - lastNanos) / 1e9, MIN_DT, MAX_DT);
        lastNanos = now;

        Pose pose = follower.getPose();
        if (pose == null) {
            setTurretPower(0.0);
            return;
        }

        double robotX = pose.getX();
        double robotY = pose.getY();
        double heading = pose.getHeading();

        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        double turretX = robotX + turretOffsetX * cosH - turretOffsetY * sinH;
        double turretY = robotY + turretOffsetX * sinH + turretOffsetY * cosH;

        double angleToTargetField = Math.atan2(targetY - turretY, targetX - turretX);

        double desiredTurretAngleDeg = normalizeAngle(Math.toDegrees(angleToTargetField - heading));
        double currentTurretAngleDeg = getTurretAngleDeg();

        double errDeg = calculateSmartError(desiredTurretAngleDeg, currentTurretAngleDeg);

        if (Math.abs(errDeg) < Constants.deadbandDeg) errDeg = 0.0;

        errFiltDeg = Constants.errAlpha * errDeg + (1.0 - Constants.errAlpha) * errFiltDeg;

        integral += errFiltDeg * dt;
        if (Math.abs(errDeg) < Constants.deadbandDeg) integral *= 0.5;
        integral = clamp(integral, -Constants.maxIntegral, Constants.maxIntegral);

        double deriv = (errFiltDeg - lastErrDeg) / dt;
        deriv = clamp(deriv, -Constants.maxDeriv, Constants.maxDeriv);
        lastErrDeg = errFiltDeg;

        double rawPower =
                Constants.kP_v * errFiltDeg +
                        Constants.kI_v * integral +
                        Constants.kD_v * deriv;

        if (Math.abs(errDeg) > Constants.deadbandDeg && Constants.kS > 0) {
            rawPower += Math.signum(errFiltDeg) * Constants.kS;
        }

        double maxPower = clamp(Constants.maxPower, 0.0, 1.0);
        double power = clamp(rawPower, -maxPower, maxPower);

        if (Math.abs(power) >= maxPower - 1e-6 && Math.signum(power) == Math.signum(errFiltDeg)) {
            integral *= 0.9;
        }

        setTurretPower(power);
    }

    private void runCenterPID() {
        long now = System.nanoTime();
        double dt = clamp((now - lastNanos) / 1e9, MIN_DT, MAX_DT);
        lastNanos = now;

        double desiredTurretAngleDeg = 0.0;
        double currentTurretAngleDeg = getTurretAngleDeg();

        double errDeg = calculateSmartError(desiredTurretAngleDeg, currentTurretAngleDeg);
        if (Math.abs(errDeg) < Constants.deadbandDeg) errDeg = 0.0;

        errFiltDeg = Constants.errAlpha * errDeg + (1.0 - Constants.errAlpha) * errFiltDeg;

        integral += errFiltDeg * dt;
        if (Math.abs(errDeg) < Constants.deadbandDeg) integral *= 0.5;
        integral = clamp(integral, -Constants.maxIntegral, Constants.maxIntegral);

        double deriv = (errFiltDeg - lastErrDeg) / dt;
        deriv = clamp(deriv, -Constants.maxDeriv, Constants.maxDeriv);
        lastErrDeg = errFiltDeg;

        double rawPower =
                Constants.kP_v * errFiltDeg +
                        Constants.kI_v * integral +
                        Constants.kD_v * deriv;

        if (Math.abs(errDeg) > Constants.deadbandDeg && Constants.kS > 0) {
            rawPower += Math.signum(errFiltDeg) * Constants.kS;
        }

        double maxPower = clamp(Constants.maxPower, 0.0, 1.0);
        double power = clamp(rawPower, -maxPower, maxPower);

        if (Math.abs(power) >= maxPower - 1e-6 && Math.signum(power) == Math.signum(errFiltDeg)) {
            integral *= 0.9;
        }

        setTurretPower(power);
    }

    private void runAutoAnglePID() {
        long now = System.nanoTime();
        double dt = clamp((now - lastNanos) / 1e9, MIN_DT, MAX_DT);
        lastNanos = now;

        double desiredTurretAngleDeg = -55.0; // your hardcoded auto angle
        double currentTurretAngleDeg = getTurretAngleDeg();

        double errDeg = calculateSmartError(desiredTurretAngleDeg, currentTurretAngleDeg);
        if (Math.abs(errDeg) < Constants.deadbandDeg) errDeg = 0.0;

        errFiltDeg = Constants.errAlpha * errDeg + (1.0 - Constants.errAlpha) * errFiltDeg;

        integral += errFiltDeg * dt;
        if (Math.abs(errDeg) < Constants.deadbandDeg) integral *= 0.5;
        integral = clamp(integral, -Constants.maxIntegral, Constants.maxIntegral);

        double deriv = (errFiltDeg - lastErrDeg) / dt;
        deriv = clamp(deriv, -Constants.maxDeriv, Constants.maxDeriv);
        lastErrDeg = errFiltDeg;

        double rawPower =
                Constants.kP_v * errFiltDeg +
                        Constants.kI_v * integral +
                        Constants.kD_v * deriv;

        if (Math.abs(errDeg) > Constants.deadbandDeg && Constants.kS > 0) {
            rawPower += Math.signum(errFiltDeg) * Constants.kS;
        }

        double maxPower = clamp(Constants.maxPower, 0.0, 1.0);
        double power = clamp(rawPower, -maxPower, maxPower);

        if (Math.abs(power) >= maxPower - 1e-6 && Math.signum(power) == Math.signum(errFiltDeg)) {
            integral *= 0.9;
        }

        setTurretPower(power);
    }

    /* ================= Helpers ================= */

    private double calculateSmartError(double desired, double current) {
        desired = normalizeAngle(desired);
        current = normalizeAngle(current);

        desired = clamp(desired, leftlim, rightlim);

        if (current < leftlim) {
            return leftlim - current;
        } else if (current > rightlim) {
            return rightlim - current;
        }

        double error = desired - current;
        double targetPos = current + error;

        if (targetPos > rightlim) {
            error = rightlim - current;
        } else if (targetPos < leftlim) {
            error = leftlim - current;
        }

        return error;
    }

    private void resetPID() {
        integral = 0.0;
        lastErrDeg = 0.0;
        errFiltDeg = 0.0;
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
