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

public class TurretOdometrySubsystem extends RE_SubsystemBase {

    private final CRServo turretServo;
    private final DcMotorEx turretEncoder;
    private final Follower follower;

    private double targetX;
    private double targetY;

    private final double turretOffsetX = -55.603/25.4;
    private final double turretOffsetY = 0;

    private double integral = 0.0;
    private double lastErrDeg = 0.0;
    private double errFiltDeg = 0.0;
    private long lastNanos = 0L;

    private double lastSetPower = 0.0;

    public enum TurretState {
        MANUAL,
        TRACK_POINT
    }

    private TurretState turretState;

    private static final double MIN_DT = 1e-3;
    private static final double MAX_DT = 0.05;

    private static final double ENCODER_TICKS_PER_REV = 8192.0;     // REV Through Bore
    private static final double ENCODER_REV_PER_TURRET_REV = 102.0 / 45.0;

    private static final double ticksPerDeg =
            (ENCODER_TICKS_PER_REV * ENCODER_REV_PER_TURRET_REV) / 360.0;

    private static final double leftlim = -180;
    private static final double rightlim = 180;

    public TurretOdometrySubsystem(HardwareMap hw, String servoName, String encoderName, Follower follower) {
        this.turretServo = hw.get(CRServo.class, servoName);
        this.turretEncoder = hw.get(DcMotorEx.class, encoderName);
        this.follower = follower;

        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretState = TurretState.MANUAL;
        setTurretPower(0.0);

        if (ALLIANCE == Globals.COLORS.BLUE) {
            targetX = 0;
            targetY = 144;
        } else if (ALLIANCE == Globals.COLORS.RED) {
            targetX = 144;
            targetY = 144;
        }

        Robot.getInstance().subsystems.add(this);
        lastNanos = System.nanoTime();
    }

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

    public double getRawTicks() {
        return (double) turretEncoder.getCurrentPosition();
    }

    public void setTargetPoint(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    public void zeroTurretEncoder() {
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetPID();
    }

    public void setTurretPower(double pwr) {
        double angleDeg = getTurretAngleDeg();

        if (angleDeg <= leftlim && pwr < 0) {
            pwr = 0;
        } else if (angleDeg >= rightlim && pwr > 0) {
            pwr = 0;
        }

        lastSetPower = clamp(pwr, -1.0, 1.0);
        turretServo.setPower(lastSetPower);
    }

    public double getTurretAngleDeg() {
        return turretEncoder.getCurrentPosition() / ticksPerDeg;
    }
    public double getDesiredTurretAngleDeg() {
        Pose pose = follower.getPose();
        if (pose == null) return 0.0;

        double robotX = pose.getX();
        double robotY = pose.getY();
        double heading = pose.getHeading();

        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        double turretX = robotX + turretOffsetX * cosH - turretOffsetY * sinH;
        double turretY = robotY + turretOffsetX * sinH + turretOffsetY * cosH;

        double angleToTargetField =
                Math.atan2(targetY - turretY, targetX - turretX);

        double desiredDeg =
                Math.toDegrees(angleToTargetField - heading);

        return normalizeAngle(desiredDeg);
    }

    public double getTurretErrorDeg() {
        double desired = getDesiredTurretAngleDeg();
        double current = getTurretAngleDeg();

        return calculateSmartError(desired, current);
    }
    public double getTurretPower() {
        return lastSetPower;
    }

    @Override
    public void updateData() {
        // Robot.getInstance().data.turretState = turretState.name();
        // Robot.getInstance().data.turretAngleDeg = getTurretAngleDeg();
        // Robot.getInstance().data.turretTargetX = targetX;
        // Robot.getInstance().data.turretTargetY = targetY;
    }

    @Override
    public void periodic() {
        if (turretState == TurretState.TRACK_POINT) {
            runTrackPointPID();
        }
    }

    private void runTrackPointPID() {
        long now = System.nanoTime();
        double dt = (now - lastNanos) / 1e9;
        lastNanos = now;

        dt = clamp(dt, MIN_DT, MAX_DT);

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

        double desiredTurretAngleDeg = Math.toDegrees(angleToTargetField - heading);
        desiredTurretAngleDeg = normalizeAngle(desiredTurretAngleDeg);

        double currentTurretAngleDeg = getTurretAngleDeg();

        double errDeg = calculateSmartError(desiredTurretAngleDeg, currentTurretAngleDeg);

        if (Math.abs(errDeg) < Constants.deadbandDeg) {
            errDeg = 0.0;
        }

        errFiltDeg = Constants.errAlpha * errDeg + (1.0 - Constants.errAlpha) * errFiltDeg;

        integral += errFiltDeg * dt;
        if (Math.abs(errDeg) < Constants.deadbandDeg) {
            integral *= 0.5;
        }
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