package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.util.Globals.ALLIANCE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotData;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

public class TurretOdometrySubsystem extends RE_SubsystemBase {

    private final CRServo turretServo;
    private final DcMotorEx turretEncoder;
    private final Follower follower;

    private double targetX;
    private double targetY;

    // Offsets are in ROBOT coords: +X right, +Y forward (inches)
    // Your value is negative => turret is left of robot origin
    private final double turretOffsetX = 55.603 / 25.4;
    private final double turretOffsetY = 0.0;

    // ----- PID state -----
    private double integral = 0.0;
    private double lastErrDeg = 0.0;
    private double errFiltDeg = 0.0;
    private long lastNanos = 0L;

    private double lastSetPower = 0.0;

    public enum TurretState {
        MANUAL,
        TRACK_POINT,
        RETURN_TO_FRONT,
        RETURN_TO_START
    }

    private TurretState turretState;

    private static final double MIN_DT = 1e-3;
    private static final double MAX_DT = 0.05;

    // Encoder constants
    private static final double ENCODER_TICKS_PER_REV = 8192.0;     // REV Through Bore
    private static final double ENCODER_REV_PER_TURRET_REV = 102.0 / 45.0;
    private static final double TICKS_PER_DEG =
            (ENCODER_TICKS_PER_REV * ENCODER_REV_PER_TURRET_REV) / 360.0;

    // --------- Turret angle convention ----------
    // turretRelDeg = 0 means turret points forward with robot
    // +90 means turret points left of robot forward
    private static final double START_REL_DEG = 90.0;

    // Constraint: turret cannot rotate more than ±180° from its start
    private static final double LEFT_LIM_DEG  = START_REL_DEG - 180.0;  // -90
    private static final double RIGHT_LIM_DEG = START_REL_DEG + 170.0;  // 270

    // Encoder-to-angle calibration offset:
    // turretAngleDeg = ticks/TICKS_PER_DEG + turretZeroDeg
    // After you call zeroTurretEncoderAtStart(), ticks=0 corresponds to START_REL_DEG.
    private double turretZeroDeg = START_REL_DEG;

    public TurretOdometrySubsystem(HardwareMap hw, String servoName, String encoderName, Follower follower) {
        this.turretServo = hw.get(CRServo.class, servoName);
        this.turretEncoder = hw.get(DcMotorEx.class, encoderName);
        this.follower = follower;

        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // correct: we only READ ticks

        turretState = TurretState.MANUAL;
        setTurretPower(0.0);

        if (ALLIANCE == Globals.COLORS.BLUE) {
            targetX = 13;
            targetY = 136.4;
        } else if (ALLIANCE == Globals.COLORS.RED) {
            targetX = 131;
            targetY = 136.4;
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

    public void setTargetPoint(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    /** Raw encoder ticks (sign-adjusted if your belt/gear direction requires it). */
    public double getRawTicks() {
        // Keep this negate only if it matches your hardware direction.
        // If your turret runs away in the wrong direction, flip THIS sign OR flip the sign in setTurretPower(), not both.
        return -(double) turretEncoder.getCurrentPosition();
    }

    /** Call when turret is at any known angle to calibrate the encoder */
    public void calibrateTurretAt(double currentAngleDeg) {
        double currentTicks = getRawTicks();
        turretZeroDeg = currentAngleDeg - (currentTicks / TICKS_PER_DEG);
        resetPID();
    }

    /**
     * Call when the turret is physically at the START orientation (90° left of robot forward).
     * This resets the encoder to zero and sets the zero offset.
     */
    public void calibrateTurretAtStart() {
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretZeroDeg = START_REL_DEG;
        resetPID();
    }

    /**
     * Deprecated: Use calibrateTurretAtStart() instead for clarity.
     * Call when the turret is physically at the START orientation (90° left of robot forward).
     */
    @Deprecated
    public void zeroTurretEncoderAtStart() {
        calibrateTurretAtStart();
    }

    /** Current turret angle in continuous degrees (should stay within [-90, 270] if obeying limits). */
    public double getTurretAngleDeg() {
        return (getRawTicks() / TICKS_PER_DEG) + turretZeroDeg;
    }

    public double getTurretPower() {
        return lastSetPower;
    }

    /** Sets turret power while enforcing hard angle limits. */
    public void setTurretPower(double pwr) {
        double angleDeg = getTurretAngleDeg();

        // No negation here - belt inversion handled in getRawTicks()
        double servoPwr = clamp(pwr, -1.0, 1.0);

        // Gate based on the REAL direction
        if (angleDeg <= LEFT_LIM_DEG && servoPwr < 0) servoPwr = 0;
        if (angleDeg >= RIGHT_LIM_DEG && servoPwr > 0) servoPwr = 0;

        lastSetPower = servoPwr;
        turretServo.setPower(servoPwr);
    }

    /** Desired turret angle (continuous) that aims at the target point, respecting limits. */
    public double getDesiredTurretAngleDeg() {
        if (turretState == TurretState.RETURN_TO_FRONT) {
            return 0.0;
        }

        if (turretState == TurretState.RETURN_TO_START) {
            return START_REL_DEG;
        }

        Pose pose = follower.getPose();
        if (pose == null) return START_REL_DEG;

        double robotX = pose.getX();
        double robotY = pose.getY();
        double heading = pose.getHeading(); // radians, 0 along +X, CCW+

        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        // Robot axes: +X right, +Y forward
        // Field axes: +X right, +Y forward
        //
        // Unit vectors in field:
        // forward = (cosH, sinH)
        // right   = (sinH, -cosH)
        //
        // pivot = robot + (offsetRight * right) + (offsetForward * forward)
        double turretX = robotX + turretOffsetX * sinH + turretOffsetY * cosH;
        double turretY = robotY - turretOffsetX * cosH + turretOffsetY * sinH;

        double angleToTargetField = Math.atan2(targetY - turretY, targetX - turretX);

        // Raw relative turret angle (deg): turretRel = fieldAngle - robotHeading
        double rawRelDeg = Math.toDegrees(angleToTargetField - heading);

        // Wrap to (-180,180] for a base representation...
        rawRelDeg = wrap180(rawRelDeg);

        // ...then pick the equivalent angle closest to current turret angle (continuous behavior)
        double currentDeg = getTurretAngleDeg();
        double desiredDeg = nearestEquivalent(rawRelDeg, currentDeg);

        // Enforce ±180 from start constraint
        desiredDeg = clamp(desiredDeg, LEFT_LIM_DEG, RIGHT_LIM_DEG);

        return desiredDeg;
    }

    /** Error in degrees (continuous). */
    public double getTurretErrorDeg() {
        double desired = getDesiredTurretAngleDeg();
        double current = getTurretAngleDeg();
        return desired - current;
    }

    @Override
    public void updateData() {
        RobotData data = Robot.getInstance().data;

        data.turretState = turretState;

        data.turretAngleDeg = getTurretAngleDeg();
        data.turretDesiredDeg = getDesiredTurretAngleDeg();
        data.turretErrorDeg = getTurretErrorDeg();
        data.turretServoPower = getTurretPower();

        data.turretTargetX = targetX;
        data.turretTargetY = targetY;

        Pose pose = follower.getPose();
        if (pose == null) return;

        double robotX = pose.getX();
        double robotY = pose.getY();
        double heading = pose.getHeading();

        data.robotHeadingDeg = Math.toDegrees(heading);

        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        double turretX = robotX + turretOffsetX * sinH + turretOffsetY * cosH;
        double turretY = robotY - turretOffsetX * cosH + turretOffsetY * sinH;

        data.turretPivotX = turretX;
        data.turretPivotY = turretY;

        double angleToTargetField =
                Math.atan2(targetY - turretY, targetX - turretX);

        data.angleToTargetFieldDeg = Math.toDegrees(angleToTargetField);

        double rawRelDeg = Math.toDegrees(angleToTargetField - heading);
        data.turretRawRelDeg = wrap180(rawRelDeg);
    }

    @Override
    public void periodic() {
        if (turretState == TurretState.TRACK_POINT ||
                turretState == TurretState.RETURN_TO_FRONT ||
                turretState == TurretState.RETURN_TO_START) {
            runTrackPointPID();
        }
        // Robot.getInstance().cameraSubsystem.setCurrentCameraYaw(getTurretAngleDeg());
    }

    private void runTrackPointPID() {
        long now = System.nanoTime();
        double dt = (now - lastNanos) / 1e9;
        lastNanos = now;

        dt = clamp(dt, MIN_DT, MAX_DT);

        Pose pose = follower.getPose();
        if (pose == null && turretState == TurretState.TRACK_POINT) {
            setTurretPower(0.0);
            return;
        }

        double desiredDeg = getDesiredTurretAngleDeg();
        double currentDeg = getTurretAngleDeg();
        double errDeg = desiredDeg - currentDeg;

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

        // Anti-windup if saturated pushing same direction as error
        if (Math.abs(power) >= maxPower - 1e-6 && Math.signum(power) == Math.signum(errFiltDeg)) {
            integral *= 0.9;
        }

        setTurretPower(power);
    }

    private void resetPID() {
        integral = 0.0;
        lastErrDeg = 0.0;
        errFiltDeg = 0.0;
        lastNanos = System.nanoTime();
    }

    // --- Angle helpers ---

    private static double wrap180(double deg) {
        deg %= 360.0;
        if (deg <= -180.0) deg += 360.0;
        if (deg > 180.0) deg -= 360.0;
        return deg;
    }

    /** Returns angleDeg + 360k that is closest to referenceDeg */
    private static double nearestEquivalent(double angleDeg, double referenceDeg) {
        return angleDeg + 360.0 * Math.round((referenceDeg - angleDeg) / 360.0);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}