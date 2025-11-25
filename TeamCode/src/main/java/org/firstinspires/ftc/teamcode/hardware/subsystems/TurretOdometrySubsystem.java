package org.firstinspires.ftc.teamcode.hardware.subsystems;

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
    
    private final DcMotorEx turretMotor;
    private final Follower follower;

    private double targetX;
    private double targetY;


    //Offset from center of crobot
    private final double turretOffsetX = 0; // change when we find out
    private final double turretOffsetY = 0;


    private double integral = 0.0;
    private double lastErrDeg = 0.0;
    private double errFiltDeg = 0.0;
    private long lastNanos = 0L;

    public enum TurretState {
        MANUAL,
        TRACK_POINT
    }

    private TurretState turretState;

    private static final double MIN_DT = 1e-3;  // 1 ms
    private static final double MAX_DT = 0.05;  // 50 ms


    private static final double ticksPerRev = 1440.0;
    private static final double gearRatio = 108.0 / 258.0;
    private static final double ticksPerDeg = (ticksPerRev * gearRatio) / 360.0;


    private static final double leftlim = -90.0;
    private static final double rightlim = 90.0;

    public TurretOdometrySubsystem(HardwareMap hw, String motorName, Follower follower) {
        this.turretMotor = hw.get(DcMotorEx.class, motorName);
        this.follower = follower;


        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretState = TurretState.MANUAL;
        setTurretPower(0.0);

        if(ALLIANCE == Globals.COLORS.BLUE){
            targetX = 16.5;
            targetY = 131;
        }
        else if(ALLIANCE == Globals.COLORS.RED){
            targetX = 144 - 16.5;
            targetY = 131;
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

    public void setTurretPower(double pwr) {
        double angleDeg = getTurretAngleDeg();

        if (angleDeg <= leftlim && pwr < 0) {
            pwr = 0;
        } else if (angleDeg >= rightlim && pwr > 0) {
            pwr = 0;
        }

        turretMotor.setPower(clamp(pwr, -1.0, 1.0));
    }

    public double getTurretAngleDeg() {
        return turretMotor.getCurrentPosition() / ticksPerDeg;
    }

    public void setTargetPoint(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    @Override
    public void updateData() {
//        Robot.getInstance().data.turretState = turretState.name();
//        Robot.getInstance().data.turretAngleDeg = getTurretAngleDeg();
//        Robot.getInstance().data.turretTargetX = targetX;
//        Robot.getInstance().data.turretTargetY = targetY;
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

        if (dt < MIN_DT) dt = MIN_DT;
        if (dt > MAX_DT) dt = MAX_DT;

        Pose pose = follower.getPose();

        if (pose == null) {
            setTurretPower(0.0);
            lastErrDeg = 0.0;
            integral = 0.0;
            return;
        }

        double robotX = pose.getX();
        double robotY = pose.getY();
        double heading = pose.getHeading();

        // Turret position in field coords: robot pose + rotated offset
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        double turretX = robotX + turretOffsetX * cosH - turretOffsetY * sinH;
        double turretY = robotY + turretOffsetX * sinH + turretOffsetY * cosH;

        // Angle from turret to target in field frame
        double angleToTargetField = Math.atan2(targetY - turretY, targetX - turretX);

        // Desired turret angle relative to robot forward
        double desiredTurretAngleDeg = Math.toDegrees(angleToTargetField - heading);
        desiredTurretAngleDeg = wrapTo180(desiredTurretAngleDeg);

        // Current turret angle
        double currentTurretAngleDeg = getTurretAngleDeg();

        double errDeg = wrapTo180(desiredTurretAngleDeg - currentTurretAngleDeg);


        if (Math.abs(errDeg) < Constants.deadbandDeg) {
            errDeg = 0.0;
        }

        errFiltDeg = Constants.errAlpha * errDeg + (1.0 - Constants.errAlpha) * errFiltDeg;


        integral += errFiltDeg * dt;
        if (errDeg == 0.0) integral *= 0.5;
        integral = clamp(integral, -Constants.maxIntegral, Constants.maxIntegral);

        double deriv = (errFiltDeg - lastErrDeg) / dt;
        deriv = clamp(deriv, -Constants.maxDeriv, Constants.maxDeriv);
        lastErrDeg = errFiltDeg;


        double rawPower =
                Constants.kP_v * errFiltDeg +
                        Constants.kI_v * integral +
                        Constants.kD_v * deriv;

        if (errDeg != 0.0 && Constants.kS > 0) {
            rawPower += Math.signum(errFiltDeg) * Constants.kS;
        }

        double maxPower = clamp(Constants.maxPower, 0.0, 1.0);
        double power = clamp(rawPower, -maxPower, maxPower);

        if (Math.abs(power) >= maxPower - 1e-6 && Math.signum(power) == Math.signum(rawPower)) {
            integral *= 0.95;
        }

        setTurretPower(power);
    }

    private void resetPID() {
        integral = 0.0;
        lastErrDeg = 0.0;
        errFiltDeg = 0.0;
        lastNanos = System.nanoTime();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double wrapTo180(double angleDeg) {
        angleDeg %= 360.0;
        if (angleDeg > 180.0) angleDeg -= 360.0;
        if (angleDeg < -180.0) angleDeg += 360.0;
        return angleDeg;
    }
}