package org.firstinspires.ftc.teamcode.hardware;

import android.annotation.SuppressLint;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.TurretOdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;

public class RobotData {

    public long loopTime = System.currentTimeMillis();
    public Pose currentPose = new Pose(0,0, Math.toRadians(0));

    public CameraSubsystem.Obelisk obelisk = CameraSubsystem.Obelisk.PPP;

    public int spindexerCurrentPosition = 0;
    public int spindexerTargetPosition = 0;
    public boolean spindexerMoving = false;
    public boolean intakeRangerCanTurn = false;
    public int ballCount = 0;
    public double TICKS_PER_REVOLUTION = 0;
    public double TICKS_120_DEG = 0;
    public double MOVE_POWER = 0;

    public IntakeSubsystem.IntakeState intakeState = IntakeSubsystem.IntakeState.STOP;

    public ShooterSubsystem.ShooterState shooterState = ShooterSubsystem.ShooterState.STOP;

    public double shooterTargetVelocity = 0;
    public double shooterCurrentVelocity1 = 0;
    public double shooterCurrentVelocity2 = 0;
    public double shooterCurrentRPM1 = 0;
    public double shooterCurrentRPM2 = 0;

    public TurretOdometrySubsystem.TurretState turretState = TurretOdometrySubsystem.TurretState.MANUAL;

    // Target point in FIELD coords
    public double turretTargetX = 0;
    public double turretTargetY = 0;

    // Turret pivot position in FIELD coords (after applying offset)
    public double turretPivotX = 0;
    public double turretPivotY = 0;

    // Turret angles (degrees)
    public double turretAngleDeg = 0;          // current from encoder (continuous)
    public double turretDesiredDeg = 0;        // desired (continuous, clamped)
    public double turretErrorDeg = 0;          // desired - current
    public double turretServoPower = 0;        // actual power sent to CRServo

    // Raw aim angles for sanity checking
    public double robotHeadingDeg = 0;
    public double angleToTargetFieldDeg = 0;   // atan2 result in field frame
    public double turretRawRelDeg = 0;         // (fieldAngle - heading) wrapped to [-180,180]

    @SuppressLint("DefaultLocale")
    public void write(Telemetry telemetry) {

        telemetry.addData("LOOP TIME", System.currentTimeMillis() - loopTime);
        loopTime = System.currentTimeMillis();

        telemetry.addLine("");
        telemetry.addLine("");

        telemetry.addData("POSE", this.currentPose);
        telemetry.addData("BUSY", Robot.getInstance().follower.isBusy());
        telemetry.addLine(Constants.robotCentric ? "ROBOT CENTRIC" : "FIELD CENTRIC");

        telemetry.addLine("");

        telemetry.addData("ALLIANCE", Globals.ALLIANCE);

        telemetry.addLine("");
        telemetry.addLine("");

        telemetry.addData("Spindexer Current Pos", spindexerCurrentPosition);
        telemetry.addData("Spindexer Target Pos", spindexerTargetPosition);
        telemetry.addData("Spindexer Moving", spindexerMoving);
        telemetry.addLine("");
        telemetry.addData("Intake Ranger Can Turn", intakeRangerCanTurn);
        telemetry.addData("Ball Count", ballCount);
        telemetry.addLine("");
        telemetry.addData("Ticks / Revolution", TICKS_PER_REVOLUTION);
        telemetry.addData("Ticks / 120 Deg", TICKS_120_DEG);
        telemetry.addData("Move Power", MOVE_POWER);

        telemetry.addLine("");
        telemetry.addLine("");

        telemetry.addData("Intake State", intakeState);

        telemetry.addData("Shooter1 Power", shooterCurrentRPM1);

        telemetry.addData("Shooter2 Power", shooterCurrentRPM2);

        telemetry.addLine("");
        telemetry.addLine("");

        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Shooter Target Vel", shooterTargetVelocity);
        telemetry.addData("Shooter Vel 1", shooterCurrentVelocity1);
        telemetry.addData("Shooter Vel 2", shooterCurrentVelocity2);
        telemetry.addData("Shooter RPM 1", shooterCurrentRPM1);
        telemetry.addData("Shooter RPM 2", shooterCurrentRPM2);

        telemetry.addLine("");
        telemetry.addLine("");

        telemetry.addData("Turret State", turretState);
        telemetry.addLine("");
        telemetry.addData("Turret Target (X,Y)", String.format("(%.2f, %.2f)", turretTargetX, turretTargetY));
        telemetry.addData("Turret Pivot (X,Y)", String.format("(%.2f, %.2f)", turretPivotX, turretPivotY));
        telemetry.addLine("");
        telemetry.addData("Robot Heading (deg)", String.format("%.2f", robotHeadingDeg));
        telemetry.addData("Field Angle->Target (deg)", String.format("%.2f", angleToTargetFieldDeg));
        telemetry.addData("Turret Raw Rel (deg)", String.format("%.2f", turretRawRelDeg));
        telemetry.addLine("");
        telemetry.addData("Turret Current (deg)", String.format("%.2f", turretAngleDeg));
        telemetry.addData("Turret Desired (deg)", String.format("%.2f", turretDesiredDeg));
        telemetry.addData("Turret Error (deg)", String.format("%.2f", turretErrorDeg));
        telemetry.addData("Turret Servo Pwr", String.format("%.3f", turretServoPower));

        telemetry.update();
    }
}