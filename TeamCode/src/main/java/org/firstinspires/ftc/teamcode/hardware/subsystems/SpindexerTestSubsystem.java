package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.advanced.ArtifactEjectCommand;
import org.firstinspires.ftc.teamcode.commands.advanced.AutoLoadBallCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.LoadBallCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

public class SpindexerTestSubsystem extends RE_SubsystemBase {

    private final DcMotorEx spindexerMotor;
    private int targetPosition = 0;

    private final AnalogInput ranger;
    private double distance = 0;
    private boolean lastBallDetected;
    private boolean ballDetected;
    private int ballCount = 0;

    // ===== Constants =====
    private static final double TICKS_PER_REVOLUTION = 537.7; // adjust if needed
    private static final double TICKS_120_DEG = TICKS_PER_REVOLUTION / 3.0;
    private static final double MOVE_POWER = 0.4;

    public SpindexerTestSubsystem(HardwareMap hw, String motorName, String rangerName) {
        spindexerMotor = hw.get(DcMotorEx.class, motorName);

        ranger = hw.get(AnalogInput.class, rangerName);

        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetPosition = 0;
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Robot.getInstance().subsystems.add(this);
    }

    public void rotate360CW() {
        moveRelative(-TICKS_PER_REVOLUTION);
    }

    public void rotate360CCW() {
        moveRelative(TICKS_PER_REVOLUTION);
    }

    public void rotate120CW() {
        moveRelative(-TICKS_120_DEG);
    }

    public void rotate120CCW() {
        moveRelative(TICKS_120_DEG);
    }

    public void stop() {
        spindexerMotor.setPower(0);
    }

    public boolean isMoving() {
        return spindexerMotor.isBusy();
    }

    public int getCurrentPosition() {
        return spindexerMotor.getCurrentPosition();
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public double getDistance() {
        return distance;
    }

    public int getBallCount() {
        return ballCount;
    }

    public void resetBallCount() {
        ballCount = 0;
    }

    private void moveRelative(double deltaTicks) {
        targetPosition += (int) deltaTicks;
        spindexerMotor.setTargetPosition(targetPosition);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexerMotor.setPower(MOVE_POWER);
    }

    @Override
    public void updateData() {
        Robot robot = Robot.getInstance();

        robot.data.spindexerCurrentPosition = getCurrentPosition();
        robot.data.spindexerTargetPosition = targetPosition;
        robot.data.spindexerMoving = isMoving();

        robot.data.intakeRangerDistance = distance;
        robot.data.ballCount = ballCount;

        robot.data.TICKS_PER_REVOLUTION = TICKS_PER_REVOLUTION;
        robot.data.TICKS_120_DEG = TICKS_120_DEG;
        robot.data.MOVE_POWER = MOVE_POWER;
    }

    @Override
    public void periodic() {
        distance = ranger.getVoltage();

        ballDetected = distance < Constants.ballDetectThreshold;

        if (ballDetected && !lastBallDetected && Globals.AUTO_SPINDEX) {
            if (ballCount >= 3) {
                CommandScheduler.getInstance().schedule(new ArtifactEjectCommand());
            } else {
                ballCount++;
                if (ballCount < 3) {
                    CommandScheduler.getInstance().schedule(new AutoLoadBallCommand());
                }
            }
        }

        lastBallDetected = ballDetected;
    }
}