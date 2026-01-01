package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.advanced.ArtifactEjectCommand;
import org.firstinspires.ftc.teamcode.commands.advanced.AutoLoadBallCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.LoadBallCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

import java.util.concurrent.TimeUnit;

@Configurable
public class SpindexerTestSubsystem extends RE_SubsystemBase {

    private final DcMotorEx spindexerMotor;
    private int targetPosition = 0;

    private double distance = 0;

    private final DigitalChannel ranger;
    private boolean canRotate = false;
    private boolean lastBallDetected;
    private boolean ballDetected;
    private int ballCount = 0;
    public boolean ignoreSensor = false;

    // ===== Constants =====
    private static final double TICKS_PER_REVOLUTION = 537.7; // adjust if needed
    private static final double TICKS_120_DEG = TICKS_PER_REVOLUTION / 3.0;
    public static double MOVE_POWER = 1;

    public static double SHOOT_POWER = 0.8;

    ElapsedTime timer;
    long lastDetectTime;

    public SpindexerTestSubsystem(HardwareMap hw, String motorName, String rangerName) {
        spindexerMotor = hw.get(DcMotorEx.class, motorName);

        ranger = hw.get(DigitalChannel.class, rangerName);

        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetPosition = 0;
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer = new ElapsedTime();

        Robot.getInstance().subsystems.add(this);
    }

    public void rotate360CW() {
        moveRelative(-TICKS_PER_REVOLUTION, SHOOT_POWER);
        this.ballCount = 0;
    }

    public void rotate360CCW() {
        moveRelative(TICKS_PER_REVOLUTION, MOVE_POWER);
    }

    public void rotate120CW() {
        moveRelative(-TICKS_120_DEG, SHOOT_POWER);
        this.ballCount --;
    }

    public void rotate120CCW() {
        moveRelative(TICKS_120_DEG, MOVE_POWER);
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

//    public double getDistance() {
//        return distance;
//    }

    public int getBallCount() {
        return ballCount;
    }

    public void resetBallCount() {
        ballCount = 0;
    }

    private void moveRelative(double deltaTicks, double power) {
        targetPosition += (int) deltaTicks;
        spindexerMotor.setTargetPosition(targetPosition);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexerMotor.setPower(power);
    }

    @Override
    public void updateData() {
        Robot robot = Robot.getInstance();

        robot.data.spindexerCurrentPosition = getCurrentPosition();
        robot.data.spindexerTargetPosition = targetPosition;
        robot.data.spindexerMoving = isMoving();

        robot.data.intakeRangerCanTurn = canRotate;
        robot.data.ballCount = ballCount;

        robot.data.TICKS_PER_REVOLUTION = TICKS_PER_REVOLUTION;
        robot.data.TICKS_120_DEG = TICKS_120_DEG;
        robot.data.MOVE_POWER = MOVE_POWER;
    }

    @Override
    public void periodic() {
        canRotate = ranger.getState();

        ballDetected = canRotate;

        if (ballDetected && !lastBallDetected && Globals.AUTO_SPINDEX && (!ignoreSensor)) {
            if (ballCount >= 3) {
                CommandScheduler.getInstance().schedule(new WaitCommand(200 ));
                CommandScheduler.getInstance().schedule(new ArtifactEjectCommand());
            } else {
                ballCount++;
                if (ballCount < 3) {
                    CommandScheduler.getInstance().schedule(new AutoLoadBallCommand());
                    ignoreSensor = true;
                    lastDetectTime = timer.time(TimeUnit.MILLISECONDS);
                }
            }
        }

        long detectTime = timer.time(TimeUnit.MILLISECONDS);
        if (detectTime - lastDetectTime >= Constants.sensorWait) ignoreSensor = false;

        lastBallDetected = ballDetected;
    }
}