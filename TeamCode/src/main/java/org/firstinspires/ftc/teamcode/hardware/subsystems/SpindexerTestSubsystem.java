package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.advanced.ArtifactEjectCommand;
import org.firstinspires.ftc.teamcode.commands.advanced.AutoLoadBallCommand;
import org.firstinspires.ftc.teamcode.commands.advanced.IntakeUnstuckCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.LoadBallCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

import java.util.concurrent.TimeUnit;

@Configurable
public class SpindexerTestSubsystem extends RE_SubsystemBase {

    private final PIDController pid;

    private final DcMotorEx spindexerMotor;
//    private final Servo led;
    private int targetPosition = 0;

    private double distance = 0;

    private double lastCurrent;
    private double current;
    private boolean ignoreUnstuck;
    private long lastUnstuckTime;

    private final DigitalChannel ranger;
    private boolean canRotate = false;
    private boolean lastBallDetected;
    private boolean ballDetected;
    private int ballCount = 0;
    public boolean ignoreSensor = false;
    public int sensorWait = Constants.sensorWait;

    // ===== Constants =====
    private static final double TICKS_PER_REVOLUTION = 537.7; // adjust if needed
    private static final double TICKS_120_DEG = TICKS_PER_REVOLUTION / 3.0;
    private static final double TICKS_PER_SHOOT = 537.7 + (TICKS_PER_REVOLUTION / 3.0);
    private static final double DEADBAND_DEGREES = 2.0;
    private static final double DEADBAND_TICKS = (DEADBAND_DEGREES / 360.0) * TICKS_PER_REVOLUTION;
    private static final long STUCK_TIMEOUT_MS = 2000; // 2 seconds
    private static final double STUCK_THRESHOLD_TICKS = 10; // minimum movement to not be considered stuck

    public static double MOVE_POWER = 1;

    public static double SHOOT_POWER = 0.7;

    ElapsedTime timer;
    long lastDetectTime;

    // Stuck detection variables
    private int lastPosition = 0;
    private long lastMovementTime = 0;
    private boolean wasStuck = false;

    public SpindexerTestSubsystem(HardwareMap hw, String motorName, String rangerName) {
        spindexerMotor = hw.get(DcMotorEx.class, motorName);
//        led = hw.get(Servo.class, "led");

        pid = new PIDController(Constants.spindexer_kP, Constants.spindexer_kI, Constants.spindexer_kD);

        ranger = hw.get(DigitalChannel.class, rangerName);

        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        targetPosition = 0;


        timer = new ElapsedTime();
        lastMovementTime = timer.time(TimeUnit.MILLISECONDS);

        Robot.getInstance().subsystems.add(this);
    }

    public void rotate360CW() {
        moveRelative(-TICKS_PER_REVOLUTION, SHOOT_POWER);
        this.ballCount = 0;
    }

    public void rotateShootCW() {
        Globals.INTAKING = false;
        Globals.SHOOTING = true;
        moveRelative(-TICKS_PER_SHOOT, SHOOT_POWER);
        ignoreSensor = true;
        sensorWait = Constants.shootSensorWait;
        lastDetectTime = timer.time(TimeUnit.MILLISECONDS);
        this.ballCount = 0;
    }

    public void rotateResetCW() {
        moveRelative(TICKS_120_DEG - 20, SHOOT_POWER);
        this.ballCount = 0;
    }

    public void resetCounter() {
        this.ballCount--;
    }


    public void rotate360CCW() {
        moveRelative(TICKS_PER_REVOLUTION, MOVE_POWER);
    }

    public void rotate15CW() {
        moveRelative(TICKS_PER_REVOLUTION / 15, MOVE_POWER);
        this.ballCount--;
    }

    public void rotate120CCW() {
        moveRelative(TICKS_120_DEG, MOVE_POWER);
    }

    public void rotate120CW() {
        moveRelative(-TICKS_120_DEG, MOVE_POWER);
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

    public double getCurrent() {
        if (spindexerMotor == null) return 0;
        return spindexerMotor.getCurrent(CurrentUnit.AMPS);
    }

    private void moveRelative(double deltaTicks, double power) {
        spindexerMotor.setPower(power);
        targetPosition += (int) deltaTicks;
        wasStuck = false; // Reset stuck flag when new movement is commanded
    }

    /**
     * Finds the nearest valid spindexer position (multiple of 120 degrees)
     */
    private int getNearestValidPosition(int currentPos) {
        // Find the nearest multiple of TICKS_120_DEG
        int remainder = (int) (currentPos % TICKS_120_DEG);
        int nearestLower = currentPos - remainder;
        int nearestUpper = nearestLower + (int) TICKS_120_DEG;

        // Return whichever is closer
        if (Math.abs(currentPos - nearestLower) < Math.abs(currentPos - nearestUpper)) {
            return nearestLower;
        } else {
            return nearestUpper;
        }
    }

    @Override
    public void updateData() {
        Robot robot = Robot.getInstance();

        robot.data.spindexerCurrent = getCurrent();
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


        pid.setPID(Constants.spindexer_kP, Constants.spindexer_kI, Constants.spindexer_kD);

        int currentPos = getCurrentPosition();
        double error = getTargetPosition() - currentPos;

//        // Check if spindexer has moved
//        if (Math.abs(currentPos - lastPosition) > STUCK_THRESHOLD_TICKS) {
//            lastPosition = currentPos;
//            lastMovementTime = timer.time(TimeUnit.MILLISECONDS);
//        }
//
//        // Check if stuck (not at target and hasn't moved in 2 seconds)
//        long currentTime = timer.time(TimeUnit.MILLISECONDS);
//        if (Math.abs(error) > DEADBAND_TICKS &&
//                (currentTime - lastMovementTime) > STUCK_TIMEOUT_MS &&
//                !wasStuck) {
//
//            // Spindexer is stuck! Revert to nearest valid position
//            int nearestValid = getNearestValidPosition(currentPos);
//            targetPosition = nearestValid;
//            wasStuck = true;
//            lastMovementTime = currentTime; // Reset timer
//        }

        spindexerMotor.setTargetPosition(getTargetPosition());
        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexerMotor.setPower(SHOOT_POWER);


        canRotate = ranger.getState();

        ballDetected = canRotate;

        if (ballDetected && Globals.AUTO_SPINDEX && (!ignoreSensor)) {
            if (Globals.INTAKING) {
                ignoreSensor = true;
                sensorWait = Constants.sensorWait;
                lastDetectTime = timer.time(TimeUnit.MILLISECONDS);
                ballCount++;
                //led.setPosition(0.277);

                if (ballCount < 3) {
                    CommandScheduler.getInstance().schedule(new AutoLoadBallCommand());
                } else {
                    CommandScheduler.getInstance().schedule(new IntakeStateCommand(IntakeSubsystem.IntakeState.OUT));
                }

//                if(ballCount == 1){
//                    led.setPosition(0.722);
//                }
//
//                if(ballCount == 2){
//                    led.setPosition(0.334);
//                }
//
//                if(ballCount == 3){
//                    led.setPosition(0.51);
//                }
            }
        }


        current = getCurrent();
        if (current >= Constants.stuckCurrent && !ignoreUnstuck) {
            CommandScheduler.getInstance().schedule(new IntakeUnstuckCommand());
            ignoreUnstuck = true;
            lastUnstuckTime = timer.time(TimeUnit.MILLISECONDS);
        }

        long detectTime = timer.time(TimeUnit.MILLISECONDS);
        if (detectTime - lastUnstuckTime >= Constants.unstuckWait) {
            ignoreUnstuck = false;
        }

        if(ballCount == 3){
//            led.setPosition(0.51);
        }
        else if (detectTime - lastDetectTime >= sensorWait) {
            ignoreSensor = false;
//            led.setPosition(0);
        }

        lastBallDetected = ballDetected;


    }
}
