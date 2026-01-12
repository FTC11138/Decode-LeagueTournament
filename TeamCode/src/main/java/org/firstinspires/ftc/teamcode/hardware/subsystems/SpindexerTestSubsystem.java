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

    private final PIDController pid;

    private final DcMotorEx spindexerMotor;
    private final Servo led;
    private int targetPosition = 0;

    private double distance = 0;

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
    public static double MOVE_POWER = 1;

    public static double SHOOT_POWER = 0.7;

    ElapsedTime timer;
    long lastDetectTime;

    public SpindexerTestSubsystem(HardwareMap hw, String motorName, String rangerName) {
        spindexerMotor = hw.get(DcMotorEx.class, motorName);
        led = hw.get(Servo.class, "led");

        pid = new PIDController(Constants.spindexer_kP, Constants.spindexer_kI, Constants.spindexer_kD);

        ranger = hw.get(DigitalChannel.class, rangerName);

        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetPosition = 0;


        timer = new ElapsedTime();

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
        moveRelative(TICKS_120_DEG -20, SHOOT_POWER);
        this.ballCount = 0;
    }

    public void resetCounter(){
        this.ballCount --;
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


        pid.setPID(Constants.spindexer_kP, Constants.spindexer_kI, Constants.spindexer_kD);
        double power = pid.calculate(getCurrentPosition(), getTargetPosition());
        spindexerMotor.setPower(power);



        canRotate = ranger.getState();

        ballDetected = canRotate;

        if (ballDetected && !lastBallDetected && Globals.AUTO_SPINDEX && (!ignoreSensor)) {
            if (Globals.INTAKING) {
                ignoreSensor = true;
                sensorWait = Constants.sensorWait;
                lastDetectTime = timer.time(TimeUnit.MILLISECONDS);
                ballCount++;
                led.setPosition(1);

                if (ballCount < 3){
                    CommandScheduler.getInstance().schedule(new AutoLoadBallCommand());
                } else {
                    CommandScheduler.getInstance().schedule(new IntakeStateCommand(IntakeSubsystem.IntakeState.STOP));
                }
            }
        }

        long detectTime = timer.time(TimeUnit.MILLISECONDS);
        if (detectTime - lastDetectTime >= sensorWait) {
            ignoreSensor = false;
            led.setPosition(0);
        }

        lastBallDetected = ballDetected;
    }
}