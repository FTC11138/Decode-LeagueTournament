package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ShooterStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.TurretStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@TeleOp(name = "Adjustable Hood Tuner")
public class AdjHoodTuner extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private GamepadEx g1;

    private boolean teleOpEnabled = false;

    private double hoodPos = Constants.adjHoodMin;
    private final double HOOD_STEP = 0.01;

    private double manualShooterTPS = -1200;
    private final double TPS_STEP = 25;

    private boolean automatic = false;

    private boolean lastLeftTrigger = false;
    private boolean lastRightTrigger = false;


    /*
    HOW TO USE:

    Set up the robot starting at the center of the field facing red.
    start this program and enable it with the start button. pressing the start button again should diable it and vice-versa.
    Right bumper makes hood higher, Left bumper makes hood lower
    DPad right makes speed higher, DPad left makes speed lower

    Intake and spindexer controls are regular tele op controls

    You can press A to toggle automatic aiming for testing;

     */


    @Override
    public void initialize() {
        g1 = new GamepadEx(gamepad1);

        Globals.IS_AUTO = false;
        robot.initialize(hardwareMap, telemetry);
        robot.follower.setPose(PoseStorage.pose);

        bindButtons();
    }

    private void bindButtons() {

        g1.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                new InstantCommand(() -> {
                    teleOpEnabled = !teleOpEnabled;
                })
        );

        g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> {
                    hoodPos -= HOOD_STEP;
                })
        );

        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> {
                    hoodPos += HOOD_STEP;
                })
        );

        g1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> {
                    manualShooterTPS -= TPS_STEP;
                })
        );

        g1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> {
                    manualShooterTPS += TPS_STEP;
                })
        );

        g1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> {
                    automatic = !automatic;
                })
        );

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        if (!teleOpEnabled) {
            robot.shooterSubsystem.updateAdjHoodState(ShooterSubsystem.AdjHoodState.NONE);
            robot.shooterSubsystem.updateShooterState(ShooterSubsystem.ShooterState.STOP);
            robot.intakeSubsystem.updateIntakeState(IntakeSubsystem.IntakeState.STOP);
        } else {

            robot.periodic();
            robot.updateData();
            robot.write();

            robot.follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * 1.4,
                    -gamepad1.left_stick_x * 1.4,
                    -gamepad1.right_stick_x * 1.0,
                    Constants.robotCentric
            );

            if (automatic) {
                robot.shooterSubsystem.updateAdjHoodState(ShooterSubsystem.AdjHoodState.AUTO);
                robot.shooterSubsystem.updateShooterState(ShooterSubsystem.ShooterState.AUTO);
            } else {
                robot.shooterSubsystem.updateAdjHoodState(ShooterSubsystem.AdjHoodState.MANUAL);
                robot.shooterSubsystem.updateShooterState(ShooterSubsystem.ShooterState.MANUAL);
            }

            robot.shooterSubsystem.setHoodPos(hoodPos);
            robot.shooterSubsystem.setShooterSpeed(manualShooterTPS);


            boolean leftTrigger = gamepad1.left_trigger > 0.5;
            boolean rightTrigger = gamepad1.right_trigger > 0.5;

            // RT: rotateShootCW, wait, intake IN
            scheduleCommand(
                    lastRightTrigger,
                    rightTrigger,
                    new SequentialCommandGroup(
                            new InstantCommand(() -> robot.spindexerTestSubsystem.rotateShootCW()),
                            new WaitCommand(Constants.shootWait),
                            new IntakeStateCommand(IntakeSubsystem.IntakeState.IN)
                    )
            );

            scheduleCommand(
                    lastLeftTrigger,
                    leftTrigger,
                    new SequentialCommandGroup(
                            new IntakeStateCommand(IntakeSubsystem.IntakeState.IN),
                            new TurretStateCommand(TurretSubsystem.TurretState.TRACK)
                    )
            );

            lastLeftTrigger = leftTrigger;
            lastRightTrigger = rightTrigger;

        }

    }

    private void scheduleCommand(boolean lastPress, boolean currPress, Command command) {
        if (currPress && !lastPress) {
            CommandScheduler.getInstance().schedule(command);
        }
    }
}