package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

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

public class Tele_Op_Solo extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private GamepadEx g1;

    public static Pose startingPose;

    private boolean teleOpEnabled = false;

    // Button state tracking
    private boolean lastLeftTrigger;
    private boolean lastRightTrigger;
    private boolean lastX;
    private boolean lastStart;

    @Override
    public void initialize() {
        g1 = new GamepadEx(gamepad1);

        Globals.IS_AUTO = false;

        robot.initialize(hardwareMap, telemetry);
        CommandScheduler.getInstance().reset();
        robot.follower.setStartingPose(PoseStorage.pose);

        if (Globals.ALLIANCE == Globals.COLORS.RED){
            gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }
        else {
            gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }

        bindButtons();
    }

    private void bindButtons() {
//        // Spindexer controls
//        g1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                new InstantCommand(() -> robot.spindexerTestSubsystem.rotate360CCW())
//        );

//        g1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
//                new InstantCommand(() -> robot.spindexerTestSubsystem.rotate360CCW())
//        );

        g1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.spindexerTestSubsystem.rotate15CW())
                )
        );

//        g1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
//
//        );

//        g1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
//                new SequentialCommandGroup(
//                        new InstantCommand(()-> robot.spindexerTestSubsystem.resetCounter())
//                )
//
//        );
        g1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new ShooterStateCommand(ShooterSubsystem.ShooterState.STOP)
        );


        // Intake stop
        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> robot.intakeSubsystem.updateIntakeState(IntakeSubsystem.IntakeState.OUT))
        );
    }

    @Override
    public void run() {

        if (teleOpEnabled) {
            CommandScheduler.getInstance().run();

            robot.periodic();
            robot.updateData();
            robot.write();

            robot.follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * 1.4,
                    -gamepad1.left_stick_x * 1.4,
                    -gamepad1.right_stick_x * 1.0,
                    Constants.robotCentric // Robot Centric
            );


        }

        // Read special buttons
        boolean x = g1.getButton(GamepadKeys.Button.X);
        boolean start = g1.getButton(GamepadKeys.Button.START);

        // Robot-centric toggle on X rising edge
        if (!lastX && x) {
            Constants.robotCentric = !Constants.robotCentric;
            gamepad1.rumble(500);

            if (Constants.robotCentric) {
                gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            } else {
                gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
            }
        }

        // Enable teleOp on START rising edge
        if (!lastStart && start) {
            teleOpEnabled = true;
            gamepad1.rumble(2000);
        }



        // Handle triggers as digital buttons
        boolean leftTrigger = gamepad1.left_trigger > 0.5;
        boolean rightTrigger = gamepad1.right_trigger > 0.5;

        // Right trigger: rotate 360 CW
        scheduleCommand(
                lastRightTrigger,
                rightTrigger,
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.spindexerTestSubsystem.rotateShootCW()),
                        new WaitCommand(Constants.shootWait),
                        new IntakeStateCommand(IntakeSubsystem.IntakeState.IN)
                )
        );

        // Left trigger: intake IN
        scheduleCommand(
                lastLeftTrigger,
                leftTrigger,
                new SequentialCommandGroup(
                        new ShooterStateCommand(ShooterSubsystem.ShooterState.AUTO),
                        new IntakeStateCommand(IntakeSubsystem.IntakeState.IN),
                        new TurretStateCommand(TurretSubsystem.TurretState.TRACK)
                )
        );

        // Update last button states
        lastX = x;
        lastStart = start;
        lastLeftTrigger = leftTrigger;
        lastRightTrigger = rightTrigger;

        // Touchpad: reset pose
        if (gamepad1.touchpad) {
            robot.follower.setPose(new Pose(
                    robot.follower.getPose().getX(),
                    robot.follower.getPose().getY(),
                    Globals.ALLIANCE == Globals.COLORS.BLUE ? Math.toRadians(0) : Math.toRadians(180)
            ));
            gamepad1.rumble(500);
            gamepad1.setLedColor(0, 1, 0, 1000);
        }
    }


    private void scheduleCommand(boolean lastPress, boolean currPress, Command command) {
        if (currPress && !lastPress) {
            CommandScheduler.getInstance().schedule(command);
        }
    }
}