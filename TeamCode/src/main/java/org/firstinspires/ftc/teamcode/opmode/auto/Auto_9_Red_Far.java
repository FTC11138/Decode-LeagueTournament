package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods.buildPath;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods.buildCurve;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.advancedcommand.SlowlyShootCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.PathCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.AdjustableHoodStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ShooterStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.TurretStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.TurretOdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous(name = "Auto_9_Red_Far")
@Configurable
public class Auto_9_Red_Far extends LinearOpMode {

    /* ---------------- START POSE ---------------- */
    public static double startX = 79;
    public static double startY = 8.7;
    public static double startHeading = 90;

    /* ---------------- PATH POINTS ---------------- */

    // Path 1
    public static double p1X = 84.5;
    public static double p1Y = 16.5;
    public static double p1Heading = 65;

    // Path 2
    public static double p2X = 102;
    public static double p2Y = 35;
    public static double p2Heading = 0;

    // Path 3
    public static double p3X = 134;
    public static double p3Y = 35;
    public static double p3Heading = 0;

    // Path 4
    public static double p4X = 84.5;
    public static double p4Y = 16.5;
    public static double p4Heading = 65;

    // Path 5
    public static double p5X = 134;
    public static double p5Y = 17;
    public static double p5Heading = 0;

    // Path 6 (curve)
    public static double p6X = 134;
    public static double p6Y = 8;
    public static double p6Heading = 0;

    public static double p6ControlX = 106;
    public static double p6ControlY = 10;

    // Path 7
    public static double p7X = 84.5;
    public static double p7Y = 16.5;
    public static double p7Heading = 65;

    // Path 8
    public static double p8X = 84;
    public static double p8Y = 38;
    public static double p8Heading = 0;

    /* ---------------- PATH OBJECTS ---------------- */

    public static Path path1;
    public static Path path2;
    public static Path path3;
    public static Path path4;
    public static Path path5;
    public static Path path6;
    public static Path path7;
    public static Path path8;

    public Pose startPose = new Pose(startX, startY, Math.toRadians(startHeading));

    public void buildPaths() {

        Pose pose1 = new Pose(p1X, p1Y, Math.toRadians(p1Heading));
        Pose pose2 = new Pose(p2X, p2Y, Math.toRadians(p2Heading));
        Pose pose3 = new Pose(p3X, p3Y, Math.toRadians(p3Heading));
        Pose pose4 = new Pose(p4X, p4Y, Math.toRadians(p4Heading));
        Pose pose5 = new Pose(p5X, p5Y, Math.toRadians(p5Heading));
        Pose pose6 = new Pose(p6X, p6Y, Math.toRadians(p6Heading));
        Pose pose7 = new Pose(p7X, p7Y, Math.toRadians(p7Heading));
        Pose pose8 = new Pose(p8X, p8Y, Math.toRadians(p8Heading));

        Pose control6 = new Pose(p6ControlX, p6ControlY);

        path1 = buildPath(startPose, pose1);
        path2 = buildPath(pose1, pose2);
        path3 = buildPath(pose2, pose3);
        path4 = buildPath(pose3, pose4);
        path5 = buildPath(pose4, pose5);
        path6 = buildCurve(pose5, pose6, control6);
        path7 = buildPath(pose6, pose7);
        path8 = buildPath(pose7, pose8);
    }

    @Override
    public void runOpMode() {

        Robot robot = Robot.getInstance();
        Globals.IS_AUTO = true;

        Globals.ALLIANCE = Globals.COLORS.RED;

        robot.initialize(hardwareMap, telemetry);
        CommandScheduler.getInstance().reset();

        buildPaths();

        robot.follower.setStartingPose(startPose);

        while (!isStarted() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            robot.updateData();
            robot.periodic();
            robot.write();
        }

        robot.follower.setPose(startPose);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        new PathCommand(path1).alongWith(
                                new SequentialCommandGroup(
                                        new ShooterStateCommand(ShooterSubsystem.ShooterState.FAR),
                                        new AdjustableHoodStateCommand(ShooterSubsystem.AdjHoodState.FAR),
                                        new TurretStateCommand(TurretOdometrySubsystem.TurretState.TRACK_POINT)
                                )
                        ),

                        new WaitCommand(2500),

                        new SlowlyShootCommand(),

                        new WaitCommand(2000),
                        new IntakeStateCommand(IntakeSubsystem.IntakeState.IN),
                        //new WaitCommand(500),

                        new PathCommand(path2, 0.8),
                        new PathCommand(path3),

                        new WaitCommand(200),

                        new PathCommand(path4).andThen(
                                new WaitCommand(1000),
                                new SlowlyShootCommand()
                        ),

                        new WaitCommand(300),

                        new IntakeStateCommand(IntakeSubsystem.IntakeState.IN),

                        new PathCommand(path5),
                        new PathCommand(path6, 0.45),

                        new WaitCommand(700),

                        new PathCommand(path7).andThen(
                                new WaitCommand(1000),
                                new SlowlyShootCommand()
                        ),

                        new WaitCommand(300),

                        new PathCommand(path8)
                )
        );

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            robot.updateData();
            robot.periodic();
            robot.write();
            PoseStorage.pose = robot.follower.getPose();
        }
    }
}