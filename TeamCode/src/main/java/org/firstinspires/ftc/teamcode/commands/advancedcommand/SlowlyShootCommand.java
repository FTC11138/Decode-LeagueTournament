package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class SlowlyShootCommand extends SequentialCommandGroup {
    public SlowlyShootCommand() {
        super(
                new SequentialCommandGroup(
                        new InstantCommand (() -> Robot.getInstance().spindexerTestSubsystem.rotate120CW()),
                        new WaitCommand(Constants.shootBetweenWait),
                        new InstantCommand (() -> Robot.getInstance().spindexerTestSubsystem.rotate120CW()),
                        new WaitCommand(Constants.shootBetweenWait),
                        new InstantCommand (() -> Robot.getInstance().spindexerTestSubsystem.rotate120CW()),
                        new WaitCommand(Constants.shootBetweenWait),
                        new InstantCommand (() -> Robot.getInstance().spindexerTestSubsystem.rotate120CW()),
                        new InstantCommand (() -> Robot.getInstance().spindexerTestSubsystem.resetBallCount())
                        )

        );
    }
}
