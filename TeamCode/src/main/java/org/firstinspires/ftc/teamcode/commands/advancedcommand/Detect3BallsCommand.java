package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Detect3BallsCommand extends CommandBase {
    @Override
    public boolean isFinished() {
        return Robot.getInstance().spindexerTestSubsystem.getBallCount() >= 3;
    }
}
