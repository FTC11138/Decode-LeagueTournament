package org.firstinspires.ftc.teamcode.commands.advanced;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.LoadBallCommand;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class AutoLoadBallCommand extends SequentialCommandGroup {
    public AutoLoadBallCommand() {
        super(
                new WaitCommand(Constants.ballDetectWait),
                new LoadBallCommand()
        );
    }
}
