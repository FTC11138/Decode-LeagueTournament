package org.firstinspires.ftc.teamcode.commands.advanced;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class IntakeUnstuckCommand extends SequentialCommandGroup {
    public IntakeUnstuckCommand() {
        super(
                new IntakeStateCommand(IntakeSubsystem.IntakeState.OUT),
                new WaitCommand(Constants.intakeUnstuckDelay),
                new IntakeStateCommand(IntakeSubsystem.IntakeState.IN)
        );
    }
}
