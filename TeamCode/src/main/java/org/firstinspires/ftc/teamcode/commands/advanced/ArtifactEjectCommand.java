package org.firstinspires.ftc.teamcode.commands.advanced;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;

public class ArtifactEjectCommand extends SequentialCommandGroup {
    public ArtifactEjectCommand() {
        super(
                new IntakeStateCommand(IntakeSubsystem.IntakeState.OUT),
                new WaitCommand(500),
                new IntakeStateCommand(IntakeSubsystem.IntakeState.IN)
        );
    }
}
