package org.firstinspires.ftc.teamcode.Andie.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Andie.Subsystems.Intake;

public class IntakeInCommand extends SequentialCommandGroup {
    public IntakeInCommand(Intake intake) {
        addCommands(
                new InstantCommand(intake::intakeIn),
                new WaitCommand(500),
                new InstantCommand(intake::intakeStop)
        );
    }
}
