package org.firstinspires.ftc.teamcode.Andie.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Andie.Subsystems.Intake;
public class IntakeOutCommand extends SequentialCommandGroup {
    public IntakeOutCommand(Intake intake) {
        addCommands(
                new InstantCommand(intake::out),
                new WaitCommand(1),
                new InstantCommand(intake::stop)
        );
    }
}
