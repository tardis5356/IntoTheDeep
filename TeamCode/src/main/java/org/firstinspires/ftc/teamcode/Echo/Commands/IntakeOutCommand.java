package org.firstinspires.ftc.teamcode.Echo.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Echo.Subsystems.Intake;
public class IntakeOutCommand extends SequentialCommandGroup {
    public IntakeOutCommand(Intake intake) {
        addCommands(
                new InstantCommand(()->intake.samplePresent = false),
                new InstantCommand(intake::out),
                new WaitCommand(2000),
                new InstantCommand(intake::stop)
        );
    }
}
