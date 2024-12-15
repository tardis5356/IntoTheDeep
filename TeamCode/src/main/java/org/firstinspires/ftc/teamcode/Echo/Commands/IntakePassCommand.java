package org.firstinspires.ftc.teamcode.Echo.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Echo.Subsystems.Intake;

public class IntakePassCommand extends SequentialCommandGroup {
    public IntakePassCommand (Intake intake) {
        addCommands(
                new InstantCommand(()->intake.samplePresent = false),
                new InstantCommand(intake::in)//,
                //new WaitCommand(750),
                //new InstantCommand(intake::stop)
        );
    }
}
