package org.firstinspires.ftc.teamcode.Andie.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Andie.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Intake;

public class ExtendoToStateCommand extends ParallelCommandGroup {
    private IntakeInCommand intakeInCommand;

    public ExtendoToStateCommand(Intake intake, Extendo extendo, String state) {
        switch (state) {
            case "extendoIn":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(intake::intakeUp), new WaitCommand(300), new InstantCommand(extendo::in), intakeInCommand

                        )
                );
                break;

        }
    }
}