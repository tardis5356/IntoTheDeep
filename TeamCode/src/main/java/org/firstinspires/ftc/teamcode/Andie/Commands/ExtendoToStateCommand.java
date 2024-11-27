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
                                new InstantCommand(intake::up),
                                new WaitCommand(300),
                                new InstantCommand(extendo::in)

                        )
                );
                break;

            case "extendoOut":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(extendo::out),
                                new WaitCommand(750)
                        )
                );
                break;

            case "RightSpec":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(extendo::rightSpec),
                                new WaitCommand(200)
                        )
                );
                break;

            case "MidSpec":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(extendo::midSpec),
                                new WaitCommand(200)
                        )
                );
                break;

            case "LeftSpec":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(extendo::leftSpec),
                                new WaitCommand(200),
                                new InstantCommand(intake::down),
                                new WaitCommand(200)
                        )
                );
                break;

        }
    }
}