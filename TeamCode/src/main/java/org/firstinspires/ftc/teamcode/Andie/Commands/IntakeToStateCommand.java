package org.firstinspires.ftc.teamcode.Andie.Commands;

import static org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions.LIFT_WALL;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Andie.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Wrist;

public class IntakeToStateCommand  extends ParallelCommandGroup {

    public String intakeCurrentState = "";


    public IntakeToStateCommand(Extendo extendo, Intake intake, String state) {
        switch (state.toLowerCase()) {
            case "Out":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(intake::intakeNeutral),
                                new InstantCommand(extendo::extendoOut)
                        )
                );
                break;
            case "Middle":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(intake::intakeNeutral),
                                new InstantCommand(extendo::extendoMiddle)
                        )
                );
                break;
            case "In":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(intake::intakeNeutral),
                                new WaitCommand(200),
                                new InstantCommand(extendo::extendoIn)
                        )
                );
                break;
        }
    }
}