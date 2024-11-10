package org.firstinspires.ftc.teamcode.Andie.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Andie.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Wrist;

public class DepositToStateCommand extends ParallelCommandGroup {

    public String currentState = "";
    public int specimenHigh = 40;
    public int specimenWall = 10;

    public int basketHigh = 50;

    public DepositToStateCommand(Arm arm, Wrist wrist, Gripper gripper, Lift lift, String state) {
        switch (state.toLowerCase()) {
            case "basketToIntake":
                currentState = "intake";
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(wrist::wristTuck),
                                new InstantCommand(arm::armIntake),
                                new WaitCommand(400),
                                new LiftToStateCommand(lift, 0, 25),
                                new InstantCommand(wrist::wristIntake),
                                new InstantCommand(gripper::openGripper)
                        )
                );
                break;
            case "specimenToIntake":
                currentState = "intake";
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(wrist::wristTuck),
                                new InstantCommand(arm::armIntake),
                                new WaitCommand(400),
                                new LiftToStateCommand(lift, 0, 25),
                                new InstantCommand(wrist::wristIntake),
                                new InstantCommand(gripper::openGripper)
                        )
                );
                break;
            case "intakeToSpecimen":
                currentState = "specimen";
                addCommands(
                        new SequentialCommandGroup(
                                new LiftToStateCommand(lift, specimenWall, 25),
                                new InstantCommand(arm::armSpecimen),
                                new InstantCommand(wrist::wristSpecimen)
                        )
                );
                break;
            case "basketToSpecimen":
                currentState = "specimen";
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(wrist::wristTuck),
                                new WaitCommand(250),
                                new InstantCommand(arm::armSpecimen),
                                new InstantCommand(wrist::wristSpecimen)
                        )
                );
                break;
            case "intakeToBasket":
                currentState = "basket";
                addCommands(
//                        new IntakeOutCommand(intake),
                        new SequentialCommandGroup(
                                new InstantCommand(wrist::wristBasket),
                                new InstantCommand(arm::armBasket)
                        )
                );
                break;

        }
    }
}