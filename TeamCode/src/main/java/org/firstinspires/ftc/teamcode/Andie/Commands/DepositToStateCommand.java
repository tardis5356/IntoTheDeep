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

    public String depositCurrentState = "";


    public DepositToStateCommand(Arm arm, Wrist wrist, Gripper gripper, Lift lift, String state) {
        switch (state) {
            case "basketToIntake":
                depositCurrentState = "intake";
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(wrist::intake),
                                new InstantCommand(arm::intake)
//                                new WaitCommand(400),
//                                new InstantCommand(gripper::intakeGripper),
//                                new LiftToStateCommand(lift, 0, 25)
                                //new InstantCommand(wrist::wristIntake)

                        )
                );
                break;
            case "wallToIntake":
                depositCurrentState = "intake";
                addCommands(
                        new SequentialCommandGroup(
//                                new LiftToStateCommand(lift, 10, 25),
                                new InstantCommand(wrist::tuck),
                                new WaitCommand(10),
                                new InstantCommand(arm::intake),
                                new WaitCommand(400),
                                new InstantCommand(wrist::intake),
                                new InstantCommand(gripper::intake),
                                new LiftToStateCommand(lift, 0, 25)

                        )
                );
                break;
            case "intakeToWall":
                depositCurrentState = "wall";
                addCommands(
                        new SequentialCommandGroup(
//                                new LiftToStateCommand(lift, 10, 25),
//                                new InstantCommand(wrist::wristTuck),
//                                new WaitCommand(100),
                                new InstantCommand(arm::wall),
//                                new WaitCommand(100),
                                new InstantCommand(wrist::wall)
                        )
                );
                break;
            case "basketToWall":
                depositCurrentState = "wall";
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(wrist::tuck),
                                new WaitCommand(250),
                                new InstantCommand(arm::transit),
                                new LiftToStateCommand(lift, 10, 25),
                                new WaitCommand(500),
                                new InstantCommand(arm::wall),
                                new WaitCommand(100),
                                new InstantCommand(wrist::wall),
                                new WaitCommand(100)

                        )
                );
                break;
            case "intakeToBasket":
                depositCurrentState = "basket";
                addCommands(
                        new SequentialCommandGroup(
//                                new LiftToStateCommand(lift, LIFT_BASKET_LOW, 25),
//                                new WaitCommand(250),
                                new InstantCommand(wrist::basket),
                                new InstantCommand(arm::basket)
                        )
                );
                break;

            case "wallToBasket":
                depositCurrentState = "basket";
                addCommands(
                        new SequentialCommandGroup(
//                                new LiftToStateCommand(lift, LIFT_BASKET_LOW, 25),
//                                new WaitCommand(250),
                                new InstantCommand(wrist::basket),
                                new InstantCommand(arm::basket)
                        )
                );

            case "intakeToSpecimen":
                depositCurrentState = "specimen";
                addCommands(
                        new SequentialCommandGroup(
//                                new LiftToStateCommand(lift, LIFT_BASKET_LOW, 25),
//                                new WaitCommand(250),
                                new InstantCommand(wrist::specimen),
                                new InstantCommand(arm::specimen)
                        )
                );

            case "specimenToIntake":
                depositCurrentState = "intake";
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(gripper::intake),
                                new InstantCommand(wrist::intake),
                                new InstantCommand(arm::intake)
//                                new LiftToStateCommand(lift, LIFT_BASKET_LOW, 25),
//                                new WaitCommand(250),
                        )

                );

            case "wallToSpecimen":
                depositCurrentState = "specimen";
                addCommands(
//                        new LiftToStateCommand(lift, LIFT_BASKET_LOW, 25),
//                        new WaitCommand(250),
                        new InstantCommand(arm::specimen),
                        new InstantCommand(wrist::specimen),
                        new InstantCommand(gripper::open)
                );
            case "specimenToWall":
                depositCurrentState = "wall";
                addCommands(
//                        new LiftToStateCommand(lift, LIFT_BASKET_LOW, 25),
//                        new WaitCommand(250),
                        new InstantCommand(arm::wall),
                        new InstantCommand(wrist::wall)
                );
            case "specimenToBasket":
                depositCurrentState = "basket";
                addCommands(
//                        new LiftToStateCommand(lift, LIFT_BASKET_LOW, 25),
//                        new WaitCommand(250),
                        new InstantCommand(arm::basket),
                        new InstantCommand(wrist::basket)
                );
            case "basketToSpecimen":
                depositCurrentState = "specimen";
                addCommands(
//                        new LiftToStateCommand(lift, LIFT_BASKET_LOW, 25),
//                        new WaitCommand(250),
                        new InstantCommand(arm::specimen),
                        new InstantCommand(wrist::specimen)
                );

//            case "transit":
//                currentState = "transit";
//                addCommands(
//                        new SequentialCommandGroup(
//                                new InstantCommand(wrist::wristTuck),
//                                new InstantCommand(arm::armTransit),
//                                new LiftToStateCommand(lift, LIFT_WALL, 25)
//                        )
//                );
//                break;

        }
    }
    public void setBasket(){
        depositCurrentState = "basket";
    }
    public void setIntake(){
        depositCurrentState = "intake";
    }
    public void setWall(){
        depositCurrentState = "wall";
    }
}