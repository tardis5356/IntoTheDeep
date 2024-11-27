package org.firstinspires.ftc.teamcode.Andie.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Andie.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Wrist;

public class DepositToStateCommand extends ParallelCommandGroup {

    public String depositCurrentState = "";


    public DepositToStateCommand(Arm arm, Wrist wrist, Gripper gripper, Lift lift, String desiredState/*, String setState*/) {
        switch (desiredState) {
            case "basketToIntake":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(wrist::intake),
                                new InstantCommand(arm::intake),
                                new WaitCommand(500),
                                new InstantCommand(gripper::intake),
                                new LiftToStateCommand(lift, 0, BotPositions.LIFT_TOLERANCE)
                                //new InstantCommand(wrist::wristIntake)

                        )
                );
                //setState = "intake";
                break;

            case "wallToIntake":
                addCommands(
                        new SequentialCommandGroup(
                                new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT, BotPositions.LIFT_TOLERANCE),
                                new InstantCommand(wrist::tuck),
                                new WaitCommand(500),
                                new InstantCommand(arm::intake),
                                new WaitCommand(500),
                                new InstantCommand(wrist::intake),
                                new InstantCommand(gripper::intake),
                                new LiftToStateCommand(lift, 0, BotPositions.LIFT_TOLERANCE)

                        )
                );
                //setState = "intake";
                break;

            case "intakeToWall":
                addCommands(
                        new SequentialCommandGroup(
                                new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT, BotPositions.LIFT_TOLERANCE),
                                new InstantCommand(wrist::tuck),
                                new WaitCommand(500),
                                new InstantCommand(arm::wall),
                                new WaitCommand(500),
                                new InstantCommand(wrist::wall),
                                new LiftToStateCommand(lift, BotPositions.LIFT_WALL, BotPositions.LIFT_TOLERANCE)
                        )
                );
                //setState = "wall";
                break;

            case "basketToWall":
                addCommands(
                        //maybe edit this one, needs to be tested
                        new SequentialCommandGroup(
                                new InstantCommand(wrist::tuck),
                                new WaitCommand(500),
                                new InstantCommand(arm::wall),
                                new WaitCommand(500),
                                new LiftToStateCommand(lift, BotPositions.LIFT_WALL, BotPositions.LIFT_TOLERANCE),
                                new InstantCommand(wrist::wall)


                        )
                );
                //setState = "wall";
                break;

            case "intakeToBasketHigh":
                addCommands(
                        new SequentialCommandGroup(
                                new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_HIGH, BotPositions.LIFT_TOLERANCE),
                                new WaitCommand(500),
                                new InstantCommand(wrist::basket),
                                new InstantCommand(arm::basket)
                        )
                );
                //setState = "basket";
                break;

            case "intakeToBasketLow":
                addCommands(
                        new SequentialCommandGroup(
                                new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_LOW, BotPositions.LIFT_TOLERANCE),
                                new WaitCommand(500),
                                new InstantCommand(wrist::basket),
                                new InstantCommand(arm::basket)
                        )
                );
                //setState = "basket";
                break;

            case "wallToBasketLow":
                addCommands(
                        new SequentialCommandGroup(
                                new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_LOW, BotPositions.LIFT_TOLERANCE),
                                new WaitCommand(500),
                                new InstantCommand(wrist::basket),
                                new InstantCommand(arm::basket)
                        )
                );
                //setState = "basket";
                break;

            case "wallToBasketHigh":
                addCommands(
                        new SequentialCommandGroup(
                                new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_HIGH, BotPositions.LIFT_TOLERANCE),
                                new WaitCommand(500),
                                new InstantCommand(wrist::basket),
                                new InstantCommand(arm::basket)
                        )
                );
                //setState = "basket";
                break;

            case "intakeToSpecimen":
                addCommands(
                        new SequentialCommandGroup(
                                new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH, BotPositions.LIFT_TOLERANCE),
                                new WaitCommand(500),
                                new InstantCommand(wrist::specimen),
                                new InstantCommand(arm::specimen)
                        )
                );
                //setState = "specimen";
                break;

            case "specimenToIntake":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(gripper::intake),
                                new InstantCommand(wrist::intake),
                                new InstantCommand(arm::intake),
                                new LiftToStateCommand(lift, 0, BotPositions.LIFT_TOLERANCE),
                                new WaitCommand(500)
                        )

                );
                //setState = "intake";
                break;

            case "wallToSpecimen":
                addCommands(
                        new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT, BotPositions.LIFT_TOLERANCE),
                        new WaitCommand(500),
                        new InstantCommand(arm::specimen),
                        new InstantCommand(wrist::specimen),
                        new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH, BotPositions.LIFT_TOLERANCE)
                );
                //setState= "specimen";
                break;

            case "specimenToWall":
                addCommands(
                        new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT, BotPositions.LIFT_TOLERANCE),
                        new WaitCommand(500),
                        new InstantCommand(arm::wall),
                        new InstantCommand(wrist::wall),
                        new LiftToStateCommand(lift, BotPositions.LIFT_WALL, BotPositions.LIFT_TOLERANCE)
                );
                //setState = "wall";
                break;

            case "specimenToBasketLow":
                addCommands(
                        new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_LOW, BotPositions.LIFT_TOLERANCE),
                        new WaitCommand(500),
                        new InstantCommand(arm::basket),
                        new InstantCommand(wrist::basket)
                );
                //setState = "basket";
                break;

            case "specimenToBasketHigh":
                addCommands(
                        new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_HIGH, BotPositions.LIFT_TOLERANCE),
                        new WaitCommand(500),
                        new InstantCommand(arm::basket),
                        new InstantCommand(wrist::basket)
                );
                //setState = "basket";
                break;

            case "basketToSpecimen":
                addCommands(
                        new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH, BotPositions.LIFT_TOLERANCE),
                        new WaitCommand(500),
                        new InstantCommand(arm::specimen),
                        new InstantCommand(wrist::specimen)
                );
                //setState = "specimen";
                break;

            case "basketLowToBasketHigh":
                addCommands(
                        new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_HIGH, BotPositions.LIFT_TOLERANCE)
                );
                //setState = "specimen";
                break;

            case "basketHighToBasketLow":
                addCommands(
                        new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_LOW, BotPositions.LIFT_TOLERANCE)
                );
                //setState = "specimen";
                break;

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