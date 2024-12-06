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
                                new LiftToStateCommand(lift, BotPositions.LIFT_INTAKE, BotPositions.LIFT_TOLERANCE)
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
                                new LiftToStateCommand(lift, BotPositions.LIFT_INTAKE, BotPositions.LIFT_TOLERANCE)

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
                                new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT, BotPositions.LIFT_TOLERANCE),
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
                                new WaitCommand(100),
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
                                new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT, BotPositions.LIFT_TOLERANCE),
                                new WaitCommand(500),
                                new InstantCommand(wrist::basket),
                                new InstantCommand(arm::basket),
                                new WaitCommand(600),
                                new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_LOW, BotPositions.LIFT_TOLERANCE)
                        )
                );
                //setState = "basket";
                break;

            case "wallToBasketHigh":
                addCommands(
                        new SequentialCommandGroup(
                                new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_HIGH, BotPositions.LIFT_TOLERANCE),
                                new WaitCommand(100),
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
                                new WaitCommand(500),
                                new LiftToStateCommand(lift, BotPositions.LIFT_INTAKE, BotPositions.LIFT_TOLERANCE)
                        )

                );
                //setState = "intake";
                break;

            case "wallToSpecimen":
                addCommands(
                        new SequentialCommandGroup(
                                new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT, BotPositions.LIFT_TOLERANCE),
                                new InstantCommand(wrist::tuck),
                                new WaitCommand(500),
                                new InstantCommand(arm::specimen),
                                new WaitCommand(400),
                                new InstantCommand(wrist::specimen),
                                new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH, BotPositions.LIFT_TOLERANCE)
                        )

                );
                //setState= "specimen";
                break;

            case "specimenToWall":
                addCommands(
                        new SequentialCommandGroup(new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT, BotPositions.LIFT_TOLERANCE),
                                //new WaitCommand(700),
                                new InstantCommand(wrist::tuck),
                                new WaitCommand(400),
                                new InstantCommand(arm::wall),
                                new WaitCommand(400),
                                new InstantCommand(wrist::wall),
                                new LiftToStateCommand(lift, BotPositions.LIFT_WALL, BotPositions.LIFT_TOLERANCE))

                );
                //setState = "wall";
                break;

            case "specimenToBasketLow":
                addCommands(
                        new SequentialCommandGroup(new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_LOW, BotPositions.LIFT_TOLERANCE),
                                new WaitCommand(500),
                                new InstantCommand(arm::basket),
                                new InstantCommand(wrist::basket))

                );
                //setState = "basket";
                break;

            case "specimenToBasketHigh":
                addCommands(
                        new SequentialCommandGroup(new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_HIGH, BotPositions.LIFT_TOLERANCE),
                                new WaitCommand(100),
                                new InstantCommand(arm::basket),
                                new InstantCommand(wrist::basket))

                );
                //setState = "basket";
                break;

            case "basketToSpecimen":
                addCommands(
                        new SequentialCommandGroup(new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH, BotPositions.LIFT_TOLERANCE),
                                new WaitCommand(500),
                                new InstantCommand(arm::specimen),
                                new InstantCommand(wrist::specimen))

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

            case "innit":
                //currentState = "transit";
                addCommands(
                        new SequentialCommandGroup(
                                new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT, 25),
                                new InstantCommand(arm::intake),
                                new InstantCommand(wrist::intake),
                                new InstantCommand(gripper::open),
                                new WaitCommand(700),
                                new LiftToStateCommand(lift, 0, 10)

                        )
                );
                break;

            case "intakeToIntake":
                //currentState = "transit";
                addCommands(
                        new SequentialCommandGroup(
                                new LiftToStateCommand(lift,BotPositions.LIFT_INTAKE, 25),
                                new InstantCommand(arm::intake),
                                new InstantCommand(wrist::intake),
                                new InstantCommand(gripper::open)
                                //new WaitCommand(700),
                                //new LiftToStateCommand(lift, 0, 10)

                        )
                );
                break;

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