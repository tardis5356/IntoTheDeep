package org.firstinspires.ftc.teamcode.Echo.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Echo.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Wrist;

public class DepositToStateCommand extends ParallelCommandGroup {
    //This beefy program is a file that holds a bunch of sequences for the lift, wrist, gripper, and arm
    //each sequence is its own case. they are all contained in a switch statement, that takes in the string 'desiredState', and then runs the case with the matching name
    //format is: case "name": addCommands(new Sequential/ParallelCommandGroup(new _Command, ... , _Command)); break;
    //in here the naming scheme of the cases is the currentRobotState To desiredRobotState. That pretty much describes what each case does

    public String depositCurrentState = "";


    public DepositToStateCommand(Arm arm, Wrist wrist, Gripper gripper, Lift lift, String desiredState/*, String setState*/) {
        switch (desiredState) {
            case "basketToIntake":
                addCommands(
                        new ParallelCommandGroup(
                                new LiftToStateCommand(lift, BotPositions.LIFT_INTAKE, BotPositions.LIFT_TOLERANCE),
                                new SequentialCommandGroup(
                                        //new WaitCommand(500),
                                        new InstantCommand(wrist::intake),
                                        new InstantCommand(arm::intake),
                                        new InstantCommand(gripper::intake)
                                )
                                //new InstantCommand(wrist::wristIntake)
                        )
                );
                //setState = "intake";
                break;

            case "wallToIntake":
                addCommands(
                        new SequentialCommandGroup(

                                new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT-100, BotPositions.LIFT_TOLERANCE),
                                new WaitCommand(150),
                                new ParallelCommandGroup(new InstantCommand(arm::intake),
                                        new InstantCommand(wrist::intake),
                                        new InstantCommand(gripper::intake)),
                                new WaitCommand(250),
                                new LiftToStateCommand(lift, BotPositions.LIFT_INTAKE, BotPositions.LIFT_TOLERANCE)

                        )
                );
                //setState = "intake";
                break;

            case "intakeToWallWithSomething":
                addCommands(
                        new ParallelCommandGroup(
                                new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT-100, BotPositions.LIFT_TOLERANCE),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new InstantCommand(arm::wall),
                                        new InstantCommand(wrist::wall),
                                        new WaitCommand(300),
                                        new LiftToStateCommand(lift, BotPositions.LIFT_WALL, BotPositions.LIFT_TOLERANCE)
                                )
                                //new InstantCommand(gripper::open)
                        )
                );
                //setState = "wall";
                break;
            case "intakeToWallWithNothing":
                addCommands(
                        new ParallelCommandGroup(
                                new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT-100, BotPositions.LIFT_TOLERANCE),
                                new SequentialCommandGroup(
                                        new WaitCommand(150),
                                        new InstantCommand(arm::wall),
                                        new InstantCommand(wrist::wall),
                                        new WaitCommand(300),
                                        new LiftToStateCommand(lift, BotPositions.LIFT_WALL, BotPositions.LIFT_TOLERANCE),
                                        new InstantCommand(()->gripper.sG.setPosition(BotPositions.GRIPPER_OPEN + .1))
                                )

                                //new InstantCommand(gripper::open)
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
                                new InstantCommand(wrist::wall),
                                new InstantCommand(()->gripper.sG.setPosition(BotPositions.GRIPPER_OPEN + .1))
                                //new InstantCommand(gripper::open)


                        )
                );
                //setState = "wall";
                break;

            case "intakeToBasketHigh":
                addCommands(
                        //Here, in order to have the arm swing as the lift rises, but after a certain amount of time, we use a ParallelCommandGroup
                        //Everything in the ParallelCommandGroup is started at the same time, so what we do is start the LiftToStateCommand, then start
                        //a SequentialCommandGroup that makes the arm and wrist wait before starting to swing up, while the lift is still traveling up
                        new ParallelCommandGroup(

                                new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_HIGH, BotPositions.LIFT_TOLERANCE),
                                new SequentialCommandGroup(
                                        new WaitCommand(700),
                                        new InstantCommand(wrist::basket),
                                        new InstantCommand(arm::basket))

                        )
                );
                //setState = "basket";
                break;

            case "intakeToBasketLow":
                addCommands(
                        new ParallelCommandGroup(

                                new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_LOW, BotPositions.LIFT_TOLERANCE),
                                new SequentialCommandGroup(
                                        //new WaitCommand(500),
                                        new InstantCommand(wrist::basket),
                                        new InstantCommand(arm::basket)
                                )
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
                        new ParallelCommandGroup(

                                new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_HIGH, BotPositions.LIFT_TOLERANCE),
                                new SequentialCommandGroup(new WaitCommand(1000),
                                        new InstantCommand(wrist::basket),
                                        new InstantCommand(arm::basket))

                        )
                );
                //setState = "basket";
                break;

            case "intakeToSpecimen":
                addCommands(
                        new SequentialCommandGroup(

                                new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH, BotPositions.LIFT_TOLERANCE),
                                //new WaitCommand(200),
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
                                new WaitCommand(200),
                                new InstantCommand(arm::specimen),
                                new WaitCommand(200),
                                new InstantCommand(wrist::specimen),
                                new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH, BotPositions.LIFT_TOLERANCE)
                        )

                );
                //setState= "specimen";
                break;

            case "specimenToWall":
                addCommands(
                        new SequentialCommandGroup(

                                new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT, BotPositions.LIFT_TOLERANCE),
                                //new WaitCommand(700),
                                new InstantCommand(wrist::tuck),
                                new WaitCommand(200),
                                new InstantCommand(arm::wall),
                                new WaitCommand(200),
                                new InstantCommand(wrist::wall),
                                new LiftToStateCommand(lift, BotPositions.LIFT_WALL, BotPositions.LIFT_TOLERANCE)),
                        new InstantCommand(()->gripper.sG.setPosition(BotPositions.GRIPPER_OPEN + .1))
                        //new InstantCommand(gripper::open)

                );
                //setState = "wall";
                break;

            case "specimenToBasketLow":
                addCommands(
                        new ParallelCommandGroup(

                                new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_LOW, BotPositions.LIFT_TOLERANCE),
                                //new WaitCommand(500),
                                new InstantCommand(arm::basket),
                                new InstantCommand(wrist::basket))

                );
                //setState = "basket";
                break;

            case "specimenToBasketHigh":
                addCommands(
                        new ParallelCommandGroup(

                                new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_HIGH, BotPositions.LIFT_TOLERANCE),
                               // new WaitCommand(100),
                                new InstantCommand(arm::basket),
                                new InstantCommand(wrist::basket))

                );
                //setState = "basket";
                break;

            case "basketToSpecimen":
                addCommands(
                        new SequentialCommandGroup(

                                new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH, BotPositions.LIFT_TOLERANCE),
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

                                new LiftToStateCommand(lift,BotPositions.LIFT_TRANSIT, 25),
                                new InstantCommand(arm::intake),
                                new InstantCommand(wrist::intake),
                                new InstantCommand(gripper::intake),
                                new LiftToStateCommand(lift, BotPositions.LIFT_INTAKE, 25)
                                //new WaitCommand(700),
                                //new LiftToStateCommand(lift, 0, 10)

                        )
                );
                break;

            case "initHang":
                //currentState = "transit";
                addCommands(
                        new SequentialCommandGroup(

                                new LiftToStateCommand(lift,BotPositions.LIFT_SPECIMEN_HIGH, 25),
                                new InstantCommand(arm::hang),
                                new InstantCommand(wrist::basket)

                                //new WaitCommand(700),
                                //new LiftToStateCommand(lift, 0, 10)

                        )
                );
                break;

        }
    }    //These are no longer used.
    //They set the depositCurrentState to a value that is then used as a condition to select which case is ran
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