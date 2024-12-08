package org.firstinspires.ftc.teamcode.Andie.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Andie.Commands.DepositToStateCommand;
import org.firstinspires.ftc.teamcode.Andie.Commands.IntakeInCommand;
import org.firstinspires.ftc.teamcode.Andie.Commands.IntakeOutCommand;
import org.firstinspires.ftc.teamcode.Andie.Commands.IntakePassCommand;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.AllianceColor;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Winch;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Wrist;

@Config
@TeleOp(name = "Gen1_TeleOp", group = "AGen1")

public class Gen1_TeleOp extends CommandOpMode {
    //gamepads
    private GamepadEx driver1, driver2;
    boolean wrongColorIntaked = false;

    private IntakeInCommand intakeInCommand;

    //drivetrain motors and variables
    private DcMotorEx mFL, mFR, mBL, mBR;
    double FB, LR, Rotation;
    double FAST_SPEED_MULTIPLIER = 1;
    double SLOW_SPEED_MULTIPLIER = 0.5;
    double CURRENT_SPEED_MULTIPLIER = FAST_SPEED_MULTIPLIER;

    //gripper
    private Gripper gripper;

    //lift
    private Lift lift;

    //wrist
    private Wrist wrist;

    //intake
    private Intake intake;

    private Extendo extendo;

    private Arm arm;
    private Winch winch;

    //private ColorSensor cI;

    //private TouchSensor limitLift;

    public Boolean TeamColorRed;

    double Trigger;

    static String DepositState;


    public DepositToStateCommand depositToStateCommand;
    double LeftTrigger;
    double RightTrigger;

    @Override
    public void initialize(){
        DepositState = "intake";

        //init controllers
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        extendo = new Extendo(hardwareMap);
        //gripper
        gripper = new Gripper(hardwareMap);

        //lift
        lift = new Lift(hardwareMap);

        //wrist
        wrist = new Wrist(hardwareMap);

        //intake
        intake = new Intake(hardwareMap);

        //intake
        arm = new Arm(hardwareMap);

        winch = new Winch(hardwareMap);

        TeamColorRed = true;

        intakeInCommand = new IntakeInCommand(intake);

        depositToStateCommand = new DepositToStateCommand(arm,wrist, gripper, lift, "basketToIntake");

        //map motors
        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");

        //cI = hardwareMap.get(ColorSensor.class, "cI");
        //limitLift = hardwareMap.get(TouchSensor.class, "lL");
        //this motor physically runs opposite. For convenience, reverse direction.
        mBR.setDirection(DcMotorSimple.Direction.REVERSE);
        mFR.setDirection(DcMotorSimple.Direction.REVERSE);

        //makes the motors brake when power = zero. Is better for driver precision
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //new DepositToStateCommand(arm,wrist,gripper,lift,"transit");

        //Changes if the drivetrain is in fast mode or slow mode. Thx Graham!
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.B)&&CURRENT_SPEED_MULTIPLIER ==SLOW_SPEED_MULTIPLIER)
                .whenActive(() -> CURRENT_SPEED_MULTIPLIER = FAST_SPEED_MULTIPLIER);
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.B)&&CURRENT_SPEED_MULTIPLIER ==FAST_SPEED_MULTIPLIER)
                .whenActive(() -> CURRENT_SPEED_MULTIPLIER = SLOW_SPEED_MULTIPLIER);


        //gripper Commands
        {new Trigger(() -> driver2.getButton(GamepadKeys.Button.B))
                .toggleWhenActive(new InstantCommand(gripper::open), new InstantCommand(gripper::close));

        new Trigger(()-> (gripper.verifyJig() && DepositState == "intake") || (DepositState == "wall" && gripper.verifyGripper()))
                .whenActive(new InstantCommand(gripper::close));}

        //intake
        {
//            new Trigger (() -> AllianceColor.aColor == "blue")
//                    .whenActive(new InstantCommand(intake::checkBlue));

            //intake tilting
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.LEFT_BUMPER) && extendo.sER.getPosition() <= .72)
                .toggleWhenActive(
                        new SequentialCommandGroup(
                                new InstantCommand(intake::up),
                                new InstantCommand(intake::stop)),
                        new InstantCommand(intake::down));

        new Trigger(() -> extendo.sER.getPosition() >= .62 || driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)!=0)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(intake::up)
                        //new WaitCommand(200),
                        //new InstantCommand(extendo::in),
                        //new WaitCommand(300),
                        /*new InstantCommand(intake::transfer)*/));

        //intake inning and outing
        new Trigger(() -> (driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER) || driver2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) && !intake.checkSample() &&(!driver2.getButton(GamepadKeys.Button.LEFT_BUMPER) || !driver1.getButton(GamepadKeys.Button.Y)))
                .toggleWhenActive(new InstantCommand(intake::in), new InstantCommand(intake::stop));

        new Trigger(()->(intake.checkSample() && intake.samplePresent)||gripper.verifyJig())
                .whenActive(new InstantCommand(intake::stop));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.LEFT_BUMPER) || driver1.getButton(GamepadKeys.Button.Y) || ((AllianceColor.aColor == "blue" && intake.checkRed()) || (AllianceColor.aColor == "red" && intake.checkBlue())))
                .whenActive(new InstantCommand(intake::out));}

        //transfer
        {//new Trigger(()-> intake.checkSample() && DepositState == "intake")
         //       .whenActive(new InstantCommand(intake::transfer));

        new Trigger(()-> driver1.getButton(GamepadKeys.Button.X) || driver2.getButton(GamepadKeys.Button.X))
                .whenActive(new IntakePassCommand(intake));}

        //Extendo
        {new Trigger(() -> driver1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON))
                .whenActive(
                        new SequentialCommandGroup(
                                new InstantCommand(intake::up),
                                new WaitCommand(500),
                                new InstantCommand(extendo::in)
                        )
                );

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON))
                .whenActive(new InstantCommand(extendo::out));}

        //Deposit to state commands
        {
        //ToIntakeCommands
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.A) && (DepositState == "basketHigh" || DepositState == "basketLow"))
                .whenActive(new SequentialCommandGroup(
                        new DepositToStateCommand(arm, wrist, gripper, lift,"basketToIntake"),
                        new InstantCommand(() -> DepositState = "intake")
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.A) && DepositState == "wall")
                .whenActive(new SequentialCommandGroup(
                        new DepositToStateCommand(arm, wrist, gripper, lift,"wallToIntake"),
                        new InstantCommand(() -> DepositState = "intake")
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.A) && DepositState == "specimen")
                .whenActive(new SequentialCommandGroup(
                        new DepositToStateCommand(arm, wrist, gripper, lift,"specimenToIntake"),
                        new InstantCommand(() -> DepositState = "intake")
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.A) && DepositState == "intake")
                .whenActive(new SequentialCommandGroup(
                   new DepositToStateCommand(arm, wrist, gripper, lift, "intakeToIntake"),
                   new InstantCommand(() -> DepositState = "intake")
                ));

        //ToWallCommands
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_LEFT) && (DepositState == "basketHigh" || DepositState == "basketLow"))
                .whenActive(new SequentialCommandGroup(
                        new DepositToStateCommand(arm, wrist, gripper, lift,"basketToWall"),
                        new InstantCommand(() -> DepositState = "wall")
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_LEFT) && DepositState == "intake")
                .whenActive(new SequentialCommandGroup(
                        new DepositToStateCommand(arm, wrist, gripper, lift,"intakeToWall"),
                        new InstantCommand(() -> DepositState = "wall")
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_LEFT) && DepositState == "specimen")
                .whenActive(new SequentialCommandGroup(
                        new DepositToStateCommand(arm, wrist, gripper, lift,"specimenToWall"),
                        new InstantCommand(() -> DepositState = "wall")
                ));

        //ToHighBasket Commands
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_UP) && DepositState == "wall")
                .whenActive(new SequentialCommandGroup(
                        new DepositToStateCommand(arm, wrist, gripper, lift,"wallToBasketHigh"),
                        new InstantCommand(() -> DepositState = "basketHigh")
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_UP) && DepositState == "intake")
                .whenActive(new SequentialCommandGroup(
                        new DepositToStateCommand(arm, wrist, gripper, lift,"intakeToBasketHigh"),
                        new InstantCommand(() -> DepositState = "basketHigh")
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_UP) && DepositState == "specimen")
                .whenActive(new SequentialCommandGroup(
                        new DepositToStateCommand(arm, wrist, gripper, lift,"specimenToBasketHigh"),
                        new InstantCommand(() -> DepositState = "basketHigh")
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_UP) && DepositState == "basketLow")
                .whenActive(new SequentialCommandGroup(
                        new DepositToStateCommand(arm, wrist, gripper, lift,"basketLowToBasketHigh"),
                        new InstantCommand(() -> DepositState = "basketHigh")
                ));

        //To Low Basket Commands
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_DOWN) && DepositState == "basketHigh")
                .whenActive(new SequentialCommandGroup(
                        new DepositToStateCommand(arm, wrist, gripper, lift,"basketHighToBasketLow"),
                        new InstantCommand(() -> DepositState = "basketLow")
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_DOWN) && DepositState == "intake")
                .whenActive(new SequentialCommandGroup(
                        new DepositToStateCommand(arm, wrist, gripper, lift,"intakeToBasketLow"),
                        new InstantCommand(() -> DepositState = "basketLow")
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_DOWN) && DepositState == "wall")
                .whenActive(new SequentialCommandGroup(
                        new DepositToStateCommand(arm, wrist, gripper, lift,"wallToBasketLow"),
                        new InstantCommand(() -> DepositState = "basketLow")
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_DOWN) && DepositState == "specimen")
                .whenActive(new SequentialCommandGroup(
                        new DepositToStateCommand(arm, wrist, gripper, lift,"specimenToBasketLow"),
                        new InstantCommand(() -> DepositState = "basketLow")
                ));

        //To high specimen commands

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_RIGHT) && (DepositState == "basketHigh" || DepositState == "basketLow"))
                .whenActive(new SequentialCommandGroup(
                        new DepositToStateCommand(arm, wrist, gripper, lift,"basketToSpecimen"),
                        new InstantCommand(() -> DepositState = "specimen")
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_RIGHT) && DepositState == "wall")
                .whenActive(new SequentialCommandGroup(
                        new DepositToStateCommand(arm, wrist, gripper, lift,"wallToSpecimen"),
                        new InstantCommand(() -> DepositState = "specimen")
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_RIGHT) && DepositState == "intake")
                .whenActive(new SequentialCommandGroup(
                        new DepositToStateCommand(arm, wrist, gripper, lift,"intakeToSpecimen"),
                        new InstantCommand(() -> DepositState = "specimen")
                ));}



        new Trigger(() -> driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) !=0)
                .whenActive(()-> winch.extend(driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));

        new Trigger(() -> driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) !=0)
                .whenActive(()-> winch.retract(driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));

        new Trigger(() -> driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 0 && driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 0)
                .whenActive(new InstantCommand(winch::stop));
    }
    public void run() {
        super.run();

        lift.hanging(driver2.getButton(GamepadKeys.Button.START));

//        if (extendo.extensionPosition > 0.6) {
//            new SequentialCommandGroup(
//                new InstantCommand(intake::up),
//                new WaitCommand(200),
//                new InstantCommand(extendo::in),
//                new WaitCommand(300),
//                new InstantCommand(intake::transfer)
//                );
//        }

        if ((AllianceColor.aColor == "blue" && intake.checkRed()) || (AllianceColor.aColor == "red" && intake.checkBlue())){
            new SequentialCommandGroup(
                    new IntakeOutCommand(intake)
            );

                wrongColorIntaked = true;
        }


        LeftTrigger = driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

        RightTrigger = driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        Trigger = (LeftTrigger - RightTrigger)/10;

        if(Trigger > .03){
            Trigger = .03;
        }
        else if(Trigger < -.03){
            Trigger = -.03;
        }

        extendo.update(Trigger);

        lift.ManualMode(gamepad2.left_stick_y, gamepad2.right_stick_y);

        //applies stick values to motor variables with cubic scaling
        Rotation = cubicScaling(-gamepad1.right_stick_x);
        FB = cubicScaling(gamepad1.left_stick_y);
        LR = cubicScaling(-gamepad1.left_stick_x) * 1.2;

        //defines the powers for the motors (trust i've written this so many times)
        double mFLPower = FB + LR + Rotation;
        double mFRPower = FB - LR - Rotation;
        double mBLPower = FB - LR + Rotation;
        double mBRPower = FB + LR - Rotation;

        mFL.setPower(mFLPower * CURRENT_SPEED_MULTIPLIER);
        mFR.setPower(mFRPower * CURRENT_SPEED_MULTIPLIER);
        mBL.setPower(mBLPower * CURRENT_SPEED_MULTIPLIER);
        mBR.setPower(mBRPower * CURRENT_SPEED_MULTIPLIER);


        telemetry.addData("RobotState", DepositState);
        telemetry.addData("IntakeState", intake.checkSample());
        telemetry.addData("AssignedExtensionPosition", Trigger);
        telemetry.addData("ActualExtensionPosition", extendo.sER.getPosition());
        telemetry.addData("checkIntake", intake.checkSample());
        telemetry.addData("Red", intake.checkRed());
        telemetry.addData("Blue", intake.checkBlue());
        telemetry.addData("Alliance Color", AllianceColor.aColor);
        telemetry.addData("wrongColorDetected", wrongColorIntaked);
        telemetry.addData("isHanging?", lift.liftHanging);
        telemetry.addData("LiftPower", lift.mLT.getPower());
        telemetry.addData("SpeedMultiplyer", CURRENT_SPEED_MULTIPLIER);
        //telemetry.addData("Yellow", intake.checkYellow());
        //telemetry.addData("ReadingIntake", cI.red());//620-650 Yellow 300-400 Red
        //telemetry.addData("ReadingIntake", cI.blue());//120-250 Blue
        //telemetry.addData("ReadingIntake", cI.green());
        telemetry.update();
    }
    private double cubicScaling(float joystickValue) {
        double v = 0.05 * joystickValue + 0.95 * Math.pow(joystickValue, 3);
        if (joystickValue > 0.02)
            return 0.1 + v;
        else if (joystickValue < -0.02)
            return -0.1 + v;
        else
            return 0;
    }
}
