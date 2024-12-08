package org.firstinspires.ftc.teamcode.Andie.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Andie.Commands.DepositToStateCommand;
import org.firstinspires.ftc.teamcode.Andie.Commands.IntakeInCommand;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Wrist;

@Config
@Disabled
@TeleOp(name = "Gen1_Test", group = "AGen1")
public class TestTeleop extends CommandOpMode {

    //gamepads
    private GamepadEx driver1, driver2;

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

    private ColorSensor cI;

    private TouchSensor limitLift;

    public Boolean TeamColorRed;

    double Trigger;


    public DepositToStateCommand depositToStateCommand;
    double LeftTrigger;
    double RightTrigger;

    @Override
    public void initialize() {
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

        TeamColorRed = true;

        intakeInCommand = new IntakeInCommand(intake);

        depositToStateCommand = new DepositToStateCommand(arm,wrist, gripper, lift, "basketToIntake");






        //map motors
        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");

        cI = hardwareMap.get(ColorSensor.class, "cI");
        limitLift = hardwareMap.get(TouchSensor.class, "lL");
        //this motor physically runs opposite. For convenience, reverse direction.
        mBR.setDirection(DcMotorSimple.Direction.REVERSE);
        mFR.setDirection(DcMotorSimple.Direction.REVERSE);
        //mBL.setDirection(DcMotorSimple.Direction.REVERSE);
        //mFL.setDirection(DcMotorSimple.Direction.REVERSE);

        //makes the motors brake when power = zero. Is better for driver precision
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Changes if the drivetrain is in fast mode or slow mode. Thx Graham!
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.B)&&CURRENT_SPEED_MULTIPLIER ==SLOW_SPEED_MULTIPLIER)
                .whenActive(() -> CURRENT_SPEED_MULTIPLIER = FAST_SPEED_MULTIPLIER);
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.B)&&CURRENT_SPEED_MULTIPLIER ==FAST_SPEED_MULTIPLIER)
                .whenActive(() -> CURRENT_SPEED_MULTIPLIER = SLOW_SPEED_MULTIPLIER);
//
//        //gripper Command
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.A))
                .toggleWhenActive(new InstantCommand(gripper::open), new InstantCommand(gripper::close));

        new Trigger(()-> (gripper.verifyJig() && depositToStateCommand.depositCurrentState == "intake") || (depositToStateCommand.depositCurrentState == "wall" && gripper.verifyGripper()))
                .whenActive(new InstantCommand(gripper::close));
//
//        //lift presets
////        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_UP))
////                .whenActive(new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_HIGH, 20));
//
//        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_LEFT))
//                .whenActive(new DepositToStateCommand(arm, wrist, gripper, lift, "basketToIntake"));
//        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_RIGHT))
//
//                .whenActive(new DepositToStateCommand(arm, wrist, gripper, lift, "intakeToBasket"));
//
//        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_DOWN))
//                .whenActive(new DepositToStateCommand(arm, wrist, gripper, lift, "intakeToSpecimen"));
////
////        new Trigger(() -> driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
////                .whenActive();
////

//            new Trigger(() -> !intake.IntakeStopped && !intake.checkIntake())
//                    .whenActive(
//                                    new InstantCommand(intake::intakeStop)
//                            );
//
//        new Trigger(() -> !intake.IntakeStopped && intake.checkIntake())
//                .whenActive(
//                        new InstantCommand(intake::intakeStop)
//                );
//
//
//
//
//        //temporary wrist
//
//
        //if(extendo.sER.getPosition()<=.6) {
            new Trigger(() -> driver1.getButton(GamepadKeys.Button.LEFT_BUMPER) && extendo.sER.getPosition() <= .72)
                    .toggleWhenActive(new SequentialCommandGroup(new InstantCommand(intake::up)), new InstantCommand(intake::down));
        //}

            new Trigger(() -> extendo.sER.getPosition() >= .72)
                    .whenActive(new InstantCommand(intake::up));


        new Trigger(() -> driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER)&&!intake.checkSample())
        .toggleWhenActive(new InstantCommand(intake::in), new InstantCommand(intake::stop));

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER)&&intake.checkSample())
                .whenActive(new InstantCommand(intake::out));

        //outake
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                .toggleWhenActive(new InstantCommand(intake::in), new InstantCommand(intake::stop));
//
//        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_RIGHT))
//                .whenActive(new InstantCommand(intake::intakeStop));
//
//        //Extendo
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON))
                .whenActive(new SequentialCommandGroup(new InstantCommand(intake::up),
                        new WaitCommand(50),
                        new InstantCommand(extendo::in)));

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON))
                .whenActive(new InstantCommand(extendo::out));

        new Trigger(()-> driver1.getButton(GamepadKeys.Button.A))
                .whenActive(new InstantCommand(intake::transfer));

        new Trigger(()-> intake.checkSample() && extendo.sER.getPosition() >= .75&& depositToStateCommand.depositCurrentState == "intake")
                .whenActive(new InstantCommand(intake::transfer));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_UP))
                .whenActive(
                        new SequentialCommandGroup(
                                new InstantCommand(depositToStateCommand::setBasket),
                                new InstantCommand(arm::basket),
                                new InstantCommand(wrist::basket)
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_DOWN))
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(depositToStateCommand::setIntake),
                        new InstantCommand(arm::intake),
                        new InstantCommand(wrist::intake)
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_LEFT) && lift.getCurrentPosition() < -500)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(depositToStateCommand::setWall),
                        new InstantCommand(arm::wall),
                        new InstantCommand(wrist::wall)
                ));
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_RIGHT) && lift.getCurrentPosition() < -500)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(depositToStateCommand::setBasket),
                        new InstantCommand(arm::specimen),
                        new InstantCommand(wrist::specimen)
                ));

    }

    public void run() {
        super.run();

//        if (extendo.extensionPosition > 0.7) {
//
//                new InstantCommand(intake::intakeUp);
//                new WaitCommand(200);
//                new InstantCommand(extendo::in);
//                new WaitCommand(300);
//                new InstantCommand(intake::transfer);
//
//        }


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

        lift.ManualMode(cubicScaling(gamepad2.left_stick_y), gamepad2.right_stick_y);

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


        telemetry.addData("RobotState", depositToStateCommand.depositCurrentState);
        telemetry.addData("IntakeState", intake.checkSample());
        telemetry.addData("AssignedExtensionPosition", Trigger);
        telemetry.addData("ActualExtensionPosition", extendo.sER.getPosition());
        telemetry.addData("checkIntake", intake.checkSample());
        telemetry.addData("Red", intake.checkRed());
        telemetry.addData("Blue", intake.checkBlue());
        //telemetry.addData("Yellow", intake.checkYellow());
        telemetry.addData("ReadingIntake", cI.red());//620-650 Yellow 300-400 Red
        telemetry.addData("ReadingIntake", cI.blue());//120-250 Blue
        telemetry.addData("ReadingIntake", cI.green());
        telemetry.update();
    }

    //Super duper cewl cubic scaling function. if the stick is only +- 4%, nothing happens.
    //Anything greater is cubically scaled, very cool. Also possibly an anti drift measure with tweaking.
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
