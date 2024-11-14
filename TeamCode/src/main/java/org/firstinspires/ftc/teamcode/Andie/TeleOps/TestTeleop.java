package org.firstinspires.ftc.teamcode.Andie.TeleOps;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Andie.Commands.DepositToStateCommand;
import org.firstinspires.ftc.teamcode.Andie.Commands.IntakeInCommand;
import org.firstinspires.ftc.teamcode.Andie.Commands.LiftToStateCommand;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Wrist;

@Config
@TeleOp(name = "Gen1_TeleOp", group = "AGen1")
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

    public Boolean TeamColorRed;
    double Trigger;

    double LeftTrigger;
    double RightTrigger;

    @Override
    public void initialize() {
        //init controllers
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        extendo = new Extendo(hardwareMap, Trigger);
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

        LeftTrigger = driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

        RightTrigger = driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);



        //map motors
        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");

        cI = hardwareMap.get(ColorSensor.class, "cI");
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
//        new Trigger(() -> driver1.getButton(GamepadKeys.Button.B))
//                .whenActive(() -> CURRENT_SPEED_MULTIPLIER = FAST_SPEED_MULTIPLIER)
//                .whenActive(new InstantCommand());
//        new Trigger(() -> driver1.getButton(GamepadKeys.Button.X))
//                .whenActive(() -> CURRENT_SPEED_MULTIPLIER = SLOW_SPEED_MULTIPLIER);
//
//        //gripper Command
//        new Trigger(() -> driver2.getButton(GamepadKeys.Button.RIGHT_BUMPER))
//                .toggleWhenActive(new InstantCommand(gripper::openGripper), new InstantCommand(gripper::closeGripper));
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
//
////        new Trigger(() -> driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
////                .whenActive();
////
////        new Trigger(() -> driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1)
////                .whenActive();
//
//
//
//
//        //temporary wrist
//
//
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.LEFT_BUMPER))
                .toggleWhenActive(new SequentialCommandGroup(new InstantCommand(intake::intakeUp)), new InstantCommand(intake::intakeDown));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                .whenActive(intakeInCommand);
//
//        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_RIGHT))
//                .whenActive(new InstantCommand(intake::intakeStop));
//
//        //Extendo
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON))
                .whenActive(new InstantCommand(extendo::extendoIn));

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON))
                .whenActive(new InstantCommand(extendo::extendoOut));




    }

    public void run() {
        super.run();

        Trigger= LeftTrigger - RightTrigger;

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


        telemetry.addData("checkIntake", intake.checkIntake());
        telemetry.addData("Red", intake.checkIntakeRed());
        telemetry.addData("Blue", intake.checkIntakeBlue());
        telemetry.addData("Yellow", intake.checkIntakeYellow());
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
