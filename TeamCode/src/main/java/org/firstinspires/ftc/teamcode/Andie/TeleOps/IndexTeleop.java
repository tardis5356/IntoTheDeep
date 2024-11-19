package org.firstinspires.ftc.teamcode.Andie.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Andie.Commands.LiftToStateCommand;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Wrist;

@TeleOp(name = "indexTele")
//@Disabled
public class IndexTeleop extends CommandOpMode {

    //gamepads
    private GamepadEx driver1, driver2;

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

    //extendo
    //private Extendo extendo;

    //arm
    private Arm arm;

    double Trigger;


    @Override
    public void initialize() {
        //init controllers
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");

        mBR.setDirection(DcMotorSimple.Direction.REVERSE);
        mFR.setDirection(DcMotorSimple.Direction.REVERSE);
        //mBL.setDirection(DcMotorSimple.Direction.REVERSE);
        //mFL.setDirection(DcMotorSimple.Direction.REVERSE);

        //makes the motors brake when power = zero. Is better for driver precision
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //gripper
        gripper = new Gripper(hardwareMap);

        //lift
        lift = new Lift(hardwareMap);

        //wrist
        wrist = new Wrist(hardwareMap);

        //intake
        intake = new Intake(hardwareMap);

        //extendo
        //extendo = new Extendo(hardwareMap);

        //arm
        arm = new Arm(hardwareMap);


        new Trigger(() -> driver1.getButton(GamepadKeys.Button.X))
                .whenActive(new InstantCommand(arm::armTransit));

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.Y))
                .whenActive(new InstantCommand(arm::armSpecimen));

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.A))
                .whenActive(new InstantCommand(arm::armIntake));

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.B))
                .whenActive(new InstantCommand(arm::armBasket));

        //intake
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                .whenActive(new InstantCommand(intake::intakeDown));

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.LEFT_BUMPER))
                .whenActive(new InstantCommand(intake::intakeUp));

        //wrist
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_RIGHT))
                .whenActive(new InstantCommand(wrist::wristTuck));

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_LEFT))
                .whenActive(new InstantCommand(wrist::wristSpecimen));

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_UP))
                .whenActive(new InstantCommand(wrist::wristIntake));

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_DOWN))
                .whenActive(new InstantCommand(wrist::wristBasket));

        //Extendo
        //new Trigger(() -> driver1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON))
        //        .whenActive(new InstantCommand(extendo::extendoIn));

        //new Trigger(() -> driver1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON))
        //        .whenActive(new InstantCommand(extendo::extendoOut));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_UP))
                .whenActive(new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_HIGH, 1));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.A))
                .toggleWhenActive(new InstantCommand(gripper::closeGripper), new InstantCommand(gripper::openGripper));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.B))
                .whenActive(new InstantCommand(gripper::intakeGripper));



        new Trigger(() -> driver2.getButton(GamepadKeys.Button.Y))
                .whenActive(new InstantCommand(intake::intakeOut));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.X))
                .whenActive(new InstantCommand(intake::intakeIn));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_RIGHT))
                .whenActive(new InstantCommand(intake::intakeStop));
    }


    public void run() {
        super.run();
        Trigger= driver2.gamepad.left_trigger - driver2.gamepad.right_trigger;

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

        //telemetry.addData("GripperState", gripper.checkGripper());
        telemetry.addData("ArmPosition", arm.sAR.getPosition());
        telemetry.update();
        //return mFLPower;
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

