package org.firstinspires.ftc.teamcode.Echo.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Echo.Commands.LiftToStateCommand;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Wrist;

//@Disabled
@TeleOp(name = "LiftTest")
public class LiftTest extends CommandOpMode {
    private GamepadEx aparatus;

    private Lift lift;
    private Arm arm;
    private Wrist wrist;
    private Gripper gripper;
    private DcMotorEx mFL, mFR, mBL, mBR;
    double FB, LR, Rotation;
    static double FAST_SPEED_MULTIPLIER = 1;
    static double SLOW_SPEED_MULTIPLIER = 0.4;

    //CURRENT_SPEED_MULTIPLIER is the actual multiplier applied to the drive train power. It is set to either the fast or slow multipliers
    double CURRENT_SPEED_MULTIPLIER;

    @Override
    public void initialize() {
        aparatus = new GamepadEx(gamepad1);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        gripper = new Gripper(hardwareMap);
        wrist = new Wrist(hardwareMap);

        //map motors
        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");

        //liftEncoder = hardwareMap.get(DcMotorEx.class, "");
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

        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.DPAD_UP))
                .whenActive( new SequentialCommandGroup(
                                new InstantCommand(()->lift.PIDEnabled = true),
                                new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_HIGH, BotPositions.LIFT_TOLERANCE)
                        )
                );
        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.DPAD_LEFT))
                .whenActive(new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_LOW, BotPositions.LIFT_TOLERANCE));
        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.DPAD_DOWN))
                .whenActive(new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT, BotPositions.LIFT_TOLERANCE));

        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.DPAD_RIGHT))
                .whenActive( new SequentialCommandGroup(
                            new InstantCommand(()->lift.PIDEnabled = true),
                            new LiftToStateCommand(lift, BotPositions.LIFT_WALL, BotPositions.LIFT_TOLERANCE)
                        )
                );

        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.A))
                .whenActive( new SequentialCommandGroup(
                        new InstantCommand(()->lift.PIDEnabled = true),
                        new LiftToStateCommand(lift, BotPositions.LIFT_INTAKE, BotPositions.LIFT_TOLERANCE)
                ));
        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.B))
                .whenActive(new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH, BotPositions.LIFT_TOLERANCE));

        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.X))
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(arm::specimen),
                        new InstantCommand(wrist::specimen),
                        new InstantCommand(gripper::close)
                ));

        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.Y))
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(arm::specimenHang),
                        new WaitCommand(750),
                        new InstantCommand(gripper::open)
                ));
        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                .whenActive(new SequentialCommandGroup(

                        new InstantCommand(gripper::open)
                ));

        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.LEFT_BUMPER))
                .whenActive(new SequentialCommandGroup(

                        new InstantCommand(arm::basket)
                ));

//        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.DPAD_DOWN))
//                .whenActive(new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_LOW, 1));
//        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.DPAD_LEFT))
//                .whenActive(new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH, 1));
//        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.DPAD_RIGHT))
//                .whenActive(new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_LOW, 1));
//        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.LEFT_BUMPER))
//                .whenActive(new LiftToStateCommand(lift, BotPositions.LIFT_WALL, 1));
//        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.RIGHT_BUMPER))
//                .whenActive(new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT, 1));
    }
    public void run() {
        super.run();

        lift.ManualMode(-aparatus.getLeftY(), -aparatus.getRightY());

        //applies stick values to motor variables with cubic scaling
//        Rotation = cubicScaling(-gamepad1.right_stick_x) * 0.7;
//        FB = cubicScaling(gamepad1.left_stick_y);
//        LR = cubicScaling(-gamepad1.left_stick_x) * 1.2;
//
//        //defines the powers for the motors based on the stick inputs (trust i've written this so many times)
//        double mFLPower = FB + LR + Rotation;
//        double mFRPower = FB - LR - Rotation;
//        double mBLPower = FB - LR + Rotation;
//        double mBRPower = FB + LR - Rotation;
//
//        //actually sets the motor powers
//        mFL.setPower(mFLPower * CURRENT_SPEED_MULTIPLIER);
//        mFR.setPower(mFRPower * CURRENT_SPEED_MULTIPLIER);
//        mBL.setPower(mBLPower * CURRENT_SPEED_MULTIPLIER);
//        mBR.setPower(mBRPower * CURRENT_SPEED_MULTIPLIER);

        telemetry.addData("LeftStick", aparatus.getLeftY());
        telemetry.addData("RightStick", aparatus.getRightY());
        telemetry.addData("liftPosition", lift.getCurrentPosition());
        telemetry.addData("TargetPosition", lift.getTargetPosition());
        telemetry.addData("PID power", lift.getCurrentPID());
        telemetry.addData("LiftAssignedPower", lift.motorPower);
        telemetry.update();


    }
    private double cubicScaling(float joystickValue) {
        //store 5% of the joystick value + 95% of the joystick value to the 3rd power
        double v = 0.05 * joystickValue + 0.95 * Math.pow(joystickValue, 3);
        if (joystickValue > 0.02)
            //if the joystick is positive, return positive .1 + the stored value
            return 0.1 + v;
        else if (joystickValue < -0.02)
            //if the joystick is negative, return -.1 plus the stored value
            return -0.1 + v;
            // theres a range where this won't do either, which is a good counter against stick drift (because you can never escape stick drift)
        else
            return 0;
    }
}
