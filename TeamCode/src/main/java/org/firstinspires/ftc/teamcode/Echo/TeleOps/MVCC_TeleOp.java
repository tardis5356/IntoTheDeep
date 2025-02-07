package org.firstinspires.ftc.teamcode.Echo.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Echo.Commands.DepositToStateCommand;
import org.firstinspires.ftc.teamcode.Echo.Commands.IntakeCommands.IntakeOutCommand;
import org.firstinspires.ftc.teamcode.Echo.Commands.IntakeCommands.IntakePassCommand;
import org.firstinspires.ftc.teamcode.Echo.Commands.IntakeToStateCommand;
import org.firstinspires.ftc.teamcode.Echo.Commands.LiftToStateCommand;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.AllianceColor;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Winch;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Wrist;

@Config
@TeleOp(name = "MVCC_TeleOp", group = "AGen1")
public class MVCC_TeleOp extends CommandOpMode {

    //gamepads
    //GamepadEx is an extended object version of gamepads that has more organized input checks that we use in triggers.
    private GamepadEx driver1, driver2;


    //drive motors
    private DcMotorEx mFL, mFR, mBL, mBR;

    //multipliers applied to the sum of the above variables to evenly change the speed of the drivetrain
    static double FAST_SPEED_MULTIPLIER = 1;
    static double SLOW_SPEED_MULTIPLIER = 0.4;

    //CURRENT_SPEED_MULTIPLIER is the actual multiplier applied to the drive train power. It is set to either the fast or slow multipliers
    double CURRENT_SPEED_MULTIPLIER;

    //Forward and back power, Left and right power, rotation power.
    //All are then added and subtracted in different ways for each drive motor
    double FB, LR, Rotation;

    private Extendo extendo;
    double LeftTrigger;
    double RightTrigger;
    //This is the difference of the two driver1 trigger values that is then added to the position of the extension servos
    double Trigger;

    //intake
    private Intake intake;
    private Servo sEL, sER, sIG;
    boolean wasRaised = false;

    IntakeToStateCommand intakeDown = new IntakeToStateCommand(intake, "intakeDown");

    IntakeToStateCommand intakeUp = new IntakeToStateCommand(intake, "intakeUp");

    @Override
    //stuff that is ran when you click init at the start of teleop.
    public void initialize() {

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        //drive train init
        {
            //map motors
            mFL = hardwareMap.get(DcMotorEx.class, "mFL");
            mFR = hardwareMap.get(DcMotorEx.class, "mFR");
            mBL = hardwareMap.get(DcMotorEx.class, "mBL");
            mBR = hardwareMap.get(DcMotorEx.class, "mBR");

            //this motor physically runs opposite. For convenience, reverse direction.
            mBR.setDirection(DcMotorSimple.Direction.REVERSE);
            mFR.setDirection(DcMotorSimple.Direction.REVERSE);

            //makes the motors brake when power = zero. Is better for driver precision
            mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            sER = hardwareMap.get(Servo.class, "sER");
            sEL = hardwareMap.get(Servo.class, "sEL");
            sIG = hardwareMap.get(Servo.class, "sIG");
        }

        //subsystem hardware maps
        {
            extendo = new Extendo(hardwareMap);

            //intake
            intake = new Intake(hardwareMap);

        }

        //initialization of subsystems for positions
        {
            intake.transferPosition();
        }

        CURRENT_SPEED_MULTIPLIER = FAST_SPEED_MULTIPLIER;
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.B))
                .toggleWhenActive(() -> CURRENT_SPEED_MULTIPLIER = FAST_SPEED_MULTIPLIER, () -> CURRENT_SPEED_MULTIPLIER = SLOW_SPEED_MULTIPLIER);

        //Extendo commands
        {
            new Trigger(() -> driver1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON) && extendo.extensionPosition >= .78)
                    .whenActive(
                            new InstantCommand(extendo::out)
                    );

            new Trigger(() -> (driver1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON) && extendo.extensionPosition < .78) /*|| (intake.checkSample() && ((intake.checkColor() == "red" && AllianceColor.aColor == "red") || (intake.checkColor() == "blue" && AllianceColor.aColor == "blue") || intake.checkColor() == "yellow"))*/)
                    .whenActive(
                            new InstantCommand(extendo::in)
                    );
        }

        //intake commands
        {
            new Trigger(() -> driver1.getButton(GamepadKeys.Button.LEFT_BUMPER) && extendo.sER.getPosition() <= .72 && wasRaised)
                    .whenActive(
                            //we have sequences for the tilting to make sure that the wrist of the intake moves first before the arm
                            //that's done so we don't the intake pinned against the ground
                            new SequentialCommandGroup(
                                    intakeDown,
                                    new InstantCommand(()->wasRaised=false)
                            )
                    );
            new Trigger(() -> driver1.getButton(GamepadKeys.Button.LEFT_BUMPER) && extendo.sER.getPosition() <= .72 && !wasRaised)
                    .whenActive(
                            new SequentialCommandGroup(
                                    intakeUp,
                                    new InstantCommand(()->wasRaised=true)
                            )
                    );
        }

    }

    //this is the main run loop
    public void run() {

        super.run();

        //applies stick values to motor variables with cubic scaling
        Rotation = cubicScaling(-gamepad1.right_stick_x) * 0.7;
        FB = cubicScaling(gamepad1.left_stick_y);
        LR = cubicScaling(-gamepad1.left_stick_x) * 1.2;

        //defines the powers for the motors based on the stick inputs (trust i've written this so many times)
        double mFLPower = FB + LR + Rotation;
        double mFRPower = FB - LR - Rotation;
        double mBLPower = FB - LR + Rotation;

        //these define the left and right trigger values as the real triggers and takes there difference divided by ten
        //as the input of the extendo.update method.
        LeftTrigger = driftLock((float) driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)) / 20;

        RightTrigger = driftLock((float) driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) / 20;

        Trigger = (LeftTrigger - RightTrigger) / 10;

        //additionally if Trigger gets to large in magnitude it is capped that way the extendo can't be manually launched somewhere crazy.
        if (Trigger > .03) {
            Trigger = .025;
        } else if (Trigger < -.03) {
            Trigger = -.025;
        }

        extendo.update(Trigger);
        double mBRPower = FB + LR - Rotation;

        //actually sets the motor powers
        mFL.setPower(mFLPower * CURRENT_SPEED_MULTIPLIER);
        mFR.setPower(mFRPower * CURRENT_SPEED_MULTIPLIER);
        mBL.setPower(mBLPower * CURRENT_SPEED_MULTIPLIER);
        mBR.setPower(mBRPower * CURRENT_SPEED_MULTIPLIER);


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

    private double driftLock(float stickValue) {
        if (stickValue > 0.02 || stickValue < -.02) {
            return stickValue;
        } else {
            return 0;
        }
    }
}
