package org.firstinspires.ftc.teamcode.Echo.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp (name="TestHang", group = "AGen1")
public class BasicHang extends CommandOpMode {

    DcMotorEx mLT, mLB, mBL, mBR, mFL, mFR;

    double Rotation, LR, FB;

    GamepadEx driver;

    @Override

    public void initialize(){
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

        mLT = hardwareMap.get(DcMotorEx.class, "mLT");
        mLB = hardwareMap.get(DcMotorEx.class, "mLB");

        mLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mLT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mLT.setDirection(DcMotorSimple.Direction.REVERSE);
        mLB.setDirection(DcMotorSimple.Direction.REVERSE);

        driver = new GamepadEx(gamepad1);
    }

    public void run() {

        Rotation = cubicScaling(-gamepad1.right_stick_x);
        FB = cubicScaling(gamepad1.left_stick_y);
        LR = cubicScaling(-gamepad1.left_stick_x) * 1.2;

        //defines the powers for the motors based on the stick inputs (trust i've written this so many times)
        double mFLPower = FB + LR + Rotation;
        double mFRPower = FB - LR - Rotation;
        double mBLPower = FB - LR + Rotation;
        double mBRPower = FB + LR - Rotation;

        //actually sets the motor powers
        mFL.setPower(mFLPower);
        mFR.setPower(mFRPower);
        mBL.setPower(mBLPower);
        mBR.setPower(mBRPower);

        mLT.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
        mLB.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

        telemetry.addData("Current of lift", mLT.getCurrent(CurrentUnit.MILLIAMPS));
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

