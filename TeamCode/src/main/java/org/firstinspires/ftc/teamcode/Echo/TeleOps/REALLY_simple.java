package org.firstinspires.ftc.teamcode.Echo.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "wtfHang")
public class REALLY_simple extends LinearOpMode {

    DcMotorEx mLT,mLB,mLF;
    DcMotorEx mBL, mBR, mFL, mFR;
    TouchSensor limitLift;
    double FB, LR, Rotation;

    @Override
    public void runOpMode() {
    waitForStart();

        mLT = hardwareMap.get(DcMotorEx.class, "mLT");
        mLB = hardwareMap.get(DcMotorEx.class, "mLB");
        mLF = hardwareMap.get(DcMotorEx.class, "mLF");

        mLT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//this is an example of using the hardwaremap method as an init
        mLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//sets the lift motors to sag and not resist anything when they have a power of 0
        mLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //mLT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//at the start of teleop reset the encoder value to 0 (localize it)
        mLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mLT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mLT.setDirection(DcMotorSimple.Direction.REVERSE);
        mLB.setDirection(DcMotorSimple.Direction.REVERSE);
        mLF.setDirection(DcMotorSimple.Direction.REVERSE);

        limitLift = hardwareMap.get(TouchSensor.class, "lL");

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

        while (opModeIsActive()) {

            mLB.setPower(gamepad2.left_stick_y);
            mLF.setPower(gamepad2.left_stick_y);
            mLT.setPower(gamepad2.left_stick_y);

            Rotation = cubicScaling(-gamepad1.right_stick_x) * 0.7;
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
        }

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
