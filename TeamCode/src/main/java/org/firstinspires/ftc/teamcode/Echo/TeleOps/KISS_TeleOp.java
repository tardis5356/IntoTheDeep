package org.firstinspires.ftc.teamcode.Echo.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "KISS_TELEOP")
public class KISS_TeleOp extends CommandOpMode {

    public DcMotorEx mLT; //top motor driving the lift
    public DcMotorEx mLB; //bottom motor driving the lift
    public DcMotorEx mLF;

    @Override
    public void initialize() {
        mLT = hardwareMap.get(DcMotorEx.class, "mLT");
        mLB = hardwareMap.get(DcMotorEx.class, "mLB");
        mLF = hardwareMap.get(DcMotorEx.class, "mLF");

        mLT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);//this is an example of using the hardwaremap method as an init
        mLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);//sets the lift motors to sag and not resist anything when they have a power of 0
        mLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        /*
        mLT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        */

        //mLT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//at the start of teleop reset the encoder value to 0 (localize it)
        mLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mLT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mLT.setDirection(DcMotorSimple.Direction.REVERSE);
        mLB.setDirection(DcMotorSimple.Direction.REVERSE);
        mLF.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void run() {
        super.run();

        mLT.setPower(-gamepad1.left_stick_y);
        mLB.setPower(-gamepad1.left_stick_y);
        mLF.setPower(-gamepad1.left_stick_y);
    }
}
