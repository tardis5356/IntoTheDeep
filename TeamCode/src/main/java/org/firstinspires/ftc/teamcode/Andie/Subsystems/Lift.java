package org.firstinspires.ftc.teamcode.Andie.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Lift extends SubsystemBase {
    DcMotorEx mLR, mLL;
    TouchSensor limitLift;

    PIDController controller;

    public static double targetPosition = 0;
    double joystickPowerInput = 0;
    double motorPower = 0;

    public Lift(HardwareMap hardwareMap) {
        mLR = hardwareMap.get(DcMotorEx.class, "mLR");
        mLL = hardwareMap.get(DcMotorEx.class, "mLL");

        limitLift = hardwareMap.get(TouchSensor.class, "touchLift");

        mLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mLL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        mLL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mLL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mLR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mLL.setDirection(DcMotorSimple.Direction.REVERSE);
        mLR.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDController(0.02, 0, 0);
    }
}
