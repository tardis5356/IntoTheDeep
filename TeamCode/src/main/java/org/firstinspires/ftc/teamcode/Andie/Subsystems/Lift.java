package org.firstinspires.ftc.teamcode.Andie.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Lift extends SubsystemBase {
    DcMotorEx mLB, mLT;
    TouchSensor limitLift;

    PIDController controller;

    public static double targetPosition = 0;
    double joystickPowerInput = 0;
    double motorPower = 0;

    public Lift(HardwareMap hardwareMap) {
        mLB = hardwareMap.get(DcMotorEx.class, "mLB");
        mLT = hardwareMap.get(DcMotorEx.class, "mLT");

        limitLift = hardwareMap.get(TouchSensor.class, "lL");

        mLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mLT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        mLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mLT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mLB.setDirection(DcMotorSimple.Direction.REVERSE);
        mLT.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDController(0.02, 0, 0);
    }

    public void ManualMode(double left, double right) {
        joystickPowerInput = left + right * 0.5;
    }

    public void periodic() {
        // runs every loop
        if (joystickPowerInput != 0) {
            motorPower = joystickPowerInput - 0.09;
            targetPosition = -15;
        } else if (targetPosition == -10) {
            motorPower = 0;
        } else if (targetPosition != -15) {
            motorPower = -0.09 + getCurrentPID();
        } else {
            motorPower = -0.09;
        }

        mLB.setPower(motorPower);
        mLT.setPower(motorPower);
    }

    public double getCurrentPosition() {
        return mLB.getCurrentPosition();
    }

    public double getCurrentPID() {
        return -controller.calculate(-mLB.getCurrentPosition(), targetPosition);
    }

    public void setTargetPosition(double newTargetPosition){
        targetPosition = newTargetPosition;
    }

    public double getCurrentMotorPower() {
        return motorPower;
    }
}
