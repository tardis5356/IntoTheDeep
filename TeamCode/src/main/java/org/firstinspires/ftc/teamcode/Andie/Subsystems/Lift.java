package org.firstinspires.ftc.teamcode.Andie.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Lift extends SubsystemBase {
    DcMotorEx mLT, mLB;
    TouchSensor limitLift;


    PIDController controller;

    public static double targetPosition = 0;
    double joystickPowerInput = 0;
    double motorPower = 0;
    boolean tooHigh;

    public Lift(HardwareMap hardwareMap) {
        mLT = hardwareMap.get(DcMotorEx.class, "mLT");
        mLB = hardwareMap.get(DcMotorEx.class, "mLB");

        limitLift = hardwareMap.get(TouchSensor.class, "lL");

        mLT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        mLT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mLT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mLT.setDirection(DcMotorSimple.Direction.REVERSE);
        mLB.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDController(0.02, 0, 0);
    }

    public void ManualMode(double left, double right) {
        joystickPowerInput = left + right * 0.5;

    }

    public void periodic() {
        // runs every loop
        if(limitLift.isPressed()){
            mLT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            targetPosition = -10;
        }
        if(getCurrentPosition() < BotPositions.LIFT_LIMIT ){
            tooHigh = true;
        } else {tooHigh = false;}

        if (joystickPowerInput != 0 && !limitLift.isPressed() && !tooHigh) {
            motorPower = joystickPowerInput - BotPositions.ANTI_GRAV;
            targetPosition = -15;
        }
        else if(joystickPowerInput != 0 && limitLift.isPressed()){
            if(joystickPowerInput > 0 ){
                motorPower = 0;
            }
            else if(joystickPowerInput <= 0 ){
                motorPower = joystickPowerInput - BotPositions.ANTI_GRAV;
                targetPosition = -15;
            }
        }
        else if(joystickPowerInput != 0 && tooHigh){
            if(joystickPowerInput < 0 ){
                motorPower = 0;
            }
            else if(joystickPowerInput >= 0 ){
                motorPower = joystickPowerInput - BotPositions.ANTI_GRAV;
                targetPosition = -15;
            }
        }
        else if (targetPosition == -10) {
            motorPower = 0;
        } else if (targetPosition != -15) {
            motorPower = -BotPositions.ANTI_GRAV + getCurrentPID();
        } else {
            motorPower = -BotPositions.ANTI_GRAV;
        }

        mLT.setPower(motorPower);
        mLB.setPower(motorPower);
    }

    public double getCurrentPosition() {
        return mLT.getCurrentPosition();
    }
    public double getTargetPosition(){return targetPosition;}

    public double getCurrentPID() {
        return controller.calculate(mLT.getCurrentPosition(), targetPosition);
    }

    public void setTargetPosition(double newTargetPosition){
        targetPosition = newTargetPosition;
    }

    public double getCurrentMotorPower() {
        return motorPower;
    }
}
