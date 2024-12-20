package org.firstinspires.ftc.teamcode.Andie.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Lift extends SubsystemBase {
    public DcMotorEx mLT;
    public DcMotorEx mLB;
    TouchSensor limitLift;


    PIDController controller;

    public static double targetPosition = 0;
    double joystickPowerInput = 0;
    double motorPower = 0;
    boolean tooHigh;
    public boolean liftHanging;


    public Lift(HardwareMap hardwareMap) {
        mLT = hardwareMap.get(DcMotorEx.class, "mLT");
        mLB = hardwareMap.get(DcMotorEx.class, "mLB");

        limitLift = hardwareMap.get(TouchSensor.class, "lL");

        mLT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        /*
        mLT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        */

        mLT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mLT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mLT.setDirection(DcMotorSimple.Direction.REVERSE);
        mLB.setDirection(DcMotorSimple.Direction.REVERSE);

        //IMPORTANT NOTES ON TUNING PID. The P value can be seen as how fast the lift moves to reach a position. Start very
        //small and slowly increase it until the lift slightly occelates around the desired location. Then, increase the D value,
        //also starting very small and slowly increasing. The D value determines how much the lift slows down as it approaches the desired value.
        controller = new PIDController(BotPositions.LIFT_P, 0, BotPositions.LIFT_D);
    }

    public void ManualMode(double left, double right) {
        // joystickPowerInput = left + right * 0.5;
        joystickPowerInput = left;
    }

    public void hanging(boolean amHanging){
        if (amHanging){
            liftHanging = true;
        }
        else{
            liftHanging = false;
        }
    }

    public void periodic() {
        // runs every loop

        if(limitLift.isPressed()){
            mLT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //targetPosition = 10;
        }

        if(getCurrentPosition() < BotPositions.LIFT_LIMIT ){
            tooHigh = true;
        } else {tooHigh = false;}

        if (joystickPowerInput != 0 && !limitLift.isPressed() && !tooHigh && !liftHanging) {
            motorPower = joystickPowerInput - BotPositions.ANTI_GRAV;
            targetPosition = 15;
        }
        else if(joystickPowerInput != 0 && limitLift.isPressed() && !liftHanging){
            if(joystickPowerInput > 0 ){
                motorPower = 0;
            }
            else if(joystickPowerInput <= 0 ){
                motorPower = joystickPowerInput - BotPositions.ANTI_GRAV;
                targetPosition = 15;
            }
        }
        else if(joystickPowerInput != 0 && tooHigh && !liftHanging){
            if(joystickPowerInput < 0 ){
                motorPower = 0;
            }
            else if(joystickPowerInput >= 0 ){
                motorPower = joystickPowerInput - BotPositions.ANTI_GRAV;
                targetPosition = 15;
            }
        }
        else if(liftHanging){
            motorPower = joystickPowerInput;
            targetPosition = 15;
        }
        else if (targetPosition != 15) {
            motorPower = -BotPositions.ANTI_GRAV + getCurrentPID();
        }
        else if (targetPosition == 10) {
            motorPower = 0;
        }
        else {
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

    public void cutPower(){
       mLT.setMotorDisable();
        mLB.setMotorDisable();
        mLT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
