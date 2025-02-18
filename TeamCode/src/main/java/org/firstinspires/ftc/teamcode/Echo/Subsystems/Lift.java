package org.firstinspires.ftc.teamcode.Echo.Subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Lift extends SubsystemBase {
    public DcMotorEx mLT; //top motor driving the lift
    public DcMotorEx mLB; //bottom motor driving the lift
    public DcMotorEx mLF;
    TouchSensor limitLift; //touch sensor near the bottom of the lift slides used to localize the lift encoder
//and prevent the lift from driving into the deck plate

    public PIDController controller;
    //PID explanation. The PIDController is a neat thing that drives a motor to a position and holds it there
    //You will use PIDObject.calculate, which takes the encoder value of the motor and the desired value (position)
    //and then spits out a motor power to drive the motor to the desired position. If its in a continual loop,
    //it will continuously change this value based on its tuned P I and D variables

    public static double targetPosition = 0;// stores the desired position of the lift in motor ticks
    public double joystickPowerInput = 0;//like the extendo, it stores the stick input, this time as a motor power
    public double motorPower = 0;//stores the final desired motor power, which is then fed into the motors
    boolean tooHigh; //Boolean to check if the lift is to high
    public boolean liftHanging;

    public double liftFF;
    public boolean localized;
    public boolean PIDEnabled;

    public static double liftOffset = 0;

    //hardwaremap virtual components to configuration
    public Lift(HardwareMap hardwareMap) {
        mLT = hardwareMap.get(DcMotorEx.class, "mLT");
        mLB = hardwareMap.get(DcMotorEx.class, "mLB");
        mLF = hardwareMap.get(DcMotorEx.class, "mLF");

        limitLift = hardwareMap.get(TouchSensor.class, "lL");

        localized = false;

        liftFF = .6;

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

        //IMPORTANT NOTES ON TUNING PID. The P value can be seen as how fast the lift moves to reach a position. Start very
        //small and slowly increase it until the lift slightly oscillates around the desired location. Then, increase the D value,
        //also starting very small and slowly increasing. The D value determines how much the lift slows down as it approaches the desired value.
        //The I value is for accuracy. It takes how far off you are from your desired location and adds power to close that gap.
        //Its tuned in the same way as the other two values and should be done last after D.
        //This video is a good explanation:
        //https://www.youtube.com/watch?time_continue=7&v=XfAt6hNV8XM&embeds_referring_euri=https%3A%2F%2Fcdn.iframe.ly%2F&source_ve_path=MTM5MTE3LDEzOTExNywyODY2Ng
        controller = new PIDController(BotPositions.LIFT_P, BotPositions.LIFT_I, BotPositions.LIFT_D);
    }

    public void ManualMode(double left, double right) {
        joystickPowerInput = left + right * 0.5;
        //joystickPowerInput = left;
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

        if(limitLift.isPressed()){//localizes the lift if its limit is pressed
            mLT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            mLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftOffset = 0;
            localized = true;
            //targetPosition = 10;
        }

        if(getCurrentPosition() < BotPositions.LIFT_LIMIT ){
            //if the lift reaches its digital height limit, set tooHigh to true
            tooHigh = true;
        } else {tooHigh = false;}
//TODO: Redo this so it doesn't suck to think through
        {
            if(liftHanging){
                liftFF = 0;
            }
            else{
                liftFF = .08;
            }
//

            if (liftHanging){
                motorPower = 10;
                targetPosition = 10;
            }
            else if(joystickPowerInput != 0 && !liftHanging){
                PIDEnabled = false;
                targetPosition = 10;
                //CommandScheduler.getInstance().reset();
                if(((getCurrentPosition()>100 && joystickPowerInput > 0) || (joystickPowerInput < 0 && tooHigh)) && localized == true){
                    motorPower = 0 - liftFF;
                }
                else{
                    motorPower = joystickPowerInput - liftFF;
                }
            }
            else if (PIDEnabled == true && !liftHanging){
                motorPower = -liftFF + getCurrentPID();
            }
            else{
                motorPower = 0 - liftFF;
            }

        }//A super messy if statement to swap between manual and pid and stop the lift from going too high or too low.
        //if (!liftHanging){
            mLB.setPower(motorPower);
            mLF.setPower(motorPower);
            mLT.setPower(motorPower);
        //}

    }

    //these are a few telemetry methods for trouble shooting
    public double getCurrentPosition() {
        return mLT.getCurrentPosition()-liftOffset;
    }
    public double getTargetPosition(){return targetPosition;}

    public double getCurrentPID() {// this is the method that takes the current position and desired position and returns a motor power
        return controller.calculate(mLT.getCurrentPosition()-liftOffset, targetPosition);
    }

    public static void setTargetPosition(double newTargetPosition){// updates the target position to whatever its set as by the LiftToStateCommand
        targetPosition = newTargetPosition;
    }

    public double getCurrentMotorPower() {
        return motorPower;
    }



    //this was for auto with so that we could do a stage one hang by letting the motors sag down to the lower bar.
    //it never worked and I still don't know why
    public void cutPower(){
       mLT.setMotorDisable();
        mLB.setMotorDisable();
        mLT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
