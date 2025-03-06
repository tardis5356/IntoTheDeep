package org.firstinspires.ftc.teamcode.Echo.Subsystems;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Echo.Commands.IntakeCommands.IntakeOutCommand;

public class VIntake extends SubsystemBase {

    private IntakeOutCommand intakeOutCommand;


    public Servo sI;

    public DcMotorEx mI;

    public ColorSensor cI;
    //IntakeToFloorCommand intakeToFloorCommand;

    public boolean Intaking;
    boolean outaking;
    public boolean samplePresent;
    public boolean wasRaised;

    boolean IntakeToggle;

    public VIntake(HardwareMap hardwareMap){
        sI = hardwareMap.get(Servo.class, "sI");
        mI = hardwareMap.get(DcMotorEx.class, "mI");
        cI = hardwareMap.get(ColorSensor.class, "cI");

        mI.setDirection(DcMotorSimple.Direction.REVERSE);

        //sIT.setPosition(BotPositions.INTAKE_UP);


    }

    @Override

    public void periodic(){
        if(checkSample()) {
            samplePresent = true;
//            //if (checkBlue()) {//Add team color conditionals later
//            //    transfer();
//            //}else if (checkRed()) {//Add team color conditionals later
//            //    transfer();
//            //}else {
//            //    stop();
//            //}
        }
//        if((checkColor() == "blue" && AllianceColor.aColor == "red") || (checkColor() == "red" && AllianceColor.aColor == "blue")){
//            out();
//        }


//        if(!samplePresent&&!Intaking){
//            stop();
//            }
    }

    public void downPosition(){
        //drives the intake arm and wrist to the ground for intaking
        //sIT.setPosition(BotPositions.INTAKE_DOWN);

                sI.setPosition(BotPositions.VINTAKE_DOWN);
                wasRaised =false;



    }
    public void transferPosition(){
        //drives the arm and wrist of the intake to a neutral position to go into the robot and hopefully not scratch the deck plate more
        //sIT.setPosition(BotPositions.INTAKE_UP);

        sI.setPosition(BotPositions.VINTAKE_TRANSFER);
        wasRaised =true;

    }
    public void upPosition(){
        //specifically moves the intake arm and wrist to the transfer position
        //wrist up isn't used here since it was determined to be unnecessary
        sI.setPosition(BotPositions.VINTAKE_UP);
        wasRaised = true;

    }

    public void sweepPosition (){
        sI.setPosition(BotPositions.VINTAKE_SWEEP);
        wasRaised = true;
    }









    public void in(){
       mI.setPower(BotPositions.INTAKE_IN);


    }
    public void out(){

        mI.setPower(BotPositions.INTAKE_OUT);


    }


    public void stop() {
        mI.setPower(BotPositions.INTAKE_STOP);


    }
//    public boolean checkRed(){
//        if (cI.red() >= RED_MIN && cI.red() <= RED_MAX){
//            return true;
//        }
//        else return false;
//    }
//    //public boolean checkYellow(){
//    //    if (cI.red() >= YELLOW_MIN && cI.red() <= YELLOW_MAX){
//    //        return true;
//    //    }
//    //    else return false;
//    //}
//    public boolean checkBlue(){
//        if (cI.blue() >= INTAKE_BLUE_MIN && cI.blue() <= BLUE_MAX){
//            return true;
//        }
//        else return false;
//    }
    public String checkColor(){
        if (cI.blue()<100 && cI.green()<320){
            return "red";
        }
        else if (cI.blue()>300){
            return "blue";
        }
        else if (cI.green()>600){
            return "yellow";
        }
        else {
            return "unknown";
        }

    }
    public boolean checkSample(){
        if ( ((DistanceSensor)cI).getDistance(DistanceUnit.CM) <= 3){
            return true;
        }
        else return false;
    }


}
