package org.firstinspires.ftc.teamcode.Echo.Subsystems;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.BLUE_MAX;
import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.BLUE_MIN;
import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.RED_MAX;
import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.RED_MIN;
import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.YELLOW_MAX;
import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.YELLOW_MIN;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Echo.Commands.IntakeOutCommand;

public class Intake extends SubsystemBase {

    private IntakeOutCommand intakeOutCommand;

    private Servo sIG;
    private Servo sIT;
    private CRServo sIW, sIO;
    private ColorSensor cI;

    public boolean Intaking;
    public boolean samplePresent;
    public boolean Passing;

    public Intake(HardwareMap hardwareMap){
        sIG = hardwareMap.get(Servo.class, "sIG");
        sIT = hardwareMap.get(Servo.class, "sIT");
        sIW = hardwareMap.get(CRServo.class, "sIW");
        sIO = hardwareMap.get(CRServo.class, "sIO");
        cI = hardwareMap.get(ColorSensor.class, "cI");
        sIW.setDirection(REVERSE);
        //sIT.setPosition(BotPositions.INTAKE_UP);


    }

    @Override

    public void periodic(){
        if(checkSample()) {
            samplePresent = true;
            //if (checkBlue()) {//Add team color conditionals later
            //    transfer();
            //}else if (checkRed()) {//Add team color conditionals later
            //    transfer();
            //}else {
            //    stop();
            //}
        }
        if(!samplePresent&&!Intaking){
            stop();
        }
    }

    public void downPosition(){
        //drives the intake arm and wrist to the ground for intaking
        //sIT.setPosition(BotPositions.INTAKE_DOWN);
        sIG.setPosition(BotPositions.INTAKE_ARM_DOWN);
        sIT.setPosition(BotPositions.INTAKE_WRIST_DOWN);
    }
    public void neutralPosition(){
        //drives the arm and wrist of the intake to a neutral position to go into the robot and hopefully not scratch the deck plate more
        //sIT.setPosition(BotPositions.INTAKE_UP);
        sIG.setPosition(BotPositions.INTAKE_ARM_NEUTRAL);
        sIT.setPosition(BotPositions.INTAKE_WRIST_NEUTRAL);
    }
    public void transferPosition(){
        //specifically moves the intake arm and wrist to the transfer position
        sIG.setPosition(BotPositions.INTAKE_ARM_TRANSFER);
        sIT.setPosition(BotPositions.INTAKE_WRIST_TRANSFER);
    }

    public void outakePosition(){
        //specifically moves the intake arm and wrist to the back of the robot so samples can be spit out the back
        sIG.setPosition(BotPositions.INTAKE_ARM_OUTAKE);
        sIT.setPosition(BotPositions.INTAKE_WRIST_OUTAKE);
    }

    public void armNeutral(){
        //specifically moves the intake arm and wrist to the back of the robot so samples can be spit out the back
        sIG.setPosition(BotPositions.INTAKE_ARM_NEUTRAL);

    }

    public void wristNeutral(){
        //specifically moves the intake arm and wrist to the back of the robot so samples can be spit out the back
        sIT.setPosition(BotPositions.INTAKE_WRIST_NEUTRAL);
    }



    public void in(){
        Intaking = true;
        sIW.setPower(BotPositions.INTAKE_IN);
        sIO.setPower(BotPositions.INTAKE_IN);

    }
    public void out(){

        sIW.setPower(BotPositions.INTAKE_OUT);
        sIO.setPower(BotPositions.INTAKE_OUT);

    }

    public void transfer(){

        sIW.setPower(BotPositions.INTAKE_IN);
//        sIL.setPower(BotPositions.INTAKE_IN);
        Intaking = false;
    }
    public void stop() {
        sIW.setPower(BotPositions.INTAKE_STOP);
        sIO.setPower(BotPositions.INTAKE_STOP);
        Intaking = false;
    }
    public boolean checkRed(){
        if (cI.red() >= RED_MIN && cI.red() <= RED_MAX){
            return true;
        }
        else return false;
    }
    public boolean checkYellow(){
        if (cI.red() >= YELLOW_MIN && cI.red() <= YELLOW_MAX){
            return true;
        }
        else return false;
    }
    public boolean checkBlue(){
        if (cI.blue() >= BLUE_MIN && cI.blue() <= BLUE_MAX){
            return true;
        }
        else return false;
    }
    public boolean checkSample(){
        if (((DistanceSensor) cI).getDistance(DistanceUnit.CM) <= 2.5){
            return true;
        }
        else return false;
    }


}
