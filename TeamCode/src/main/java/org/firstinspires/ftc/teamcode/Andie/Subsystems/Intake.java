package org.firstinspires.ftc.teamcode.Andie.Subsystems;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import static org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions.BLUE_MAX;
import static org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions.BLUE_MIN;
import static org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions.RED_MAX;
import static org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions.RED_MIN;
import static org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions.YELLOW_MAX;
import static org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions.YELLOW_MIN;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Andie.Commands.IntakeOutCommand;

public class Intake extends SubsystemBase {

    private IntakeOutCommand intakeOutCommand;

    private Servo sIT;
    private CRServo sIR;
    private CRServo sIL;
    private ColorSensor cI;

    public boolean Intaking;
    public boolean samplePresent;
    public boolean Passing;

    public Intake(HardwareMap hardwareMap){
        sIT = hardwareMap.get(Servo.class, "sIT");
        sIR = hardwareMap.get(CRServo.class, "sIR");
        sIL = hardwareMap.get(CRServo.class, "sIL");
        cI = hardwareMap.get(ColorSensor.class, "cI");
        sIL.setDirection(REVERSE);
        sIT.setPosition(BotPositions.INTAKE_UP);


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

    public void down(){sIT.setPosition(BotPositions.INTAKE_DOWN);}
    public void up(){sIT.setPosition(BotPositions.INTAKE_UP);}



    public void in(){
        Intaking = true;
        sIR.setPower(BotPositions.INTAKE_IN);
        sIL.setPower(BotPositions.INTAKE_IN);

    }
    public void out(){

        sIR.setPower(BotPositions.INTAKE_OUT);
        sIL.setPower(BotPositions.INTAKE_OUT);

    }

    public void transfer(){

        sIR.setPower(BotPositions.INTAKE_IN);
        sIL.setPower(BotPositions.INTAKE_IN);
        Intaking = false;
    }
    public void stop() {
        sIR.setPower(BotPositions.INTAKE_STOP);
        sIL.setPower(BotPositions.INTAKE_STOP);
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
