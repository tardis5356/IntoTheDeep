package org.firstinspires.ftc.teamcode.Andie.Subsystems;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions.BLUE_MAX;
import static org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions.BLUE_MIN;
import static org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions.RED_MAX;
import static org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions.RED_MIN;
import static org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions.YELLOW_MAX;
import static org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions.YELLOW_MIN;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Andie.Commands.IntakeInCommand;
import org.firstinspires.ftc.teamcode.Andie.Commands.IntakeOutCommand;

public class Intake extends SubsystemBase {

    private IntakeOutCommand intakeOutCommand;

    private Servo sIT;
    private CRServo sIR;
    private CRServo sIL;
    private ColorSensor cI;

    public boolean IntakeStopped;

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
        if(checkIntake()) {

            if (checkIntakeBlue()) {

                intakeIn();


            }


            if (checkIntakeRed() ) {

               intakeIn();

            }

        }
    }

    public void intakeDown(){sIT.setPosition(BotPositions.INTAKE_DOWN);}
    public void intakeUp(){sIT.setPosition(BotPositions.INTAKE_UP);}



    public void intakeIn(){
        sIR.setPower(BotPositions.INTAKE_IN);
        sIL.setPower(BotPositions.INTAKE_IN);
        IntakeStopped = false;
    }
    public void intakeOut(){
        sIR.setPower(BotPositions.INTAKE_OUT);
        sIL.setPower(BotPositions.INTAKE_OUT);
        IntakeStopped = false;
    }
    public void intakeStop(){
        sIR.setPower(BotPositions.INTAKE_STOP);
        sIL.setPower(BotPositions.INTAKE_STOP);
        IntakeStopped = true;
    }
    public boolean checkIntakeRed(){
        if (cI.red() >= RED_MIN && cI.red() <= RED_MAX){
            return true;
        }
        else return false;
    }
    public boolean checkIntakeYellow(){
        if (cI.red() >= YELLOW_MIN && cI.red() <= YELLOW_MAX){
            return true;
        }
        else return false;
    }
    public boolean checkIntakeBlue(){
        if (cI.green() >= BLUE_MIN && cI.green() <= BLUE_MAX){
            return true;

        }
        else return false;
    }
    public boolean checkIntake(){
        if (checkIntakeYellow()||checkIntakeBlue()||checkIntakeRed()){
            return true;
        }
        else return false;
    }


}
