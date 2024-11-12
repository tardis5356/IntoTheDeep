package org.firstinspires.ftc.teamcode.Andie.Subsystems;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase {
    private Servo sIT;
    private CRServo sIR;
    private CRServo sIL;
    private ColorSensor cI;

    public Intake(HardwareMap hardwareMap){
        sIT = hardwareMap.get(Servo.class, "sIT");
        sIR = hardwareMap.get(CRServo.class, "sIR");
        sIL = hardwareMap.get(CRServo.class, "sIL");
        cI = hardwareMap.get(ColorSensor.class, "cI");
        sIL.setDirection(REVERSE);
        sIT.setPosition(BotPositions.INTAKE_UP);
    }

    @Override

    public void periodic(){}

    public void intakeDown(){sIT.setPosition(BotPositions.INTAKE_DOWN);}
    public void intakeUp(){sIT.setPosition(BotPositions.INTAKE_UP);}



    public void intakeIn(){
        sIR.setPower(BotPositions.INTAKE_IN);
        sIL.setPower(BotPositions.INTAKE_IN);
    }
    public void intakeOut(){
        sIR.setPower(BotPositions.INTAKE_OUT);
        sIL.setPower(BotPositions.INTAKE_OUT);
    }
    public void intakeStop(){
        sIR.setPower(BotPositions.INTAKE_STOP);
        sIL.setPower(BotPositions.INTAKE_STOP);
    }

}
