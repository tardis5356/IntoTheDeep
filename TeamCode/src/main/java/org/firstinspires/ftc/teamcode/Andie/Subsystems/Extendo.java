package org.firstinspires.ftc.teamcode.Andie.Subsystems;

import static org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions.EXTENDO_IN;
import static org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions.EXTENDO_OUT;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Extendo extends SubsystemBase {
    public Servo sER, sEL;
    public double extensionPosition;
    double stickInput;

    public Extendo(HardwareMap hardwareMap){
        sER = hardwareMap.get(Servo.class, "sER");
        sEL = hardwareMap.get(Servo.class, "sEL");
        extensionPosition = EXTENDO_IN;
    }

    @Override

    public void periodic(){

//
        sER.setPosition(extensionPosition);
        sEL.setPosition(extensionPosition);

    }

    public void in(){
        extensionPosition = EXTENDO_IN;
    }
    public void out(){
        extensionPosition = EXTENDO_OUT;
    }

    public void update(double stickPosition){
        stickInput = stickPosition;

        if(sER.getPosition() > .8){
            extensionPosition = .8;
        }
        else if(sER.getPosition() < .4){
            extensionPosition = .4;
        }
        else{
            extensionPosition += stickInput;
        }
    }
}
