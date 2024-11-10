package org.firstinspires.ftc.teamcode.Andie.Subsystems;

import static org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions.EXTENDO_IN;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Extendo extends SubsystemBase {
    private Servo sER, sEL;
    double extensionPosition;
    double stickInput;

    public Extendo(HardwareMap hardwareMap, double stickPosition){
        sER = hardwareMap.get(Servo.class, "sER");
        sEL = hardwareMap.get(Servo.class, "sEL");
        stickInput = stickPosition;
    }

    @Override

    public void periodic(){
        sER.setPosition(extensionPosition);
        sEL.setPosition(extensionPosition);
        extensionPosition += stickInput;
    }

    public void extendoIn(){
        extensionPosition = EXTENDO_IN;
        extensionPosition = EXTENDO_IN;
    }
    public void extendoOut(){
        sER.setPosition(BotPositions.EXTENDO_OUT);
        sEL.setPosition(BotPositions.EXTENDO_OUT);
    }
    public void extendoMiddle(){
        sER.setPosition(BotPositions.EXTENDO_MIDDLE);
        sEL.setPosition(BotPositions.EXTENDO_MIDDLE);
    }
}
