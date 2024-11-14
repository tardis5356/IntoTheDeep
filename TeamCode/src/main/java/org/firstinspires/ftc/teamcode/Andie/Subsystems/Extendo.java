package org.firstinspires.ftc.teamcode.Andie.Subsystems;

import static org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions.EXTENDO_IN;
import static org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions.EXTENDO_MIDDLE;
import static org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions.EXTENDO_OUT;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Extendo extends SubsystemBase {
    public Servo sER, sEL;
    public double extensionPosition;
    double stickInput;

    public Extendo(HardwareMap hardwareMap, double stickPosition){
        sER = hardwareMap.get(Servo.class, "sER");
        sEL = hardwareMap.get(Servo.class, "sEL");
        stickInput = stickPosition;
        extensionPosition = EXTENDO_IN;
    }

    @Override

    public void periodic(){
        sER.setPosition(extensionPosition);
        sEL.setPosition(extensionPosition);
        extensionPosition += stickInput;

    }

    public void extendoIn(){
        extensionPosition = EXTENDO_IN;
    }
    public void extendoOut(){
        extensionPosition = EXTENDO_OUT;
    }
    public void extendoMiddle(){
        extensionPosition = EXTENDO_MIDDLE;
    }
}
