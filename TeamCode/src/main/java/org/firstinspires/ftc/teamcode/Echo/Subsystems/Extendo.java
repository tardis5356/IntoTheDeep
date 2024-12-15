package org.firstinspires.ftc.teamcode.Echo.Subsystems;

import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.EXTENDO_IN;
import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.EXTENDO_OUT;
import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.EXTENDO_SPECLEFT;
import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.EXTENDO_SPECMID;
import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.EXTENDO_SPECRIGHT;

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

    public void leftSpec(){
        extensionPosition = EXTENDO_SPECLEFT;
    }
    public void midSpec(){
        extensionPosition = EXTENDO_SPECMID;
    }
    public void rightSpec(){
        extensionPosition = EXTENDO_SPECRIGHT;
    }

    public void update(double stickPosition){
        stickInput = stickPosition;

        if(sER.getPosition() > .78){
            extensionPosition = EXTENDO_IN;
        }
        else if(sER.getPosition() < .4){
            extensionPosition = .4;
        }
        else{
            extensionPosition += stickInput;
        }
    }
}
