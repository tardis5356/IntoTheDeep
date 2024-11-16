package org.firstinspires.ftc.teamcode.Andie.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist extends SubsystemBase {
    private Servo sW;

    public Wrist(HardwareMap hardwareMap){
        sW = hardwareMap.get(Servo.class, "sW");
        sW.setPosition(BotPositions.WRIST_INTAKE);
    }

    @Override

    public void periodic(){}

    public void wristTuck(){
        sW.setPosition(BotPositions.WRIST_TRANSIT);
    }
    public void wristSpecimen(){
        sW.setPosition(BotPositions.WRIST_WALL);
    }
    public void wristIntake(){
        sW.setPosition(BotPositions.WRIST_INTAKE);
    }
    public void wristBasket(){sW.setPosition(BotPositions.WRIST_BASKET);}
}
