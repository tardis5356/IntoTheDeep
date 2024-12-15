package org.firstinspires.ftc.teamcode.Echo.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist extends SubsystemBase {
    private Servo sW;

    public Wrist(HardwareMap hardwareMap){
        sW = hardwareMap.get(Servo.class, "sW");
        //sW.setPosition(BotPositions.WRIST_INTAKE);
    }

    @Override

    public void periodic(){}

    public void tuck(){
        sW.setPosition(BotPositions.WRIST_TRANSIT);
    }
    public void wall(){
        sW.setPosition(BotPositions.WRIST_WALL);
    }
    public void specimen(){
        sW.setPosition(BotPositions.WRIST_SPECIMEN);
    }
    public void intake(){
        sW.setPosition(BotPositions.WRIST_INTAKE);
    }
    public void basket(){sW.setPosition(BotPositions.WRIST_BASKET);}
}
