package org.firstinspires.ftc.teamcode.Echo.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends SubsystemBase {
    private Servo sAL;
    public Servo sAR;

    public Arm(HardwareMap hardwareMap){
        sAL = hardwareMap.get(Servo.class, "sAL");
        sAR = hardwareMap.get(Servo.class, "sAR");

        //sAL.setPosition(BotPositions.ARM_INTAKE);
        //sAR.setPosition(BotPositions.ARM_INTAKE);
    }

    @Override

    public void periodic(){}

    public void specimen(){
        sAL.setPosition(BotPositions.ARM_SPECIMEN);
        sAR.setPosition(BotPositions.ARM_SPECIMEN);
    }
    public void wall(){
        sAL.setPosition(BotPositions.ARM_WALL);
        sAR.setPosition(BotPositions.ARM_WALL);
    }
    public void basket(){
        sAL.setPosition(BotPositions.ARM_BASKET);
        sAR.setPosition(BotPositions.ARM_BASKET);
    }
    public void transit(){
        sAL.setPosition(BotPositions.ARM_TRANSIT);
        sAR.setPosition(BotPositions.ARM_TRANSIT);
    }
    public void intake(){
        sAL.setPosition(BotPositions.ARM_INTAKE);
        sAR.setPosition(BotPositions.ARM_INTAKE);
    }
}
