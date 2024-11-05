package org.firstinspires.ftc.teamcode.Andie.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends SubsystemBase {
    private Servo sAL;
    private Servo sAR;

    public Arm(HardwareMap hardwareMap){
        sAL = hardwareMap.get(Servo.class, "sAL");
        sAR = hardwareMap.get(Servo.class, "sAR");
    }

    @Override

    public void periodic(){}

    public void armSpecimen(){
        sW.setPosition(BotPositions.WRIST_TRANSIT);
    }
    public void armWall(){
        sW.setPosition(BotPositions.WRIST_WALL);
    }
    public void armTransit(){
        sW.setPosition(BotPositions.WRIST_INTAKE);
    }
    public void armIntake(){sAl.setPosition(BotPositions.WRIST_BASKET);
        sAR
    }
}
