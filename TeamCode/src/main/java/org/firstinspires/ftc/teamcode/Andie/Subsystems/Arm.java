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

        sAL.setPosition(BotPositions.ARM_INTAKE);
        sAR.setPosition(BotPositions.ARM_INTAKE);
    }

    @Override

    public void periodic(){}

    public void armSpecimen(){
        sAL.setPosition(BotPositions.ARM_SPECIMEN);
        sAR.setPosition(BotPositions.ARM_SPECIMEN);
    }
    public void armBasket(){
        sAL.setPosition(BotPositions.ARM_BASKET);
        sAR.setPosition(BotPositions.ARM_BASKET);
    }
    public void armTransit(){
        sAL.setPosition(BotPositions.ARM_TRANSIT);
        sAR.setPosition(BotPositions.ARM_TRANSIT);
    }
    public void armIntake(){
        sAL.setPosition(BotPositions.ARM_INTAKE);
        sAR.setPosition(BotPositions.ARM_INTAKE);
    }
}
