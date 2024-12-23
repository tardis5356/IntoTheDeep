package org.firstinspires.ftc.teamcode.Echo.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist extends SubsystemBase {
    //okay so really this could have been in the arm or gripper subsystem rather than its own separate file
    //its good practice for subsystem base ig
    //going forward, this should probably just be included in the arm should 2025-2026 season be a similar bot
    private Servo sW; //the servo on the wrist

    public Wrist(HardwareMap hardwareMap){ //hardware map
        sW = hardwareMap.get(Servo.class, "sW");
        //sW.setPosition(BotPositions.WRIST_INTAKE);
    }

    @Override

    public void periodic(){}

    //bunch of methods to set the wrist to a position.
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
