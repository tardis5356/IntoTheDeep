package org.firstinspires.ftc.teamcode.Echo.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends SubsystemBase {
    private Servo sAL; //left servo driving the arm
    public Servo sAR; //right servo driving the arm

    public Arm(HardwareMap hardwareMap){
        sAL = hardwareMap.get(Servo.class, "sAL"); //hardware map both of the servos to the on robot configuration
        sAR = hardwareMap.get(Servo.class, "sAR");
        //generally we keep the configuration name and code name the same.
        //It makes us mentally associate the physical servos with the code objects.

        //sAL.setPosition(BotPositions.ARM_INTAKE);
        //sAR.setPosition(BotPositions.ARM_INTAKE);
    }

    @Override

    //runs every 'frame.' whatever you put inside this will run continuously
    public void periodic(){}

    //the following methods all drive the arm to a set position.
    //the name of the method is the position the arm will be driven to.
    //keeping the name of the method and the desired position the same makes things easier to think about when your trying to call these in other code
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
    public void hang(){
        sAL.setPosition(BotPositions.ARM_HANG);
        sAL.setPosition(BotPositions.ARM_HANG);
    }
}
