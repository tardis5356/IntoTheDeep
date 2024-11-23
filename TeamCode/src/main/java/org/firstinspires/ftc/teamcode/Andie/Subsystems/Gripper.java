package org.firstinspires.ftc.teamcode.Andie.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Gripper extends SubsystemBase {
    private Servo sG;
    private ColorSensor cG;
    boolean gripperClosed;
    boolean sensorClose;

    public Gripper(HardwareMap hardwareMap){
        sG = hardwareMap.get(Servo.class, "sG");
        cG = hardwareMap.get(ColorSensor.class, "cG");
        //sensorClose = false;
        sG.setPosition(BotPositions.GRIPPER_INTAKE);
    }

    @Override

    public void periodic(){
        //if(verifyGripper(gripperClosed, sensorClose) == true){
        //if( ((DistanceSensor)cG).getDistance(DistanceUnit.CM) <= 2.5 && !gripperClosed) {
        //    closeGripper();
        //}
        //}

    }

    public void open(){
        sG.setPosition(BotPositions.GRIPPER_OPEN);
        gripperClosed = false;
    }
    public void close(){
        sG.setPosition(BotPositions.GRIPPER_CLOSED);
        gripperClosed = true;
    }
    public void intake(){
        sG.setPosition(BotPositions.GRIPPER_INTAKE);
        gripperClosed = false;
    }
    public boolean verify(){
        if ((((DistanceSensor) cG).getDistance(DistanceUnit.CM) <= 2.5)){
            return true;
        }
        else return false;
    }

}
