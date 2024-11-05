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
        sensorClose = true;
    }

    @Override

    public void periodic(){
        if(checkGripper(gripperClosed, sensorClose) == true){
            closeGripper();
        }
    }

    public void openGripper(){
        sG.setPosition(BotPositions.GRIPPER_OPEN);
        gripperClosed = false;
    }
    public void closeGripper(){
        sG.setPosition(BotPositions.GRIPPER_CLOSED);
        gripperClosed = true;
    }
    public void intakeGripper(){
        sG.setPosition(BotPositions.GRIPPER_INTAKE);
        gripperClosed = false;
    }
    public boolean checkGripper(boolean gripperState, boolean autoClose){
        if ((((DistanceSensor) cG).getDistance(DistanceUnit.CM) <= 2.5) && gripperState == false && autoClose == true){
            return true;
        }
        else return false;
    }
}
