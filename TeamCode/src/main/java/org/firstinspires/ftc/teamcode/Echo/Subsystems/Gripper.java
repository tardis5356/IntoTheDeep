package org.firstinspires.ftc.teamcode.Echo.Subsystems;

import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.INTAKE_BLUE_MIN;
import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.INTAKE_RED_MIN;
//import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.RED_MAX;
//import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.RED_MIN;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Gripper extends SubsystemBase {
    public Servo sG; //the servo of the gripper. opens and closes it
    public ColorSensor cG; // this is a color sensor in the gripper that checks if a sample is inside of it
    //private ColorSensor cJ;

    //boolean gripperClosed;
    boolean gripperClear; // this is a variable to store if a sample is inside the gripper or not.

    public Gripper(HardwareMap hardwareMap){
        //hardware map the objects to the configuration

        sG = hardwareMap.get(Servo.class, "sG");
        cG = hardwareMap.get(ColorSensor.class, "cG");
        //cJ = hardwareMap.get(ColorSensor.class, "cJ");
        //sensorClose = false;
        //sG.setPosition(BotPositions.GRIPPER_INTAKE);
    }

    @Override

    public void periodic(){
        //runs constantly in the background

        //if the gripper was clear and something is detected inside it, the gripper will close.
        if(gripperClear && verifyGripper()){
            close();
        }

    }

    //the following methods set the gripper fingers to different positions. They are triggered either by manual inputs or automated sequences
    public void open(){
        //gripperClear = false;
        sG.setPosition(BotPositions.GRIPPER_OPEN);
    }
    public void close(){
        sG.setPosition(BotPositions.GRIPPER_CLOSED);
        //really the gripper should only be closed if there is a sample inside.
        //We set gripperClear to false, that way it disables the if statement in the periodic loop
        //This is done so when the driver tries to open the gripper, the color sensor still detecting the sample wont trigger the gripper to close again
        gripperClear = false;
    }
    public void intake(){
        sG.setPosition(BotPositions.GRIPPER_INTAKE);
        //gripperClosed = false;
    }
    public boolean verifyGripper(){
        if ((((DistanceSensor) cG).getDistance(DistanceUnit.CM) <= 3)){
            return true;
        }
        else {
            gripperClear = true;
            return false;
        }
    }

    public void checkColor(){
        if(cG.blue() >= INTAKE_BLUE_MIN){
            AllianceColor.aColor = "blue";
        }
        else if(cG.red() >= INTAKE_RED_MIN){
            AllianceColor.aColor = "red";
        }
        else{
            AllianceColor.aColor = "FailedToDetectColor";
        }
    }

    //public boolean verifyJig(){
    //    if ((((DistanceSensor) cJ).getDistance(DistanceUnit.CM) <= 2)){
     //       return true;
     //   }
     //   else return false;
   // }

}
