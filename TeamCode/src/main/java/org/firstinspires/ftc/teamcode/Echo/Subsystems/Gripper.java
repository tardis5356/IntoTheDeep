package org.firstinspires.ftc.teamcode.Echo.Subsystems;

import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.BLUE_MIN;
import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.RED_MAX;
import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.RED_MIN;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Gripper extends SubsystemBase {
    private Servo sG;
    public ColorSensor cG;
    private ColorSensor cJ;

    boolean gripperClosed;
    boolean sensorClose;

    public Gripper(HardwareMap hardwareMap){
        sG = hardwareMap.get(Servo.class, "sG");
        cG = hardwareMap.get(ColorSensor.class, "cG");
        cJ = hardwareMap.get(ColorSensor.class, "cJ");
        //sensorClose = false;
        //sG.setPosition(BotPositions.GRIPPER_INTAKE);
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
    public boolean verifyGripper(){
        if ((((DistanceSensor) cG).getDistance(DistanceUnit.CM) <= 4)){
            return true;
        }
        else return false;
    }

    public void checkColor(){
        if(cG.blue() >= BLUE_MIN){
            AllianceColor.aColor = "blue";
        }
        else if(cG.red() >= RED_MIN && cG.red() <= RED_MAX){
            AllianceColor.aColor = "red";
        }
        else{
            AllianceColor.aColor = "huh";
        }
    }

    public boolean verifyJig(){
        if ((((DistanceSensor) cJ).getDistance(DistanceUnit.CM) <= 2)){
            return true;
        }
        else return false;
    }

}
