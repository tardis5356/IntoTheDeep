package org.firstinspires.ftc.teamcode.Echo.Subsystems;

import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.EXTENDO_IN;
import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.EXTENDO_OUT;
import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.EXTENDO_SPECLEFT;
import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.EXTENDO_SPECMID;
import static org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions.EXTENDO_SPECRIGHT;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Extendo extends SubsystemBase {
    public Servo sER, sEL; //the left and right extension servos. when you are making multiple objects of the same class, it can be done in one line
    //just make sure you separate each instance with a comma
    public double extensionPosition; //this is the variable that the servos will be set to. Instead of changing the servos in each method, we change this instead
    //using a separate variable also makes it easier to base logic off of it, rather than having to call a getPosition method of the servos
    //there is another reason for using a separate variable which we'll get to shortly
    double stickInput; //this will store a calculated value that's based off of the stick values of the gamepad

    public Extendo(HardwareMap hardwareMap){
        //map the servos to the configuration
        sER = hardwareMap.get(Servo.class, "sER");
        sEL = hardwareMap.get(Servo.class, "sEL");

//the hardware mapping method also serves as an initialization for the subsystem, so you can reverse servos, motors, and set variables and positions in here
        extensionPosition = EXTENDO_IN;
    }

    @Override

    public void periodic(){
        //runs every 'frame'

        //we set the servo's to the extensionPosition here so that they are constantly updated.
        sER.setPosition(extensionPosition);
        sEL.setPosition(extensionPosition);

    }

    //methods that manually set the extensionPosition variable to specific values. These are used in automated sequences
    public void in(){
        extensionPosition = EXTENDO_IN;
    }

    public void out(){
        extensionPosition = EXTENDO_OUT;
    }

    public void leftSpec(){
        extensionPosition = EXTENDO_SPECLEFT;
    }
    public void midSpec(){
        extensionPosition = EXTENDO_SPECMID;
    }
    public void rightSpec(){
        extensionPosition = EXTENDO_SPECRIGHT;
    }

    //this is the manual control method that updates teh extensionPosition variable constantly.
    //this is called in the TeleOp's main run loop instead of the periodic. We do this since it takes the live joystick positions as an input.
    //that is then set to the stickInput variable
    public void update(double stickPosition){
        stickInput = stickPosition;

        //Should the actual position of the extendo be outside the in and out limits of the extendo, set extensionPosition to be the limit
        //its like a mini super simple PID, you can learn more about those in the lift subsystem file.
        if(sER.getPosition() > .78){
            extensionPosition = EXTENDO_IN;
        }
        else if(sER.getPosition() < .5){
            extensionPosition = .5;
        }
        else{
            //if the actual extendo position is within its limits, we add the stickInput value too the extensionPosition variable
            //what this does is give the drivers analog control over the extendo. adding increments rather than directly changing the value gradually changes the position
            extensionPosition += stickInput;
        }
    }
}
