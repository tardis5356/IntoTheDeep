package org.firstinspires.ftc.teamcode.Echo.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Echo.Subsystems.AllianceColor;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Intake;

//This is a simple TeleOp to test the color sensors on the robot.
//The intake needs to be able to spit out the wrong alliance color sample
//the gripper needs to be able to detect the pre-load's color to define the alliance color
@TeleOp(name = "SensorCalibration", group = "AGen1") //Makes it so that the driver station app recognizes the file as a TeleOp and has a name
public class SensorCalibrationTest extends CommandOpMode {
    Gripper gripper; //make an object with class gripper. We have to do this with each subsystem. Format is: class object;
    Intake intake;

    @Override
    public void initialize(){
        gripper = new Gripper(hardwareMap); //this is a constructor. It says, hey this object needs to run this method a new time.
        //the method is the Gripper method from the gripper subsystem, which was commented as the hardware map, since it is the hardware mapping method

        intake = new Intake(hardwareMap); //we also do this with every subsystem
    }
    public void run() {
        super.run(); //this is the scheduler for all our commands and such. Any command groups are scheduled and ran by this, as well as the periodic loops. Inw, VERY IMPORTANT

        gripper.checkColor(); //This is the method that is called to assign the alliance color.
        //Any methods or if statements written inside the run loop will be ran continuously

        //telemetry lines that print out values. The thing in "" are the caption, and the following item is the variable being printed
        telemetry.addData("Gripper_Detected_Color", AllianceColor.aColor);
        telemetry.addData("Gripper_Blue_Recording", gripper.cG.blue());
        telemetry.addData("Gripper_Red_Recording", gripper.cG.red());

        telemetry.addData("Intake_detected:", intake.checkColor());
        //telemetry.addData("Intake_detected_blue?", intake.checkBlue());
        telemetry.addData("Intake_Blue_Recording", intake.cI.blue());
        telemetry.addData("Intake_Red_Recording", intake.cI.red());
        telemetry.addData("Intake_Green_Recording", intake.cI.green());

        telemetry.update(); //Also super important. This actually prints the data. Make sure you have it.


    }

}
