package org.firstinspires.ftc.teamcode.Echo.Commands;



import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Echo.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Intake;

public class GripperAutoCloseCommand extends CommandBase { ;

    private Gripper gripper;//create a lift object. It will have all the associated code of the lift file since that file outlines a class
    private ElapsedTime runtime = new ElapsedTime();
    private Double timeout = 5.0;
    public GripperAutoCloseCommand(Gripper gripper) {
        // this is the actual method itself. It takes a lift as an input to associate with its own, that way it can change the target position value of the lift.
        // Does the same thing with the target position, taking a double in as an input
        // and same thing with the tolerance

        this.gripper = gripper;

    }

    @Override
    public void initialize() { // runs once


        runtime.reset();
//        lift.setTargetPosition(targetPosition);

    }

    @Override
    public void execute() { // runs continuously
        //  lift.setTolerance(tolerance);
        // the execute is like the periodic of subsystems, just for commands instead.
        // here we finally set the lift target position to the target position

        //  lift.updatePIDValues();

    }

    @Override
    public boolean isFinished() { // returns true when finished
        if ( gripper.verifyGripper()||runtime.seconds()>timeout){
            return true;
        }


        return false;
    }

    @Override
    public void end(boolean interrupted) {
        new InstantCommand(gripper::close);
    }

}


