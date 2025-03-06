package org.firstinspires.ftc.teamcode.Echo.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Echo.Subsystems.AllianceColor;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.VIntake;

public class IntakeGetSampleSubCommand extends CommandBase {//This is a separate command used to actually set the target position of the lift for the PID
    private VIntake vintake;//create a lift object. It will have all the associated code of the lift file since that file outlines a class
    private ElapsedTime runtime = new ElapsedTime();
    private Double timeout = 4.0;
    public IntakeGetSampleSubCommand(VIntake vintake) {
        // this is the actual method itself. It takes a lift as an input to associate with its own, that way it can change the target position value of the lift.
        // Does the same thing with the target position, taking a double in as an input
        // and same thing with the tolerance

        this.vintake = vintake;

    }

    @Override
    public void initialize() { // runs once

        //TODO Test run this
        vintake.Intaking = true;
      new InstantCommand(vintake::in);
        new SequentialCommandGroup(
                new InstantCommand(vintake::downPosition),
                new WaitCommand(200),
                new InstantCommand(vintake::upPosition),
                new WaitCommand(200)
        );
        runtime.reset();
//        lift.setTargetPosition(targetPosition);

    }

    @Override
    public void execute() {



    }

    @Override
    public boolean isFinished() { // returns true when finished
        if ( (vintake.checkSample() && (vintake.checkColor() == AllianceColor.aColor || vintake.checkColor() == "yellow"))||runtime.seconds()>timeout){
            return true;
        }


        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

}
