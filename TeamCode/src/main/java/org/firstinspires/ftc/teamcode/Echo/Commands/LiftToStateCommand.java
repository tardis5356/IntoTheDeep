package org.firstinspires.ftc.teamcode.Echo.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Echo.Subsystems.Lift;

public class LiftToStateCommand extends CommandBase {//This is a separate command used to actually set the target position of the lift for the PID
    private Lift lift;//create a lift object. It will have all the associated code of the lift file since that file outlines a class

    int targetPosition;
    int tolerance;//This is a +/- # of ticks on the lift. We have this so that the PID doesn't get stuck oscillating trying to reach an exact value.

    public LiftToStateCommand(Lift lift, int targetPosition, int tolerance) {
        // this is the actual method itself. It takes a lift as an input to associate with its own, that way it can change the target position value of the lift.
        // Does the same thing with the target position, taking a double in as an input
        // and same thing with the tolerance

        this.lift = lift;
        this.targetPosition = targetPosition;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize() { // runs once

//        lift.setTargetPosition(targetPosition);
    }

    @Override
    public void execute() { // runs continuously
        //  lift.setTolerance(tolerance);
        // the execute is like the periodic of subsystems, just for commands instead.
        // here we finally set the lift target position to the target position
        lift.setTargetPosition(targetPosition);
        //  lift.updatePIDValues();
    }

    @Override
    public boolean isFinished() { // returns true when finished
        if (targetPosition == 10)
            return true;
        else
            return Math.abs(lift.getCurrentPosition() - targetPosition) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
//        lift.stop();
    }

}
