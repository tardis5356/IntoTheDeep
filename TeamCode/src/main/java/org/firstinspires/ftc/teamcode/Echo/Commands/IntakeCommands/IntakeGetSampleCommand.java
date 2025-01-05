package org.firstinspires.ftc.teamcode.Echo.Commands.IntakeCommands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Lift;

public class IntakeGetSampleCommand extends CommandBase {//This is a separate command used to actually set the target position of the lift for the PID
    private Intake intake;//create a lift object. It will have all the associated code of the lift file since that file outlines a class
    private ElapsedTime runtime = new ElapsedTime();
    private Double timeout = 10.0;
    public IntakeGetSampleCommand(Intake intake) {
        // this is the actual method itself. It takes a lift as an input to associate with its own, that way it can change the target position value of the lift.
        // Does the same thing with the target position, taking a double in as an input
        // and same thing with the tolerance

        this.intake = intake;

    }

    @Override
    public void initialize() { // runs once

        //TODO Test run this
        intake.Intaking = true;
        intake.sIW.setPower(BotPositions.INTAKE_IN);
        intake.sIO.setPower(BotPositions.INTAKE_IN);
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
        if ( intake.checkSample()||runtime.seconds()>timeout){
            return true;
        }


        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.sIW.setPower(BotPositions.INTAKE_STOP);
        intake.sIO.setPower(BotPositions.INTAKE_STOP);
        intake.Intaking = false;;
    }

}
