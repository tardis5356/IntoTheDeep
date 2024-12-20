package org.firstinspires.ftc.teamcode.Echo.Commands;



import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Echo.Subsystems.Gripper;

public class GripperAutoCloseCommand extends SequentialCommandGroup { ;

    @SuppressLint("NotConstructor")

    private String botState;

    public GripperAutoCloseCommand (Gripper gripper){
        if (botState == "wall" && gripper.verifyGripper()){
            addCommands(
            new SequentialCommandGroup(
                    new InstantCommand(gripper::close),
                    new InstantCommand(this::isFinished)
            ));
        }

    }
}
