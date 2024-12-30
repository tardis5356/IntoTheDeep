package org.firstinspires.ftc.teamcode.Echo.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Intake;

public class IntakeToFloorCommand extends SequentialCommandGroup {
    public IntakeToFloorCommand(Intake intake){
        new SequentialCommandGroup(
                new InstantCommand(()->intake.sIT.setPosition(BotPositions.INTAKE_WRIST_DOWN)),
                new WaitCommand(250),
                new InstantCommand(()->intake.sIG.setPosition(BotPositions.INTAKE_ARM_DOWN))
        );
    }
}
