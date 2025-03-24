package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.SwerveDrive;

public class LiftCommand extends Command{
    private OperatorInterface oi;
    private Lift lift;
    public LiftCommand(OperatorInterface oi, Lift lift){
        this.oi = oi;
        this.lift = lift;
        addRequirements(lift);
    }

    @Override
    public void execute() {
        lift.tick(oi);
    }
}