package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.SwerveDrive;

public class IntakeCommand extends Command{
    private OperatorInterface oi;
    private Intake intake;
    public IntakeCommand(OperatorInterface oi, Intake intake){
        this.oi = oi;
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.tick(oi);
    }
}