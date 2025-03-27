package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.SwerveDrive;

public class DriveCommand extends Command {
    private OperatorInterface oi;
    private SwerveDrive swerveDrive;
    
    public DriveCommand(OperatorInterface oi, SwerveDrive drive){
        this.oi = oi;
        this.swerveDrive = drive;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        swerveDrive.accept(oi.getChassisSpeeds());
    }
}