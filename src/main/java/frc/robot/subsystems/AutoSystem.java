package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoSystem extends SubsystemBase {

    private SwerveDrive swerveDrive;

    public AutoSystem(SwerveDrive swerveDrive) {

    this.swerveDrive = swerveDrive;

    RobotConfig config;
    try{
        config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
        e.printStackTrace();
        throw new RuntimeException(e);
    }
  
    // Configure AutoBuilder last
    AutoBuilder.configure(
            swerveDrive::getPose2d, 
            swerveDrive::resetPose,
            swerveDrive::getChassisSpeeds, 
            (speeds, feedforwards) -> swerveDrive.accept(speeds),
            new PPHolonomicDriveController( 
                    new PIDConstants(0.005, 0.00, 0.00), 
                    new PIDConstants(0.005, 0.00, 0.00) 
            ),
            config, 
            () -> {

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
            },
            this
    );
    }
  }