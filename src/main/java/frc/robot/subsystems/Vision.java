package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase{

    private LimelightResults results;

    public Vision(){
        
    }

    @Override
    public void periodic(){
        results = LimelightHelpers.getLatestResults("limelight");
    }

    public boolean hasTargets(){
        return results.targets_Fiducials.length > 0;
    }

    public PoseEstimate getRobotPose(){
        return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    }

    public void enableLight(){
        LimelightHelpers.setLEDMode_ForceOn("limelight");
    }

    public void disableLight(){
        LimelightHelpers.setLEDMode_ForceOff("limelight");
    }

    public double getTagYaw(){
        return results.targets_Fiducials[0].getTargetPose_RobotSpace().getRotation().getAngle();
    }
}