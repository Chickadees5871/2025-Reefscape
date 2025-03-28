package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase{

    private LimelightResults results;
    private LimelightTarget_Fiducial lastTarget;

    public Vision(){}

    @Override
    public void periodic(){
        results = LimelightHelpers.getLatestResults("limelight");
        if (results.targets_Fiducials.length > 0) {
            lastTarget = results.targets_Fiducials[0];
        }
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

    public double getLastTagYaw(){
        return lastTarget.getTargetPose_RobotSpace().getRotation().getAngle();
    }
}