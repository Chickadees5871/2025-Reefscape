package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Odometry extends SubsystemBase {

    private final SwerveDrivePoseEstimator poseEst;
    private Pose2d pose;
    private SwerveDrive swerveDrive;

    //
    private final StructPublisher<Pose2d> poseOdoPublisher = NetworkTableInstance.getDefault().getStructTopic("Pose_Odo", Pose2d.struct).publish();
    //private final StructPublisher<Pose2d> poseVibPublisher = NetworkTableInstance.getDefault().getStructTopic("Pose_Vis", Pose2d.struct).publish();


    public Odometry(SwerveDrive swerveDrive, Vision vision) {
        this.swerveDrive = swerveDrive;
        poseEst = new SwerveDrivePoseEstimator(
                Constants.DriveConstants.kDriveKinematics,
                swerveDrive.getRotation2d(),
                swerveDrive.getPositions(),
                new Pose2d());

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose2d,
                this::resetPose,
                swerveDrive::getChassisSpeeds,
                (speeds, feedforwards) -> swerveDrive.accept(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(0.005, 0.00, 0.00),
                        new PIDConstants(0.005, 0.00, 0.00)),
                config,
                () -> {

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
    }

    @Override
    public void periodic() {
        pose = poseEst.update(swerveDrive.getRotation2d(), swerveDrive.getPositions());
        poseOdoPublisher.set(pose);
    }

    public Pose2d getPose2d() {
        return pose;
    }

    public void resetPose(Pose2d val) {
        pose = val;
    }
}
