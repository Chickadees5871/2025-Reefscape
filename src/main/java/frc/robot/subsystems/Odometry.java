package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Odometry extends SubsystemBase {

    private final SwerveDrivePoseEstimator poseEst;
    private Pose2d pose;
    private SwerveDrive swerveDrive;
    private Vision vision;

    //
    private final StructPublisher<Pose2d> poseOdoPublisher = NetworkTableInstance.getDefault().getStructTopic("Pose_Odo", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> poseVibPublisher = NetworkTableInstance.getDefault().getStructTopic("Pose_Vis", Pose2d.struct).publish();

    private final Field2d field2d = new Field2d();


    public Odometry(SwerveDrive swerveDrive, Vision vision) {
        this.vision = vision;
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
                (speeds, feedforwards) -> swerveDrive.acceptAuto(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(0.005, 0.00, 0.00),
                        new PIDConstants(0.005, 0.00, 0.00)),
                config,
                this::getAlliance,
                this);
    }

    @Override
    public void periodic() {
        pose = poseEst.update(swerveDrive.getRotation2d().plus(getAlliance() ? new Rotation2d() : Rotation2d.fromDegrees(180)), swerveDrive.getPositions());
        poseOdoPublisher.set(pose);
        field2d.setRobotPose(pose);
        SmartDashboard.putData(field2d);
        updatePoseEstimatorWithVisonBotPose();
    }

    public void updatePoseEstimatorWithVisonBotPose(){
        PoseEstimate visionPose = vision.getRobotPose();
        if (visionPose == null) return;
        poseVibPublisher.set(visionPose.pose);

        if(visionPose.pose.getX() == 0.0){
            return;
        }

        double poseDiff = poseEst.getEstimatedPosition().getTranslation().getDistance(visionPose.pose.getTranslation());

        double xyStds = 50.0;
        double degDev = 0.0;

        if(visionPose.tagCount >=2){
            xyStds = 0.5;
            degDev = 6;
        }else if(visionPose.tagSpan > 0.8 && poseDiff < 0.5){
            xyStds = 1.0;
            degDev = 12;
        }else if (visionPose.tagSpan > 0.1 && poseDiff < 0.3){
            xyStds = 2.0;
            degDev = 30;
        }
        
        poseEst.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degDev)));
        poseEst.addVisionMeasurement(visionPose.pose, visionPose.timestampSeconds);


    }

    public Pose2d getPose2d() {
        return pose;
    }

    public void resetPose(Pose2d val) {
        poseEst.resetPose(val);
        pose = val;
    }

    private boolean getAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
}
