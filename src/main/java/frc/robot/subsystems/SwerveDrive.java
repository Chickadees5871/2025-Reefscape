package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {
    private SwerveModule[] modules;
    private SwerveModulePosition[] positions;
    private SwerveModuleState[] states;

    private Pigeon2 gyro;
    private final SwerveDrivePoseEstimator poseEst;

    private Pose2d pose;

    public SwerveDrive() {
        modules = new SwerveModule[4];
        positions = new SwerveModulePosition[4];
        states = new SwerveModuleState[4];

        // fl
        modules[0] = new SwerveModule(
                new SparkMax(5, MotorType.kBrushless),
                new SparkMax(6, MotorType.kBrushless),
                new CANcoder(50), 0.124756);
        // fr
        modules[1] = new SwerveModule(
                new SparkMax(8, MotorType.kBrushless),
                new SparkMax(7, MotorType.kBrushless),
                new CANcoder(53), -0.154297);
        // bl
        modules[2] = new SwerveModule(
                new SparkMax(4, MotorType.kBrushless),
                new SparkMax(3, MotorType.kBrushless),
                new CANcoder(52), 0.010498);
        // br
        modules[3] = new SwerveModule(
                new SparkMax(1, MotorType.kBrushless),
                new SparkMax(2, MotorType.kBrushless),
                new CANcoder(51), 0.409424);

        gyro = new Pigeon2(41);

        updateModules();

        // Init pose
        poseEst = new SwerveDrivePoseEstimator(
                Constants.DriveConstants.kDriveKinematics,
                gyro.getRotation2d(),
                positions,
                new Pose2d());
    }

    private void updateModules() {
        for (int i = 0; i < 4; i++) {
            positions[i] = new SwerveModulePosition(modules[i].getRotation(), gyro.getRotation2d());
            states[i] = modules[i].getState();
        }
    }

    @Override
    public void periodic() {
        updateModules();
        SmartDashboard.putNumber("Rotation", gyro.getRotation2d().getDegrees());
        pose = poseEst.update(gyro.getRotation2d(), positions);
    }

    public void accept(ChassisSpeeds fieldCentricChassisSpeeds) {
        ChassisSpeeds chassis = ChassisSpeeds.fromFieldRelativeSpeeds(fieldCentricChassisSpeeds,
                this.getRotation2d());

        SwerveModuleState[] states = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassis);
        for (int i = 0; i < 4; i++) {
            modules[i].acceptMotion(states[i]);
        }
    }

    public void resetGyro() {
        gyro.reset();
    }

    public Pose2d getPose2d() {
        return pose;
    }

    public void resetPose(Pose2d val) {
        pose = val;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble() + 180);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(states);
    }
}