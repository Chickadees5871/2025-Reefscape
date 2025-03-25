package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private SwerveModule[] modules;
    private SwerveModulePosition[] positions;
    private SwerveModuleState[] states;
    private SwerveDriveKinematics kinematics;
    private PIDController controller;
    private double setpoint = 0;
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
        kinematics = new SwerveDriveKinematics(
                new Translation2d(15 * 2.54 / 100, 15 * 2.54 / 100),
                new Translation2d(15 * 2.54 / 100, -15 * 2.54 / 100),
                new Translation2d(-15 * 2.54 / 100, 15 * 2.54 / 100),
                new Translation2d(-15 * 2.54 / 100, -15 * 2.54 / 100));
        controller = new PIDController(.01, 0, 0);

        gyro = new Pigeon2(41);
       
        // Init pose
        poseEst =
        new SwerveDrivePoseEstimator(
            kinematics,
            gyro.getRotation2d(), 
            positions,
            new Pose2d());
    }

    private void updateModules(){
        for(int i = 0; i < 4; i++){
            positions[i] = new SwerveModulePosition(modules[i].getEncoderPosition(), gyro.getRotation2d());
            states[i] = modules[i].currentState;
        }
    }

    @Override
    public void periodic(){
        updateModules();
        // Update the pose
        pose = poseEst.update(gyro.getRotation2d(), positions);
    }

    public void accept(ChassisSpeeds fieldCentricChassisSpeeds) {
        var chassis = ChassisSpeeds.fromFieldRelativeSpeeds(fieldCentricChassisSpeeds,
                Rotation2d.fromDegrees(-gyro.getYaw().getValueAsDouble() + 180));
        double rotation = -gyro.getYaw().getValueAsDouble();

        if (chassis.vxMetersPerSecond != 0 && chassis.vyMetersPerSecond != 0
                && chassis.omegaRadiansPerSecond == 0) {
            chassis.omegaRadiansPerSecond = -controller.calculate(rotation, setpoint);
        } else {
            setpoint = rotation;
        }
        var states = kinematics.toSwerveModuleStates(chassis);
        for (int i = 0; i < 4; i++) {
            modules[i].acceptMotion(states[i]);
        }

        // SmartDashboard.putNumberArray("swerve_target", fieldCentricChassisSpeeds);

    }

    public void resetGyro() {
        gyro.reset();
    }

    public Pose2d getPose2d(){
        return pose;
    }

    public void resetPose(Pose2d val){
        pose = val;
    }

    public ChassisSpeeds getChassisSpeeds(){
        return kinematics.toChassisSpeeds(states);
    }


}