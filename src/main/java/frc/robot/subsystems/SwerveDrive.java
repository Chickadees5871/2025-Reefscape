package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private SwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    private AHRS navX;
    private PIDController controller;
    private double setpoint = 0;

    public SwerveDrive() {
        modules = new SwerveModule[4];
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
        navX = new AHRS(NavXComType.kMXP_SPI);
        controller = new PIDController(.01, 0, 0);
    }

    public void accept(ChassisSpeeds fieldCentricChassisSpeeds) {
        var chassis = ChassisSpeeds.fromFieldRelativeSpeeds(fieldCentricChassisSpeeds,
                Rotation2d.fromDegrees(-navX.getYaw() + 180));
        double gyro = -navX.getYaw();

        if (chassis.vxMetersPerSecond != 0 && chassis.vyMetersPerSecond != 0
                && chassis.omegaRadiansPerSecond == 0) {
            chassis.omegaRadiansPerSecond = -controller.calculate(gyro, setpoint);
        } else {
            setpoint = gyro;
        }
        var states = kinematics.toSwerveModuleStates(chassis);
        for (int i = 0; i < 4; i++) {
            modules[i].acceptMotion(states[i]);
        }

        // SmartDashboard.putNumberArray("swerve_target", fieldCentricChassisSpeeds);

    }

    public void resetGyro() {
        navX.reset();
    }
}