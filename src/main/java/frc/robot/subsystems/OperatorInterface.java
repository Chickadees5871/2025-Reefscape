package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OperatorInterface extends SubsystemBase {
    public XboxController driveController;
    public XboxController gunnerController;

    private PIDController pidController;
    private Vision vision;

    public OperatorInterface(Vision vision) {
        this.vision = vision;
        pidController = new PIDController(0.005, 0, 0);
        driveController = new XboxController(0);
        gunnerController = new XboxController(1);
    }

    public ChassisSpeeds getChassisSpeeds() {
        double ySpeed = deadzone(driveController.getLeftY(), .075);
        double xSpeed = deadzone(driveController.getLeftX(), .075);
        double zSpeed;

        if (driveController.getRightBumperButtonPressed()) {
            zSpeed = pidController.calculate(vision.getTagYaw(), 0);
        } else {
            zSpeed = deadzone(driveController.getRightX(), .075);
        }

        SmartDashboard.putString("Chasis Speed", xSpeed + ", " + ySpeed + ", " + zSpeed);

        return new ChassisSpeeds(-ySpeed * Constants.DriveConstants.kMaxSpeedMetersPerSecond,
                -xSpeed * Constants.DriveConstants.kMaxSpeedMetersPerSecond,
                zSpeed * Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    }

    public static double deadzone(double val, double threshold) {
        return Math.abs(val) < threshold ? 0 : val;
    }
}