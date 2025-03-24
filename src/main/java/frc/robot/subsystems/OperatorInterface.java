package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OperatorInterface extends SubsystemBase {
    public XboxController driveController;
    public XboxController gunnerController;

    public OperatorInterface() {
        driveController = new XboxController(0);
        gunnerController = new XboxController(1);
    }

    public ChassisSpeeds getChassisSpeeds() {
        double ySpeed = deadzone(driveController.getLeftY(), .075);
        double xSpeed = deadzone(driveController.getLeftX(), .075);
        double zSpeed = deadzone(driveController.getRightX(), .075);

        SmartDashboard.putString("Chasis Speed", xSpeed + ", " + ySpeed + ", " + zSpeed);

        return new ChassisSpeeds(
                -Math.pow(ySpeed, 3),
                -Math.pow(xSpeed, 3),

                Math.pow(zSpeed, 3));
    }

    public static double deadzone(double val, double threshold) {
        return Math.abs(val) < threshold ? 0 : val;
    }
}