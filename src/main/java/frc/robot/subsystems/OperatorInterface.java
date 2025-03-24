package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OperatorInterface extends SubsystemBase {
    public XboxController driveController;
    public XboxController gunnerController;

    SlewRateLimiter xLimiter = new SlewRateLimiter(10);
    SlewRateLimiter yLimiter = new SlewRateLimiter(10);
    SlewRateLimiter zLimiter = new SlewRateLimiter(10);

    public OperatorInterface() {
        driveController = new XboxController(0);
        gunnerController = new XboxController(1);
    }

    public ChassisSpeeds getChassisSpeeds() {
        double ySpeed = deadzone(yLimiter.calculate(driveController.getLeftY()), .2);
        double xSpeed = deadzone(xLimiter.calculate(driveController.getLeftX()), .2);
        double zSpeed = deadzone(zLimiter.calculate(driveController.getRightX()), .2);

        SmartDashboard.putString("Chasis Speed", xSpeed + ", " + ySpeed + ", " + zLimiter);

        return new ChassisSpeeds(
                -Math.pow(ySpeed, 3),
                -Math.pow(xSpeed, 3),

                Math.pow(zSpeed, 3));
    }

    public static double deadzone(double val, double threshold) {
        return Math.abs(val) < threshold ? 0 : val;
    }
}