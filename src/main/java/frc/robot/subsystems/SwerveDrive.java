package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.Kinematics;

/** The subsystem that handles swerve drive. */
public class SwerveDrive extends SubsystemBase {

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    /** Initializes a new SwerveDrive subsystem object. */
    public SwerveDrive() {
        this.frontLeft = new SwerveModule(5, false,
                6, false, 50,
                SwerveConstants.Dimensions.magOffsetFL);
        this.frontRight = new SwerveModule(8, false,
                7, false, 53,
                SwerveConstants.Dimensions.magOffsetFR);
        this.backLeft = new SwerveModule(4, false,
                3, false, 52,
                SwerveConstants.Dimensions.magOffsetBL);
        this.backRight = new SwerveModule(1, false,
                2, false, 51,
                SwerveConstants.Dimensions.magOffsetBR);
    }

    /** Stops all motors. */
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    public void setDirection(
            double xSpeed, double ySpeed,
            double turningSpeed) {
        xSpeed *= JoystickConstants.xySpeedMultiplier;
        ySpeed *= JoystickConstants.xySpeedMultiplier;
        turningSpeed *= JoystickConstants.turningSpeedMultiplier;

        setSpeed(new ChassisSpeeds(xSpeed, ySpeed, turningSpeed));
    }

    public void setSpeed(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = Kinematics.driveKinematics.toSwerveModuleStates(speeds);
        this.setStates(moduleStates);
    }

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        frontLeft.periodic();
        frontRight.periodic();
        backLeft.periodic();
        backRight.periodic();
    }

    /**
     * Sets the swerve modules to a new state.
     *
     * @param desiredStates The new desired states
     * @param log           If the Swerve Modules should log there target state and
     *                      current state
     */
    private void setStates(SwerveModuleState[] desiredStates) {
        // Normalize the speed so we don attempt to overpower motors
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, SwerveConstants.Kinematics.drivePhysicalMaxSpeed);

        frontLeft.setState(desiredStates[0]);
        frontRight.setState(desiredStates[1]);
        backLeft.setState(desiredStates[2]);
        backRight.setState(desiredStates[3]);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Kinematics.driveKinematics.toChassisSpeeds(
                new SwerveModuleState[] {
                        frontLeft.getState(), frontRight.getState(),
                        backLeft.getState(), backRight.getState()
                });
    }
}