// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.subsystems.SwerveDrive;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SwerveDrive m_SwerveDrive;
  private JoystickDriveCommand m_SwerveDriveCommand;

  // The driver's controller
  XboxController m_driveController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_SwerveDrive = new SwerveDrive();

    // Configure default commands
    m_SwerveDriveCommand = new JoystickDriveCommand(m_SwerveDrive,
                () -> {
                    return m_driveController.getLeftY()
                            / (m_driveController.getXButton() ? 0.25 : 1.0);
                }, () -> {
                    return m_driveController.getLeftX()
                            / (m_driveController.getXButton() ? 0.25 : 1.0);
                }, () -> {
                    return Util.applyLinearDeadZone(
                            JoystickConstants.joystickDeadZone, m_driveController.getRightX());
                });
  }

  public void initTeleop() {
        m_SwerveDrive.setDefaultCommand(m_SwerveDriveCommand);
  }
}

//   /**
//    * Use this method to define your button->command mappings. Buttons can be
//    * created by
//    * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
//    * subclasses ({@link
//    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
//    * passing it to a
//    * {@link JoystickButton}.
//    */
//   private void configureButtonBindings() {
//   }

//   /**
//    * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */
//   public Command getAutonomousCommand() {
//     // Create config for trajectory
//     TrajectoryConfig config = new TrajectoryConfig(
//         AutoConstants.kMaxSpeedMetersPerSecond,
//         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//         // Add kinematics to ensure max speed is actually obeyed
//         .setKinematics(DriveConstants.kDriveKinematics);

//     // An example trajectory to follow. All units in meters.
//     Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
//         // Start at the origin facing the +X direction
//         new Pose2d(0, 0, new Rotation2d(0)),
//         // Pass through these two interior waypoints, making an 's' curve path
//         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
//         // End 3 meters straight ahead of where we started, facing forward
//         new Pose2d(3, 0, new Rotation2d(0)),
//         config);

//     var thetaController = new ProfiledPIDController(
//         AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);

//     SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//         exampleTrajectory,
//         m_robotDrive::getPose, // Functional interface to feed supplier
//         DriveConstants.kDriveKinematics,

//         // Position controllers
//         new PIDController(AutoConstants.kPXController, 0, 0),
//         new PIDController(AutoConstants.kPYController, 0, 0),
//         thetaController,
//         m_robotDrive::setModuleStates,
//         m_robotDrive);

//     // Reset odometry to the starting pose of the trajectory.
//     m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

//     // Run path following command, then stop at the end.
//     return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
//   }
// }
