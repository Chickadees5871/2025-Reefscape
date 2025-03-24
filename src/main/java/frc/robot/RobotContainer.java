// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {
  public SwerveDrive swerveDrive;
  public OperatorInterface oi;
  public final DriveCommand driveCommand;

  public RobotContainer() {
    // Initialize the subsystems
    swerveDrive = new SwerveDrive();
    oi = new OperatorInterface();
    driveCommand = new DriveCommand(oi, swerveDrive);

    // Sets Keybinds
    configureBindings();
  }

  private void configureBindings() {

    Trigger gyroReset = new JoystickButton(oi.driveController, XboxController.Button.kLeftBumper.value);
    // When gyroreset pressed reset the swerve drive gyros
    gyroReset.onTrue(new InstantCommand(() -> {
      swerveDrive.resetGyro();
    }));

  }

  // Auto Code
  public Command getAutonomousCommand() {
    return null;
  }
}