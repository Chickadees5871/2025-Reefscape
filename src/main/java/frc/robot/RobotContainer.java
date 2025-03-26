// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.subsystems.AutoSystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {
    public final DriveCommand driveCommand;
    public final IntakeCommand intakeCommand;
    public final LiftCommand liftCommand;

    public SwerveDrive swerveDrive;
    public Intake intake;
    public Lift lift;
    public OperatorInterface oi;
    public AutoSystem autoSystem;

    public RobotContainer() {
        // Initialize the subsystems
        swerveDrive = new SwerveDrive();
        intake = new Intake();
        lift = new Lift();
        //autoSystem = new AutoSystem(swerveDrive);
        oi = new OperatorInterface();

        driveCommand = new DriveCommand(oi, swerveDrive);
        intakeCommand = new IntakeCommand(oi, intake);
        liftCommand = new LiftCommand(oi, lift);

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
        //return new PathPlannerAuto("2025Auto");
    }
}