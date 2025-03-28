// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.OperatorInterface;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {
    public final DriveCommand driveCommand;

    public SwerveDrive swerveDrive;
    public Intake intake;
    public Lift lift;
    public Pivot pivot;
    public OperatorInterface oi;

    private boolean atL1 = false;


    public RobotContainer() {
        // Initialize the subsystems
        swerveDrive = new SwerveDrive();
        intake = new Intake();
        lift = new Lift();
        pivot = new Pivot(intake);
        // autoSystem = new AutoSystem(swerveDrive);
        oi = new OperatorInterface();

        driveCommand = new DriveCommand(oi, swerveDrive);

        lift.setHomeState();

        // Sets Keybinds
        configureBindings();
    }

    private void configureBindings() {
        Trigger gyroReset = new JoystickButton(oi.driveController, XboxController.Button.kStart.value);
        Trigger rest = new JoystickButton(oi.gunnerController, XboxController.Button.kX.value);
        Trigger level1 = new JoystickButton(oi.gunnerController, XboxController.Button.kA.value);
        Trigger level2 = new JoystickButton(oi.gunnerController, XboxController.Button.kB.value);
        Trigger intakePos = new JoystickButton(oi.gunnerController, XboxController.Button.kY.value);
        Trigger coralIn = new Trigger(() -> oi.gunnerController.getRightTriggerAxis() > 0.1);
        Trigger coralOut = new Trigger(() -> oi.gunnerController.getLeftTriggerAxis() > 0.1);
        Trigger alegaIn = new JoystickButton(oi.gunnerController, XboxController.Button.kRightBumper.value);
        Trigger alegaOut = new JoystickButton(oi.gunnerController, XboxController.Button.kLeftBumper.value);


        // Coral 
        Trigger algea2 = new Trigger(() -> oi.gunnerController.getPOV() == 0);
        Trigger alega1 = new Trigger(() -> oi.gunnerController.getPOV() == 180);
        
        // When gyroreset pressed reset the swerve drive gyros
        gyroReset.onTrue(new InstantCommand(() -> {
            swerveDrive.resetGyro();
        }));

        rest.onTrue(lift.moveToSetpoint(Constants.LiftConstants.level0)
                .andThen(pivot.pivotToPoint(Constants.LiftConstants.pivotRest)));

        coralIn.whileTrue(intake.intakeCoralIn());
        coralOut.whileTrue(intake.intakeCoralOut(atL1));

        alegaIn.whileTrue(intake.intakeAlgea());
        alegaOut.whileTrue(intake.outtakeAlega());

        level1.onTrue(pivot.pivotToPoint(Constants.LiftConstants.pivotScore).onlyIf(intake::notHasAlgea).withTimeout(1)
                .andThen(lift.moveToSetpoint(Constants.LiftConstants.level1)).alongWith(new InstantCommand(() -> {atL1 = true;})));
        level2.onTrue(pivot.pivotToPoint(Constants.LiftConstants.pivotScore).onlyIf(intake::notHasAlgea).withTimeout(1)
                .andThen(lift.moveToSetpoint(Constants.LiftConstants.level2)).alongWith(new InstantCommand(() -> {atL1 = false;})));
        intakePos.onTrue(lift.moveToSetpoint(Constants.LiftConstants.liftIntake)
                .andThen(pivot.pivotToPoint(Constants.LiftConstants.pivotIntake)).alongWith(new InstantCommand(() -> {atL1 = false;})));

        alega1.onTrue(pivot.pivotToPoint(Constants.LiftConstants.pivotIntake).andThen(lift.moveToSetpoint(Constants.LiftConstants.level1_5)));
        algea2.onTrue(pivot.pivotToPoint(Constants.LiftConstants.pivotIntake).andThen(lift.moveToSetpoint(Constants.LiftConstants.level2_5)));

    }

    // Auto Code
    public Command getAutonomousCommand() {
        return new InstantCommand(() -> swerveDrive.resetGyro())
                .andThen(new InstantCommand(() -> swerveDrive.accept(new ChassisSpeeds(-2.0, 0.0, 0.0))).repeatedly())
                .withTimeout(0.95)
                .andThen(new InstantCommand(() -> swerveDrive.accept(new ChassisSpeeds(0.0, 0.0, 0.0))));
        // return new PathPlannerAuto("2025Auto");
    }
}