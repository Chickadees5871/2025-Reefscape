package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lift extends SubsystemBase {
    private SparkMax liftLeftMotor;
    private SparkMax liftRightMotor;

    private PIDController liftController;

    // I could make Intake a sub-subsystem of this but that seems unnessasary
    public Lift(){
        liftController = new PIDController(0.005, 0.00, 0.00);
        liftLeftMotor = new SparkMax(Constants.LiftConstants.liftMotorLeft, MotorType.kBrushless);
        liftRightMotor = new SparkMax(Constants.LiftConstants.liftMotorRight, MotorType.kBrushless);
    }

    public void periodic(){
        double output = liftController.calculate(liftLeftMotor.getEncoder().getPosition());

        liftLeftMotor.set(output);
        liftRightMotor.set(-output);
    }

    public void setSetpoint(double setpoint) {
        liftController.setSetpoint(setpoint);
    }

    public boolean atSetpoint() {
        return liftController.atSetpoint();
    }

    public Command moveToSetpoint(double setpoint) {
        return new InstantCommand(() -> this.setSetpoint(setpoint)).repeatedly().until(this::atSetpoint);
    }
}
