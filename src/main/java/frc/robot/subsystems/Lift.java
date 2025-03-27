package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        liftController = new PIDController(0.15, 0.00, 0.00);
        liftController.setTolerance(1.0);
        liftLeftMotor = new SparkMax(Constants.LiftConstants.liftMotorLeft, MotorType.kBrushless);
        liftRightMotor = new SparkMax(Constants.LiftConstants.liftMotorRight, MotorType.kBrushless);
    }

    public void periodic(){
        SmartDashboard.putNumber("ELEVATOR POS", liftLeftMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Lift Setpoint", liftController.getSetpoint());

        double output = clamp(liftController.calculate(liftLeftMotor.getEncoder().getPosition()), -0.5, 0.5);

        if(liftController.getSetpoint() > 80){
            setSetpoint(80);
        }
        if(liftLeftMotor.getEncoder().getPosition() > 80){
            output = 0;
        }

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

    public Command setHomeState(){
        return new InstantCommand(() -> this.setSetpoint(Constants.LiftConstants.level0)).repeatedly().until(this::atSetpoint);
    }   

    public double clamp(double d, double min, double max){
        if(d > max) return max;
        if(d < min) return min;
        return d;
    }
}
