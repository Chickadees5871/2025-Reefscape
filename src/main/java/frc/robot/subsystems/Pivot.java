package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
    private Intake intake;

    private SparkMax pivotMotor;

    private PIDController pivotController;

    private double pivotPower = 0.0;

    private final double kFF = 0.45;
    private final double kFFCoral = 0.85;
    
    public Pivot(Intake intake){
        this.intake = intake;
        pivotController = new PIDController(0.55, 0, 0.00);
        pivotController.setTolerance(0.05);
        
        pivotMotor = new SparkMax(Constants.LiftConstants.pivotMotor, MotorType.kBrushless); 
    }

    public void periodic(){
        SmartDashboard.putNumber("Pivot Postion", pivotMotor.getEncoder().getPosition());
        pivotPower = clamp(pivotController.calculate(pivotMotor.getEncoder().getPosition()), -0.5, 0.5);
        double ff = Math.abs(Math.sin((pivotMotor.getEncoder().getPosition() / 5) * Math.PI)) * (intake.hasCoral() ? kFFCoral : kFF);
        if(pivotMotor.getEncoder().getPosition() > 0.0 && pivotPower +  ff > 0){
            pivotMotor.set(0.0);
            return;
        }
        if(pivotMotor.getEncoder().getPosition() < -0.28 && pivotPower + ff < 0){
            pivotMotor.set(0.0);
            return;
        }
        pivotPower += ff;
        pivotPower = clamp(pivotPower, -1.0, 1.0);
        SmartDashboard.putNumber("TOTAL OUTPUT FOR PIVOT", pivotPower);
        SmartDashboard.putNumber("Pivot Setpoint", pivotController.getSetpoint());
        pivotMotor.set(pivotPower);
    }

    public void setSetpoint(double pos){
        pivotController.setSetpoint(pos);
    }

    public boolean atSetpoint() {
        return pivotController.atSetpoint();
    }

    public Command pivotToPoint(double setpoint) {
        return new InstantCommand(() -> this.setSetpoint(setpoint)).repeatedly().until(this::atSetpoint);
    }
    

    public double clamp(double d, double min, double max){
        if(d > max) return max;
        if(d < min) return min;
        return d;
    }
}
