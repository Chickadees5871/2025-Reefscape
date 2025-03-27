package frc.robot.subsystems;
import java.io.ObjectInputFilter.Config;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private SparkMax intakeMotor1;
    private SparkMax intakeMotor2;
    private SparkMax coralMotor;

    private double rodPower = 0.0;
    private double ballPower = 0.0;

    private boolean hasAlgea = false;
    
    public Intake(){
        intakeMotor1 = new SparkMax(Constants.LiftConstants.algeIntake1CanId, MotorType.kBrushless);
        intakeMotor2 = new SparkMax(Constants.LiftConstants.algeIntake2CanId, MotorType.kBrushless);
        coralMotor = new SparkMax(Constants.LiftConstants.coralMotorCanId, MotorType.kBrushless);
    }

    public void periodic(){
        if (hasAlgea && ballPower == 0.0) {
            intakeMotor1.set(0.05);
            intakeMotor2.set(-0.05);
        }
        else {
            intakeMotor1.set(ballPower);
            intakeMotor2.set(-ballPower);
        }
        
        coralMotor.set(rodPower);
    }

    private void intakeCoral(){
        rodPower = 0.5;
    }

    private void outtakeCoral(){
        rodPower = -0.5;
    }

    private void noCoral(){
        rodPower = 0.0;
    }

    private void intakeBall() {
        ballPower = 0.5;
        hasAlgea = true;
    }

    private void outtakeBall() {
        ballPower = -0.5;
        hasAlgea = false;
    }

    public void restBall(){
        ballPower = 0.0;
    }

    public Command intakeCoralIn(){
        return new InstantCommand(() -> this.intakeCoral()).repeatedly().finallyDo(() -> this.noCoral());
    }

    public Command intakeCoralOut(){
        return new InstantCommand(() -> this.outtakeCoral()).repeatedly().finallyDo(() -> this.noCoral());
    }

    public Command intakeAlgea(){
        return new InstantCommand(() -> this.intakeBall());
    }

    public Command restAlega(){
        return new InstantCommand(() -> this.restBall());
    }

    public Command outtakeAlega(){
        return new InstantCommand(() -> this.outtakeBall()).repeatedly().finallyDo(() -> this.restBall());
    }

    public boolean notHasAlgea() {
        return !hasAlgea;
    }
}
