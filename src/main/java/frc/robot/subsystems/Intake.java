package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private SparkMax intakeMotor1;
    private SparkMax intakeMotor2;
    private SparkMax coralMotor;
    private SparkMax pivotMotor;
    
    public Intake(){
        intakeMotor1 = new SparkMax(Constants.LiftConstants.algeIntake1CanId, MotorType.kBrushless);
        intakeMotor2 = new SparkMax(Constants.LiftConstants.algeIntake2CanId, MotorType.kBrushless);
        coralMotor = new SparkMax(Constants.LiftConstants.coralMotorCanId, MotorType.kBrushless);
        pivotMotor = new SparkMax(Constants.LiftConstants.pivotMotor, MotorType.kBrushless); 
    }

    public void tick(OperatorInterface oi){
        // Fat ball intake
        double ballPower = oi.gunnerController.getRightBumperButton() ? 1.0 : (oi.gunnerController.getLeftBumperButton() ? -1.0 : 0.0);
        

        intakeMotor1.set(ballPower);
        intakeMotor2.set(ballPower);
        
        // Retrive states for the rod shaft shooter
        double rodPowerPositive = oi.gunnerController.getRightTriggerAxis();
        double rodPowerNegative = -oi.gunnerController.getLeftTriggerAxis();

        double rodPower = rodPowerPositive - rodPowerNegative;

        coralMotor.set(rodPower);
    }
}
