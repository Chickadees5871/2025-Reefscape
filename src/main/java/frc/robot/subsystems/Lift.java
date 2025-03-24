package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lift extends SubsystemBase{
    private SparkMax liftMotorLeft;
    private SparkMax liftMotorRight;

    private double currentLiftPower = 0.0;
    private double target = -999.0;

    // I could make Intake a sub-subsystem of this but that seems unnessasary
    public Lift(){
        liftMotorLeft = new SparkMax(Constants.LiftConstants.liftMotorLeft, MotorType.kBrushless);
        liftMotorRight = new SparkMax(Constants.LiftConstants.liftMotorRight, MotorType.kBrushless);
    }

    public void tick(OperatorInterface oi){
        double lMotorPos = liftMotorLeft.getAbsoluteEncoder().getPosition();
        double rMotorPos = liftMotorLeft.getAbsoluteEncoder().getPosition();

        if(oi.gunnerController.getAButton()){
            target = Constants.LiftConstants.alegePreset;
        }else if(oi.gunnerController.getBButton()){
            target = Constants.LiftConstants.coralPreset;
        }

        if(target != -999.0){
            if(Math.abs(rMotorPos - target) < 10.0){
                target = -999.0;
                currentLiftPower = 0.0;
            }else{
                if(rMotorPos < target){
                    currentLiftPower = (rMotorPos/target) * 1.0;
                }else{
                    currentLiftPower = -(target/rMotorPos) * 1.0;
                }
            }
        }

        // Be super careful when using manual control 
        double inputPower = oi.gunnerController.getLeftY();
        if(target != -999.0 && inputPower != 0.0){
            // Overrides preset movement
            target = -999.0;
        }
        currentLiftPower = inputPower != 0 ? inputPower : currentLiftPower;
        
        // One of these needs to be negative idk which one
        liftMotorLeft.set(currentLiftPower);
        liftMotorRight.set(-currentLiftPower);
    }
}
