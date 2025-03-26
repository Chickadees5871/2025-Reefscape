package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lift extends SubsystemBase {
    private SparkMax liftMotorLeft;
    private SparkMax liftMotorRight;
    private PIDController liftController;

    private double currentLiftPower = 0.0;
    private double target = -999.0;

    // I could make Intake a sub-subsystem of this but that seems unnessasary
    public Lift(){
        liftController = new PIDController(0.005, 0.00, 0.00);
        liftMotorLeft = new SparkMax(Constants.LiftConstants.liftMotorLeft, MotorType.kBrushless);
        liftMotorRight = new SparkMax(Constants.LiftConstants.liftMotorRight, MotorType.kBrushless);
    }

    public void tick(OperatorInterface oi){
        double lMotorPos = liftMotorLeft.getEncoder().getPosition();
        double rMotorPos = liftMotorLeft.getEncoder().getPosition();

        System.out.println("RMOTOR " + rMotorPos + " | LMOTOR " + lMotorPos);

        if(oi.gunnerController.getAButton()){
            target = Constants.LiftConstants.alegePreset;
        }else if(oi.gunnerController.getBButton()){
            target = Constants.LiftConstants.coralPreset;
        }

        if(target != -999.0){
            /*if(Math.abs(rMotorPos - target) < 0.005){
                target = -999.0;
                currentLiftPower = 0.0;
            }else{
                if(rMotorPos < target){
                    currentLiftPower = (rMotorPos/target) * 0.5;
                }else{
                    currentLiftPower = -(rMotorPos / target) * 0.5;
                }
            }*/
            currentLiftPower = liftController.calculate(lMotorPos, target);
        }

        // Be super careful when using manual control 
        double inputPower = oi.gunnerController.getLeftY();
        if(target != -999.0 && inputPower != 0.0){
            // Overrides preset movement
            target = -999.0;
        }
        currentLiftPower = inputPower != 0 ? inputPower / 10.0: currentLiftPower;
        
        // One of these needs to be negative idk which one
        liftMotorLeft.set(-currentLiftPower);
        liftMotorRight.set(currentLiftPower);
    }
}
