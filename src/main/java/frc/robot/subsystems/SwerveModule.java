package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Configs;

public class SwerveModule {
    //private SparkPIDController azimController;
    private SparkClosedLoopController driveController;
    private SparkMax driveMotor;
    private SparkMax azimMotor;
    private CANcoder cancoder;
    double maxSpeed = 3;
    boolean log = false;
    //private RelativeEncoder encoder;
    private DirectSwerveCommand swerveCommand;
    PIDController anglePIDcontroller = new PIDController(0.005, 0.00, 0.00);

    RelativeEncoder drive_Encoder;

    SwerveModuleState currentState;


    private class DirectSwerveCommand {
        double angle;
        double drive;

        public DirectSwerveCommand(double angle, double drive) {
            this.angle = angle;
            this.drive = drive;

           // driveMotor.setIdleMode(IdleMode.kCoast);
        }
    }
public SwerveModule(SparkMax driveMotor, SparkMax azimuthMotor, CANcoder cancoder, double offset, boolean log) {
    this(driveMotor, azimuthMotor, cancoder, offset);
    this.log = log;
}
    public SwerveModule(SparkMax driveMotor, SparkMax azimuthMotor, CANcoder cancoder, double offset) {

        this.driveMotor = driveMotor;
        this.driveMotor.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        this.azimMotor = azimuthMotor;
        this.azimMotor.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        //azimController = azimMotor.getPIDController();
        driveController = driveMotor.getClosedLoopController();
        this.cancoder = cancoder;

        drive_Encoder = driveMotor.getEncoder();

        // set offset to encoder
        MagnetSensorConfigs conf = new MagnetSensorConfigs();

        anglePIDcontroller.enableContinuousInput(-180, 180);

        //azimMotor.getEncoder().setPositionConversionFactor(Math.PI * 2 / 13.3714);

        //azimMotor.getEncoder().setPosition(-cancoder.getPosition().getValue() * Math.PI * 2 + Math.PI / 2);
        azimMotor.setInverted(false);
        //azimController.setP(0.4);
        //encoder = azimuthMotor.getEncoder();
        swerveCommand = new DirectSwerveCommand(0, 0);
    }


    void update() {

        SwerveModuleState temp = new SwerveModuleState();

        temp.speedMetersPerSecond = drive_Encoder.getVelocity();

        temp.angle = Rotation2d.fromRotations( cancoder.getAbsolutePosition().getValueAsDouble() );

        currentState = temp;
    }



    int counter = 0;

    public void acceptMotion(SwerveModuleState state) {
        //double theta = state.angle.getRadians();

        update();

        state = SwerveModuleState.optimize(state, currentState.angle);

        double driveVoltage = state.speedMetersPerSecond / maxSpeed;
        //calculateOptimalSetpoint(theta, driveVoltage, cancoder.getAbsolutePosition().getValueAsDouble());
        var PIDval = anglePIDcontroller.calculate(cancoder.getAbsolutePosition().getValueAsDouble()*360, state.angle.getDegrees());
        azimMotor.set(-MathUtil.clamp(PIDval, -0.5, 0.5));
        //setReferenceAngle(swerveCommand.angle);
        driveMotor.setVoltage(driveVoltage * 60 /*swerveCommand.drive * 60 */);
        if( counter > 25 && log) {
          System.out.println("target: " + state.angle.getDegrees() + " actual: " + cancoder.getAbsolutePosition().getValueAsDouble()*360 + " state: " + state.angle.getRadians() + " PIDvalue: " + PIDval);
          counter = 0;
        }
        counter++;
    }

    public void setReferenceAngle(double referenceAngleRadians) {
        //double currentAngleRadians = encoder.getPosition();
        double currentAngleRadians = cancoder.getPosition().getValueAsDouble();


        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Neo's encoder can go above
        // that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        //azimController.setReference(adjustedReferenceAngleRadians, SparkMax.ControlType.kPosition);
    }

    private void calculateOptimalSetpoint(double steerAngle, double driveVoltage, double currentAngle) {

        steerAngle %= (2.0 * Math.PI);
        if (steerAngle < 0.0) {
            steerAngle += 2.0 * Math.PI;
        }

        double difference = steerAngle - currentAngle;
        // Change the target angle so the difference is in the range [-pi, pi) instead
        // of [0, 2pi)
        if (difference >= Math.PI) {
            steerAngle -= 2.0 * Math.PI;
        } else if (difference < -Math.PI) {
            steerAngle += 2.0 * Math.PI;
        }
        difference = steerAngle - currentAngle; // Recalculate difference

        // If the difference is greater than 90 deg or less than -90 deg the drive can
        // be inverted so the total
        // movement of the module is less than 90 deg
        if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
            // Only need to add 180 deg here because the target angle will be put back into
            // the range [0, 2pi)
            steerAngle += Math.PI;
            driveVoltage *= -1.0;
        }

        // Put the target angle back into the range [0, 2pi)
        steerAngle %= (2.0 * Math.PI);
        if (steerAngle < 0.0) {

            steerAngle += 2.0 * Math.PI;
        }
        swerveCommand.angle = steerAngle;
        swerveCommand.drive = driveVoltage;
        // return swerveCommand;

    }
}