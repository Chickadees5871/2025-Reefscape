package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Configs;

public class SwerveModule {
    private SparkMax driveMotor;
    private SparkMax azimMotor;
    private CANcoder cancoder;

    private double maxSpeed = 3;
    private boolean log = false;
    private PIDController anglePIDcontroller = new PIDController(0.005, 0.00, 0.00);
    private SwerveModuleState currentState;

    public SwerveModule(SparkMax driveMotor, SparkMax azimuthMotor, CANcoder cancoder, double offset) {
        this.driveMotor = driveMotor;
        this.azimMotor = azimuthMotor;
        this.cancoder = cancoder;

        this.driveMotor.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        this.azimMotor.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);

        anglePIDcontroller.enableContinuousInput(-180, 180);
    }

    public SwerveModule(SparkMax driveMotor, SparkMax azimuthMotor, CANcoder cancoder, double offset, boolean log) {
        this(driveMotor, azimuthMotor, cancoder, offset);
        this.log = log;
    }

    private void UpdateCurrentState() {
        SwerveModuleState temp = new SwerveModuleState();

        temp.speedMetersPerSecond = driveMotor.getEncoder().getVelocity();
        temp.angle = Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble());

        currentState = temp;
    }

    public void acceptMotion(SwerveModuleState state) {
        UpdateCurrentState();
        /*state.optimize(currentState.angle);

        var PIDval = anglePIDcontroller.calculate(cancoder.getAbsolutePosition().getValueAsDouble() * 360,
                state.angle.getDegrees());
        azimMotor.set(anglePIDcontroller.calculate(cancoder.getAbsolutePosition().getValueAsDouble() * 360,
                state.angle.getDegrees()));
        driveMotor.set(state.speedMetersPerSecond / maxSpeed);*/
        state = SwerveModuleState.optimize(state, currentState.angle);

        double driveVoltage = state.speedMetersPerSecond / maxSpeed;
        //calculateOptimalSetpoint(theta, driveVoltage, cancoder.getAbsolutePosition().getValueAsDouble());
        var PIDval = anglePIDcontroller.calculate(cancoder.getAbsolutePosition().getValueAsDouble()*360, state.angle.getDegrees());
        azimMotor.set(-MathUtil.clamp(PIDval, -0.5, 0.5));
        //setReferenceAngle(swerveCommand.angle);
        driveMotor.setVoltage(driveVoltage * 60 /*swerveCommand.drive * 60 */);
        if (log)
            System.out.println("target: " + state.angle.getDegrees() + " actual: "
                    + cancoder.getAbsolutePosition().getValueAsDouble() * 360 + " state: " + state.angle.getRadians()
                    + " PIDvalue: " + PIDval);
    }

    public SwerveModuleState getState() {
        return this.currentState;
    }

    public double getRotation() {
        return cancoder.getPosition().getValueAsDouble();
    }
}