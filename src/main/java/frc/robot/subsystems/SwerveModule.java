package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Configs;
import frc.robot.Constants;

public class SwerveModule {
    private SparkMax driveMotor;
    private SparkMax azimMotor;
    private CANcoder cancoder;

    private PIDController anglePIDcontroller;
    private SwerveModuleState currentState;

    public SwerveModule(SparkMax driveMotor, SparkMax azimuthMotor, CANcoder cancoder, double offset) {
        this.driveMotor = driveMotor;
        this.azimMotor = azimuthMotor;
        this.cancoder = cancoder;

        this.driveMotor.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        this.azimMotor.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);

        this.anglePIDcontroller = new PIDController(Constants.PIDConstants.ModuleAngleConstants.kP, Constants.PIDConstants.ModuleAngleConstants.kI, Constants.PIDConstants.ModuleAngleConstants.kD);
        this.anglePIDcontroller.setTolerance(Constants.PIDConstants.ModuleAngleConstants.kT);

        anglePIDcontroller.enableContinuousInput(0.0, 1.0);
    }

    private void UpdateCurrentState() {
        SwerveModuleState temp = new SwerveModuleState();

        temp.speedMetersPerSecond = driveMotor.getEncoder().getVelocity();
        temp.angle = Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble());

        currentState = temp;
    }

    public void acceptMotion(SwerveModuleState state) {
        UpdateCurrentState();
        state.optimize(currentState.angle);

        azimMotor.set(-anglePIDcontroller.calculate(cancoder.getAbsolutePosition().getValueAsDouble(),
                state.angle.getDegrees()/360));
        driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    }

    public SwerveModuleState getState() {
        return this.currentState;
    }

    public double getRotation() {
        return cancoder.getPosition().getValueAsDouble();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getEncoder().getPosition(), Rotation2d.fromRotations(getRotation()));
    }
}