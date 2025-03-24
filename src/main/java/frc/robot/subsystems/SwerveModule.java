package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;
import frc.robot.Constants.SwerveConstants;

/** An individual swerve module. */
public class SwerveModule {
        // Drive
        private final SparkMax driveController;

        // Turning
        private final SparkMax turningController;
        private final PIDController turningPid;

        // CAN Coder
        private final CANcoder canCoder;
        private final double canCoderOffset;

        private SwerveModuleState state;
        private int driveId;

        /**
         * Creates a new swerve module.
         *
         * @param driveId           CAN ID for the drive motor (Falcon)
         * @param isDriveReversed   If the drive motor is reversed
         * @param turningId         CAN ID for the turning motor (NEO)
         * @param isTurningReversed If the turning motor is reversed
         * @param coderId           CAN ID for the CAN coder within the swerve module
         * @param coderOffset       The offset to get the CAN coder to true zero (NEED
         *                          TO BE POSITIVE)
         */
        public SwerveModule(
                        int driveId,
                        boolean isDriveReversed,
                        int turningId,
                        boolean isTurningReversed,
                        int coderId,
                        double coderOffset) {
                driveController = new SparkMax(driveId, MotorType.kBrushless);
                this.driveId = driveId;

                turningController = new SparkMax(turningId, MotorType.kBrushless);
                turningController.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

                turningPid = new PIDController(
                                SwerveConstants.PID.ModuleAngle.kP,
                                SwerveConstants.PID.ModuleAngle.kI,
                                SwerveConstants.PID.ModuleAngle.kD);
                turningPid.enableContinuousInput(0.0, 1.0);
                turningPid.setTolerance(SwerveConstants.PID.ModuleAngle.kT);

                canCoder = new CANcoder(coderId);

                this.canCoderOffset = coderOffset;
        }

        /**
         * Sets the swerve module to a new state.
         *
         * @param state The new desired state
         * @param log   If the Swerve Module should log there target state and current
         *              state
         */
        public void setState(SwerveModuleState state) {
                this.state = SwerveModuleState.optimize(
                                state, Rotation2d.fromRadians(getTurningPosition() * 2 * Math.PI));
                this.state = state;
                driveController.set(this.state.speedMetersPerSecond / SwerveConstants.Kinematics.drivePhysicalMaxSpeed);
                turningController.set(turningPid.calculate(
                        getTurningPosition(), this.state.angle.getRadians() / (2 * Math.PI)));
                
                SmartDashboard.putNumber("" + this.driveId + " Target", this.state.angle.getRadians());
        }

        public SwerveModuleState getState() {
                if (state != null)
                        return state;
                return new SwerveModuleState();
        }

        /** Stops the swerve module. */
        public void stop() {
                driveController.set(0);
                turningController.set(0);
        }

        /**
         * Gets the turning motor position in radians.
         *
         * @return Turning motor position
         */
        public double getTurningPosition() {
                return (((canCoder.getPosition().getValueAsDouble() - canCoderOffset) % 1) + 1) % 1;
        }

        /**
         * Gets the turning motor position in radians in a human readable form.
         *
         * @return Turning motor position
         */
        public double getTurningPositionReadable() {
                return Math.abs((canCoder.getPosition().getValueAsDouble() - canCoderOffset) % 1)
                                * 2
                                * Math.PI;
        }

        public SwerveModulePosition getPosition() {
                return new SwerveModulePosition(
                                driveController.getEncoder().getPosition()
                                                * SwerveConstants.Kinematics.driveRotToMeters,
                                new Rotation2d(getTurningPosition() * 2 * Math.PI));
        }

        public void periodic() {
                SmartDashboard.putNumber("" + this.driveId + " Current", getTurningPosition());
        }
}