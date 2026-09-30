package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;

  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final SparkClosedLoopController m_driveClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private final SparkMaxConfig m_driveConfig = new SparkMaxConfig();
  private final SparkMaxConfig m_turningConfig = new SparkMaxConfig();

  public SwerveModule(int driveMotorChannel, int turningMotorChannel) {
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getEncoder();

    m_driveClosedLoopController = m_driveMotor.getClosedLoopController();
    m_turningClosedLoopController = m_turningMotor.getClosedLoopController();

    // Drive Motor Configuration
    m_driveConfig.idleMode(IdleMode.kBrake);
    m_driveConfig.encoder
        .positionConversionFactor(2 * Math.PI * kWheelRadius / kEncoderResolution)
        .velocityConversionFactor(2 * Math.PI * kWheelRadius / (60.0 * kEncoderResolution));
    m_driveConfig.closedLoop
        .p(1)
        .i(0)
        .d(0);

    // Turning Motor Configuration
    m_turningConfig.idleMode(IdleMode.kBrake);
    m_turningConfig.encoder
        .positionConversionFactor(2 * Math.PI / kEncoderResolution)
        .velocityConversionFactor(2 * Math.PI / (60.0 * kEncoderResolution));
    m_turningConfig.closedLoop
        .p(1)
        .i(0)
        .d(0)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 2 * Math.PI);

    // Apply configs
    m_driveMotor.configure(m_driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningMotor.configure(m_turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // SmartDashboard setup
    SmartDashboard.putNumber("Drive P " + driveMotorChannel, 1.0);
    SmartDashboard.putNumber("Drive I " + driveMotorChannel, 0.0);
    SmartDashboard.putNumber("Drive D " + driveMotorChannel, 0.0);
    SmartDashboard.putNumber("Turn P " + turningMotorChannel, 1.0);
    SmartDashboard.putNumber("Turn I " + turningMotorChannel, 0.0);
    SmartDashboard.putNumber("Turn D " + turningMotorChannel, 0.0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_turningEncoder.getPosition());

    // Call the non-deprecated instance method directly
    desiredState.optimize(encoderRotation);

    m_driveClosedLoopController.setSetpoint(desiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    m_turningClosedLoopController.setSetpoint(desiredState.angle.getRadians(), SparkMax.ControlType.kPosition);
  }

  public void updatePIDFromDashboard() {
    double driveP = SmartDashboard.getNumber("Drive P " + m_driveMotor.getDeviceId(), 1.0);
    double driveI = SmartDashboard.getNumber("Drive I " + m_driveMotor.getDeviceId(), 0.0);
    double driveD = SmartDashboard.getNumber("Drive D " + m_driveMotor.getDeviceId(), 0.0);

    double turnP = SmartDashboard.getNumber("Turn P " + m_turningMotor.getDeviceId(), 1.0);
    double turnI = SmartDashboard.getNumber("Turn I " + m_turningMotor.getDeviceId(), 0.0);
    double turnD = SmartDashboard.getNumber("Turn D " + m_turningMotor.getDeviceId(), 0.0);

    m_driveConfig.closedLoop.p(driveP).i(driveI).d(driveD);
    m_turningConfig.closedLoop.p(turnP).i(turnI).d(turnD);

    m_driveMotor.configure(m_driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_turningMotor.configure(m_turningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}