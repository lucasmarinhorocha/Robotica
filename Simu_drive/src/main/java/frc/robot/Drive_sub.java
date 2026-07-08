package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive_sub extends SubsystemBase {

  // ======================
  // 1. PARÂMETROS DE CONTROLE (CORRIGIDOS)
  // ======================
  private static final double TRACK_WIDTH = 0.65; 
  private static final double WHEEL_RADIUS = Units.inchesToMeters(3); // 0.0762m
  private static final double GEAR_RATIO = 10.71; 
  private static final double METERS_PER_ROTATION = (2 * Math.PI * WHEEL_RADIUS) / GEAR_RATIO;

  // Velocidades realistas para um Chassi CIM/NEO de 10.71 (~3.9 m/s teórico)
  private static final double MAX_SPEED = 3.5; 
  private static final double MAX_ANGULAR_SPEED = 6.0; 

  // ======================
  // 2. MOTORES E SENSORES
  // ======================
  private final SparkMax leftLeader = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax rightLeader = new SparkMax(2, MotorType.kBrushless);
  private final SparkMax leftFollower = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax rightFollower = new SparkMax(4, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
  private final Pigeon2 gyro = new Pigeon2(0);

  // ======================
  // 3. CONTROLE E FILTRAGEM
  // ======================
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);
  private final DifferentialDriveOdometry odometry;
  
  // PID ajustado para velocidade (Erro em m/s gerando saída em Volts)
  private final PIDController leftPID = new PIDController(1.5, 0.0, 0.0);
  private final PIDController rightPID = new PIDController(1.5, 0.0, 0.0);
  
  // kV recalculado: 12V / 3.9m/s = ~3.0. kS superando o atrito estático.
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.15, 2.8, 0.2);

  // Simulação
  private DifferentialDrivetrainSim drivetrainSim;
  private com.revrobotics.sim.SparkMaxSim leftSim;
  private com.revrobotics.sim.SparkMaxSim rightSim;
  
  private final Field2d field = new Field2d();

  public Drive_sub() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.encoder
        .positionConversionFactor(METERS_PER_ROTATION)
        .velocityConversionFactor(METERS_PER_ROTATION / 60.0);
    
    leftLeader.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    
    config.inverted(true); 
    rightLeader.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

    // Seguidores
    SparkMaxConfig leftFollowConfig = new SparkMaxConfig();
    leftFollowConfig.follow(1);
    leftFollower.configure(leftFollowConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

    SparkMaxConfig rightFollowConfig = new SparkMaxConfig();
    rightFollowConfig.follow(2);
    rightFollower.configure(rightFollowConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0, 0);

    if (RobotBase.isSimulation()) {
      initSimulation();
    }

    setupPathPlanner();
    SmartDashboard.putData("Field", field);
  }

  private void initSimulation() {
    leftSim = new com.revrobotics.sim.SparkMaxSim(leftLeader, DCMotor.getCIM(2));
    rightSim = new com.revrobotics.sim.SparkMaxSim(rightLeader, DCMotor.getCIM(2));

    drivetrainSim = new DifferentialDrivetrainSim(
        DCMotor.getCIM(2), 
        GEAR_RATIO, 
        6.0,                // Inércia Jóia (MOI)
        45.0,               // Peso do robô em kg
        WHEEL_RADIUS, 
        TRACK_WIDTH, 
        null
    );
  }

  private void setupPathPlanner() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose,
          this::resetPose,
          this::getRobotRelativeSpeeds,
          this::driveRobotRelative,
          new PPLTVController(0.02),
          config,
          () -> DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false),
          this
      );
    } catch (Exception e) {
      DriverStation.reportError("PathPlanner: " + e.getMessage(), e.getStackTrace());
    }
  }

  @Override
  public void periodic() {
    // Agora o Gyro funciona uniformemente tanto na simulação quanto no real
    odometry.update(
        gyro.getRotation2d(),
        leftEncoder.getPosition(),
        rightEncoder.getPosition()
    );
    field.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
    // Captura as tensões aplicadas pelos controladores virtuais
    double leftVolts = leftSim.getAppliedOutput() * 12.0;
    double rightVolts = -rightSim.getAppliedOutput() * 12.0; 

    drivetrainSim.setInputs(leftVolts, rightVolts);
    drivetrainSim.update(0.020);

    // Converte a velocidade linear da física simulada de volta para RPM da REV
    double leftRPM = (drivetrainSim.getLeftVelocityMetersPerSecond() / METERS_PER_ROTATION) * 60.0;
    double rightRPM = (drivetrainSim.getRightVelocityMetersPerSecond() / METERS_PER_ROTATION) * 60.0;

    leftSim.iterate(leftRPM, 12.0, 0.020);
    rightSim.iterate(-rightRPM, 12.0, 0.020); 

    // 🔥 SOLUÇÃO DO PIGEON: Atualiza o estado de simulação do hardware Phoenix 6 diretamente
    gyro.getSimState().setRawYaw(drivetrainSim.getHeading().getDegrees());
  }

  /**
   * Movimentação controlada por PID + Feedforward robusto
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    // Limitadores dinâmicos baseados no teto físico real do robô
    double vx = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, speeds.vxMetersPerSecond));
    double omega = Math.max(-MAX_ANGULAR_SPEED, Math.min(MAX_ANGULAR_SPEED, speeds.omegaRadiansPerSecond));
    
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(vx, 0, omega));

    // PID calcula a correção com base no erro de velocidade real vs desejada
    double leftOutput = leftPID.calculate(leftEncoder.getVelocity(), wheelSpeeds.leftMetersPerSecond)
                        + feedforward.calculate(wheelSpeeds.leftMetersPerSecond);
    
    double rightOutput = rightPID.calculate(rightEncoder.getVelocity(), wheelSpeeds.rightMetersPerSecond)
                         + feedforward.calculate(wheelSpeeds.rightMetersPerSecond);

    leftLeader.setVoltage(leftOutput);
    rightLeader.setVoltage(rightOutput);
  }

  /**
   * Controle Arcade suave para Teleoperado
   */
  public void setArcade(double speed, double rotation) {
    // Aplica uma curva cúbica para manter sensibilidade fina no centro do analógico
    double s = Math.pow(speed, 3);
    double r = Math.pow(rotation, 3);
    driveRobotRelative(new ChassisSpeeds(s * MAX_SPEED, 0, -r * MAX_ANGULAR_SPEED));
  }

  public Pose2d getPose() { return odometry.getPoseMeters(); }

  public void resetPose(Pose2d pose) {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    
    if (RobotBase.isSimulation()) {
      drivetrainSim.setPose(pose);
      gyro.getSimState().setRawYaw(pose.getRotation().getDegrees());
    } else {
      gyro.setYaw(pose.getRotation().getDegrees());
    }

    odometry.resetPosition(gyro.getRotation2d(), 0, 0, pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity())
    );
  }
}