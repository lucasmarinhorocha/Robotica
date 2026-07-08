package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class Elevator implements AutoCloseable {

  private final DCMotor m_elevatorGearbox = DCMotor.getCIM(4);

  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          Constants.kElevatorKp,
          Constants.kElevatorKi,
          Constants.kElevatorKd,
          new TrapezoidProfile.Constraints(2.45, 2.45));

  private final ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          Constants.kElevatorkS,
          Constants.kElevatorkG,
          Constants.kElevatorkV,
          Constants.kElevatorkA);

  private final Encoder m_encoder =
      new Encoder(Constants.kEncoderAChannel, Constants.kEncoderBChannel);

  private final PWMSparkMax m_motor = new PWMSparkMax(Constants.kMotorPort);

  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          Constants.kElevatorGearing,
          Constants.kCarriageMass,
          Constants.kElevatorDrumRadius,
          Constants.kMinElevatorHeightMeters,
          Constants.kMaxElevatorHeightMeters,
          true,
          0,
          0.01,
          0.0);

  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  private final PWMSim m_motorSim = new PWMSim(m_motor);

  private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(new MechanismLigament2d("Elevator", 0, 90));

  /** Construtor do subsistema */
  public Elevator() {
    m_encoder.setDistancePerPulse(Constants.kElevatorEncoderDistPerPulse);
    
    // Configura cor e espessura para a simulação 2D
    m_elevatorMech2d.setColor(new Color8Bit(Color.kRed));
    m_elevatorMech2d.setLength(4);

    // ALTERAÇÃO: Inicializa a chave na tabela se não existir (evita loops de sobrescrita)
    if (!SmartDashboard.containsKey("Elevator Goal Target")) {
      SmartDashboard.putNumber("Elevator Goal Target", 0.0);
    }

    updateTelemetry();
    SmartDashboard.putData("Elevator Sim", m_mech2d);
  }

  /** Loop periódico principal (Deve ser chamado a cada 20ms no Robot principal) */
  public void periodic() {
    // ALTERAÇÃO: Lê o valor digitado externamente no AdvantageScope
    double alvoDesejado = SmartDashboard.getNumber("Elevator Goal Target", 0.0);
    
    // Move o robô em direção à meta definida pela interface gráfica
    reachGoal(alvoDesejado);
    
    updateTelemetry();
  }

  /** Atualiza a simulação física */
  public void simulationPeriodic() {
    m_elevatorSim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());
    m_elevatorSim.update(0.020);
    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }

  public void reachGoal(double goal) {
    m_controller.setGoal(goal);
    double pidOutput = m_controller.calculate(getElevatorPositionMeters());
    double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
    m_motor.setVoltage(pidOutput + feedforwardOutput);
  }

  public void stop() {
    SmartDashboard.putNumber("Elevator Goal Target", 0.0);
    m_motor.set(0.0);
  }

  public void updateTelemetry() {
    double posicaoAtual = getElevatorPositionMeters();
    m_elevatorMech2d.setLength(posicaoAtual * 10); 
    SmartDashboard.putNumber("Elevator Position", posicaoAtual);
  }

  public double getElevatorPositionMeters() {
    return m_encoder.getDistance();
  }

  @Override
  public void close() {
    m_encoder.close();
    m_motor.close();
    m_mech2d.close();
  }
}