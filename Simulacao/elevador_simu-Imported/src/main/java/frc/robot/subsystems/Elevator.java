package frc.robot.subsystems;

// Importações das ferramentas matemáticas, de simulação e controle do WPILib
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

/**
 * Subsistema do Elevador.
 * Implementa controle de loop fechado (PID + Feedforward) com Perfil Trapezoidal
 * e suporte a gêmeo digital para simulação física e visualização no AdvantageScope.
 */
public class Elevator implements AutoCloseable {

  /** Define a caixa de redução física do robô, configurada com 4 motores CIM. */
  private final DCMotor m_elevatorGearbox = DCMotor.getCIM(4);

  /**
   * Controlador PID Avançado (Profiled). 
   * Controla a aceleração e velocidade máxima (2.45 m/s) para criar um movimento
   * suave em formato de trapézio, evitando trancos mecânicos nas paradas.
   */
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          Constants.kElevatorKp,
          Constants.kElevatorKi,
          Constants.kElevatorKd,
          new TrapezoidProfile.Constraints(2.45, 2.45));

  /**
   * Controlador Feedforward para o Elevador.
   * Calcula antecipadamente a voltagem estática básica (kG) necessária para vencer
   * a força da gravidade e manter o elevador suspenso no ar sem cair.
   */
  private final ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          Constants.kElevatorkS,
          Constants.kElevatorkG,
          Constants.kElevatorkV,
          Constants.kElevatorkA);

  /** O sensor físico do Encoder, responsável por contar os pulsos nas portas digitais do robô. */
  private final Encoder m_encoder =
      new Encoder(Constants.kEncoderAChannel, Constants.kEncoderBChannel);

  /** O controlador eletrônico físico de velocidade (ESC) conectado na porta PWM do robô. */
  private final PWMSparkMax m_motor = new PWMSparkMax(Constants.kMotorPort);

  // ==========================================
  // CONFIGURAÇÕES DA SIMULAÇÃO FÍSICA
  // ==========================================

  /**
   * O motor de simulação física matemática.
   * Modela matematicamente a gravidade, massa do carrinho, o raio da polia e os limites de altura,
   * calculando a mecânica exata do elevador dentro do computador.
   */
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

  /** Objeto de simulação do Encoder. Permite "injetar" valores falsos na memória durante os testes no PC. */
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  
  /** Objeto de simulação do Motor. Monitora a velocidade teórica enviada ao SparkMax na simulação. */
  private final PWMSim m_motorSim = new PWMSim(m_motor);

  // ==========================================
  // CONFIGURAÇÕES DA INTERFACE VISUAL (2D MECHANISM)
  // ==========================================

  /** Cria a tela preta de desenho virtual com proporção de 20x50 de tamanho. */
  private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
  
  /** Define o ponto de partida (a base x=10, y=0) de onde a estrutura do elevador vai subir. */
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
  
  /** Cria a linha visual do elevador chamada "Elevator", inicialmente com tamanho 0 e apontando a 90° para cima. */
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(new MechanismLigament2d("Elevator", 0, 90));

  /**
   * Construtor da classe Elevator.
   * Executa as configurações iniciais de hardware assim que o robô liga.
   */
  public Elevator() {
    // Configura a conversão matemática de quantos metros o elevador anda por pulso do encoder
    m_encoder.setDistancePerPulse(Constants.kElevatorEncoderDistPerPulse);
    
    // Customiza o desenho do simulador: define a linha como vermelha e com espessura 4
    m_elevatorMech2d.setColor(new Color8Bit(Color.kRed));
    m_elevatorMech2d.setLength(4);

   

    // CRUCIAL PARA O MODO TUNING: Cria a caixinha editável no Dashboard APENAS se ela já não existir.
    // Isso impede que o código apague o valor digitado pelo usuário na tela durante o loop.
 
      SmartDashboard.putNumber("Elevator Goal Target", 1.0);
    

    // Publica o estado inicial e envia a tela de desenho 2D para a rede da NetworkTable
    updateTelemetry();
    SmartDashboard.putData("Elevator Sim", m_mech2d);
  }

  /**
   * Loop periódico principal do Subsistema.
   * Executado de forma contínua a cada 20 milissegundos, tanto no robô real quanto no simulador.
   */
  public void periodic() {
    // Captura em tempo real qual é o alvo de altura digitado no AdvantageScope/Dashboard
    double alvoDesejado = SmartDashboard.getNumber("Elevator Goal Target", 0.0);
    
    // Alimenta os cálculos matemáticos para deslocar o elevador até a altura capturada
    reachGoal(alvoDesejado);
    
    // Envia os dados atualizados de altura e o tamanho do desenho para a tela
    updateTelemetry();
  }

  /**
   * Loop de simulação de física computacional.
   * Atenção: Este método é executado exclusivamente no computador (Simulação) e é ignorado no robô real.
   */
  public void simulationPeriodic() {
    // 1. Pega a velocidade teórica do motor, multiplica pela voltagem da bateria e aplica como entrada no simulador físico
    m_elevatorSim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());
    
    // 2. Avança o relógio da simulação física em 20ms, processando a gravidade e forças de Newton
    m_elevatorSim.update(0.020);
    
    // 3. ENGENHARIA DA ILUSÃO: Injeta a altura calculada pela matemática direto na memória RAM do sensor Encoder simulado
    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
    
    // 4. Calcula o consumo elétrico dos motores CIM e simula a queda de tensão real da bateria do robô na RoboRIO
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }

  /**
   * Executa o cálculo de movimentação em Malha Fechada até o alvo desejado.
   * * @param goal A altura do alvo final onde o elevador deve parar, medida em metros.
   */
  public void reachGoal(double goal) {
    // Atualiza o objetivo do perfil trapezoidal do PID
    m_controller.setGoal(goal);
    
    // Calcula quantos Volts o PID exige com base no erro de distância atual
    double pidOutput = m_controller.calculate(getElevatorPositionMeters());
    
    // Calcula quantos Volts o Feedforward exige para neutralizar a gravidade na velocidade planejada
    double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
    
    // Soma as duas forças de tensão elétrica e envia diretamente para as bobinas do motor
    m_motor.setVoltage(pidOutput + feedforwardOutput);
  }

  /** Desliga o motor do elevador imediatamente e redefine o alvo no Dashboard para zero. */
  public void stop() {
   // SmartDashboard.putNumber("Elevator Goal Target", 0.0);
    m_motor.set(0.0);
  }

  /** Atualiza os dados de telemetria e redimensiona o desenho 2D do mecanismo. */
  public void updateTelemetry() {
    double posicaoAtual = getElevatorPositionMeters();
    
    // Multiplica a distância por 10 para dar uma proporção melhor no tamanho da linha visual da tela
    m_elevatorMech2d.setLength(posicaoAtual * 10); 
    
    // Envia o valor numérico puro da altura em metros para gráficos de linha ou tabelas
    SmartDashboard.putNumber("Elevator Position", posicaoAtual);
  }

  /**
   * Retorna a leitura atual de deslocamento do elevador.
   * * @return A distância percorrida medida em metros. (Lê o hardware real ou a simulação injetada).
   */
  public double getElevatorPositionMeters() {
    
    return m_encoder.getDistance();
  }

  /** Fecha todos os recursos de hardware de forma segura para evitar vazamento de memória se o subsistema parar. */
  @Override
  public void close() {
    m_encoder.close();
    m_motor.close();
    m_mech2d.close();
  }
}