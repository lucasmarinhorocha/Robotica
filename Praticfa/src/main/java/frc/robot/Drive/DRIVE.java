package frc.robot.Drive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType; // Import necessário para o tipo do motor
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Motor.Spark_INTER;
import frc.robot.Sensor.Pigeon;
import frc.robot.Sensor.Sensor_INTER;

public class DRIVE extends DRIVE_ABS {
 
  // 1. CORREÇÃO: Declaramos os motores no topo da classe para que a interface possa usá-los
  private SparkMax ESQ;
  private SparkMax DIR;
  private SparkMax ESQ2;
  private SparkMax DIR2;

  private Sensor_INTER pigeon = new Pigeon(1);

  public DRIVE() {
    // 2. CORREÇÃO: Inicializamos os motores com IDs reais e o tipo correto (ex: kBrushed)
    ESQ = new SparkMax(1, MotorType.kBrushed);
    DIR = new SparkMax(2, MotorType.kBrushed);
    ESQ2 = new SparkMax(4, MotorType.kBrushed);
    DIR2 = new SparkMax(3, MotorType.kBrushed);
    // Boa prática: Inverter o lado direito para não andar em círculos
    DIR.setInverted(true);
    pigeon.ResetSensor(); // Resetar o sensor ao iniciar

    configurarMotores(ESQ, DIR);
  }

  @Override
  public void periodic(){
    super.periodic();
    SmartDashboard.putNumber("Pigeon Yaw", pigeon.Getsensor());
    SmartDashboard.putNumber("Pigeon Pitch", pigeon.Getsensor(2));
    SmartDashboard.putNumber("Pigeon Roll", pigeon.Getsensor(3));
    
  }
  
    
  @Override
  public void configurarMotores(SparkMax ESQ, SparkMax DIR) {
    Follow(ESQ2, ESQ);
    Follow(DIR2, DIR);
    this.drive = new DifferentialDrive(ESQ, DIR);
  }

  @Override
  public void dirigir(Joystick joy){
    if (this.drive != null) {
        this.drive.arcadeDrive(joy.getRawAxis(1), joy.getRawAxis(4));
    }
  }

  // ==========================================
  // CORREÇÃO DOS MÉTODOS DA SUA INTERFACE (Spark_INTER)
  // ==========================================

  @Override
  public void setVelocidade(double forca) {
    // Agora usamos os nomes corretos das variáveis globais. 
    // Se quiser mover o robô em linha reta pela velocidade da interface:
    ESQ.set(forca);
    DIR.set(forca);
  }

  @Override
  public double getVelocidade() {
    // Retorna a média de velocidade dos dois motores, ou de um deles
    return ESQ.get();
  }

  @Override
  public void parar() {
    // Usa o objeto 'drive' herdado da classe abstrata para parar o robô inteiro com segurança
    if (this.drive != null) {
        this.drive.stopMotor();
    }
  }
}