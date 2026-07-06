package frc.robot.Drive;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Motor.Spark_INTER;

public abstract class DRIVE_ABS extends SubsystemBase implements Spark_INTER {

    // O objeto 'drive' continua aqui para que os métodos possam usá-lo
    protected DifferentialDrive drive;

    // Construtor vazio: a classe mãe não exige mais que o drive venha de fora
    public DRIVE_ABS() {}
    
    public abstract void configurarMotores(SparkMax ESQ, SparkMax DIR);
    public abstract void dirigir(Joystick joy);

   

    public void dashboard() {
        if (drive != null) {
            SmartDashboard.putData("Drivetrain", drive);
            //SmartDashboard.putNumber("algo", this.getVelocidade());
        }   
    }

    @Override 
    public void periodic() {
        dashboard();
    }

    
   
}