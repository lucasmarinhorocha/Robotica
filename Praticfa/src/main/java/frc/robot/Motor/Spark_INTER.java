package frc.robot.Motor;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public interface Spark_INTER {
    
    // Agora os métodos controlam o motor interno da classe, sem precisar pedir o SparkMax por parâmetro
   //  void newSpark();

  

    void setVelocidade(double forca);
    double getVelocidade();
    void parar();
    
    // Lógica padrão simplificada
    default void rodarComCondicao(boolean ativacao, double forca) {
        if (ativacao) {
            setVelocidade(forca);
        } else {
            parar();
        }
    }

     public default void Follow(SparkMax escravo, SparkMax lider){
           SparkMaxConfig follow = new SparkMaxConfig();
           
            follow.follow(lider.getDeviceId());

        escravo.configure(follow, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
         }
}