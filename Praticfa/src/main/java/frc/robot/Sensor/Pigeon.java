package frc.robot.Sensor;

import com.ctre.phoenix6.hardware.Pigeon2;

public class Pigeon implements Sensor_INTER {


    Pigeon2 pigeon;
    
    public Pigeon(int canId) {
        this.pigeon = new Pigeon2(canId);
    }
    @Override
    public void Setsensor(double value) {
        pigeon.setYaw(value);
        
    }

   // @Override

   public double Getsensor() {
        return pigeon.getYaw().getValueAsDouble();
    }

    
    public double Getsensor(int escolha) {
        switch (escolha) {
            case 1:
                return pigeon.getYaw().getValueAsDouble();
            case 2:
                return pigeon.getPitch().getValueAsDouble();
            case 3:
                return pigeon.getRoll().getValueAsDouble();
            default:
                throw new IllegalArgumentException("Escolha inválida. Use 1 para Yaw, 2 para Pitch ou 3 para Roll.");
        }
       

    }

    @Override
    public void ResetSensor() {
        pigeon.setYaw(0);
        
    }

   
    
}