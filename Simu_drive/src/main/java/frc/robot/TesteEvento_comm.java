package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TesteEvento_comm extends Command {
    private final Timer timer = new Timer();
    public TesteEvento_comm() {
        // Não adicionamos requirement do Drive_sub para o robô não parar de andar!
    }

    @Override
    public void initialize() {
      System.out.println(">>> EVENTO TESTE DISPARADO! <<<");
        timer.reset(); // Zera o cronômetro
    timer.start(); // Inicia a contagem
    SmartDashboard.putBoolean("EventoAtivo", true);
       
    }

    @Override
    public void execute() {
        // Mantém ligado enquanto o comando rodar
    }

    @Override
    public boolean isFinished() {
        // Se você quer que ele rode por um tempo fixo (ex: 2 segundos)
        // return false; // Se for usar o "Wait Command" do PathPlanner
        return timer.hasElapsed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        // Desliga o booleano no Elastic
        SmartDashboard.putBoolean("EventoAtivo", false);
        System.err.println(">>>EVENTO TESTE FINALIZADO<<<");
    }
}