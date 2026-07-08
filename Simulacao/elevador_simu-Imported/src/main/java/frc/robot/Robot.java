package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;

public class Robot extends TimedRobot {
  private Elevator elevator;
  private Joystick joystick;

  @Override
  public void robotInit() {
    elevator = new Elevator();
    joystick = new Joystick(0); // Joystick na porta USB 0
  }

  @Override
  public void teleopPeriodic() {
    // Botão 1 -> sobe até o topo
    if (joystick.getRawButton(1)) {
      elevator.reachGoal(0.6);
    }
    // Botão 2 -> desce até a base
    else if (joystick.getRawButton(2)) {
      elevator.reachGoal(Constants.kMinElevatorHeightMeters);
    }
  
    // Nenhum botão -> para
    else {
      elevator.stop();
    }

    // Atualiza visualização no SmartDashboard
    elevator.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // Atualiza simulação do elevador
    elevator.simulationPeriodic();
  }
}
