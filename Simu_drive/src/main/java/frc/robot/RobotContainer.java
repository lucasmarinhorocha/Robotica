package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

  private final Drive_sub drive = new Drive_sub();
  private final Joystick joy = new Joystick(0);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
// No seu Drive_sub ou RobotContainer
NamedCommands.registerCommand("Teste", new TesteEvento_comm());

NamedCommands.registerCommand("Atirar",
    Commands.runOnce(() -> SmartDashboard.putBoolean("Atirando", true))
    .andThen(Commands.waitSeconds(0.6))
    .finallyDo((interrupted) -> SmartDashboard.putBoolean("Atirando", false))
);

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Seletor de Auto", autoChooser);

    configureBindings();

    drive.setDefaultCommand(new Drive_comm(drive, joy));
  }

  private void configureBindings() {
    // Controles aqui
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
