package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

/**
 * A command to control the elevator with a joystick axis
 */
public class IntakeAlgae extends Command {
  private final AlgaeSubsystem algae;
  private final Double power;

  // Constructor
  public IntakeAlgae(AlgaeSubsystem algae, Double power)
  {
    this.algae = algae;
    this.power = power;
  
    addRequirements(algae);
  }

  // Called once when the command is initially scheduled.
  @Override
  public void initialize()
  {

  }
  
  // Called every cycle while command is active
  @Override
  public void execute()
  {
    algae.setManualPowerPower(-power);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false; // Should run continuously
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    algae.setManualPowerPower(0.0);
  }

}
