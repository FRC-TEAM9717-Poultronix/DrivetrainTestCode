package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

/**
 * A command to control the elevator with a joystick axis
 */
public class IntakeCoral extends Command {
  private final CoralSubsystem coral;
  private final Double power;

  // Constructor
  public IntakeCoral(CoralSubsystem coral, Double power)
  {
    this.coral = coral;
    this.power = power;
  
    addRequirements(coral);
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
    coral.setManualPowerLaunch(power);
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
    coral.setManualPowerLaunch(0.0);
  }

}
