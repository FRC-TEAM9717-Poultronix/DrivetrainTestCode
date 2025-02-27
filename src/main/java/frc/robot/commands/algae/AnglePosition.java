package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.AlgaeSubsystem;

/**
 * An instant command to set a new target position for the elevator
 */
public class AnglePosition extends InstantCommand {
  private final AlgaeSubsystem algae;
  private final double  position;

  // Constructor
  public AnglePosition(AlgaeSubsystem algae, double position)
  {
    this.algae = algae;
    this.position = position;
  
    addRequirements(algae);
  }

  // Called once when the command is initially scheduled.
  @Override
  public void initialize()
  {
    algae.setPositionAngle(position);
  }

}
