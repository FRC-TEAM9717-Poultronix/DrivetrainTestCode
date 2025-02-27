package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CoralSubsystem;

/**
 * An instant command to set a new target position for the elevator
 */
public class ArmPosition extends InstantCommand {
  private final CoralSubsystem coral;
  private final double  position;

  // Constructor
  public ArmPosition(CoralSubsystem coral, double position)
  {
    this.coral = coral;
    this.position = position;
  
    addRequirements(coral);
  }

  // Called once when the command is initially scheduled.
  @Override
  public void initialize()
  {
    coral.setPositionArm(position);
  }

}
