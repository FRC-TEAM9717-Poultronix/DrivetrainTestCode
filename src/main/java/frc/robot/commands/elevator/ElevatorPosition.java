package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class ElevatorPosition extends InstantCommand {
  private final ElevatorSubsystem elevator;
  private final double  position;

  public ElevatorPosition(ElevatorSubsystem elevator, double position)
  {
    System.out.print("ElevatorPosition Command: "); System.out.println(position);
    
    this.elevator = elevator;
    this.position = position;
  
    addRequirements(elevator);
  }

  @Override
  public void initialize()
  {
    elevator.setPositionInches(position);
  }

}
