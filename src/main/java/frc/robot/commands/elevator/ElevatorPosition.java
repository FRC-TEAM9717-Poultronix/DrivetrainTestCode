package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * An instant command to set a new target position for the elevator
 */
public class ElevatorPosition extends InstantCommand {
  private final ElevatorSubsystem elevator;
  private final double  position;

  // Constructor
  public ElevatorPosition(ElevatorSubsystem elevator, double position)
  {
    System.out.print("ElevatorPosition Command: "); System.out.println(position);
    
    this.elevator = elevator;
    this.position = position;
  
    addRequirements(elevator);
  }

  // Called once when the command is initially scheduled.
  @Override
  public void initialize()
  {
    elevator.setPositionInches(position);
  }

}
