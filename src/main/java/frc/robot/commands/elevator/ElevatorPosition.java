package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * An instant command to set a new target position for the elevator
 */
public class ElevatorPosition extends InstantCommand {
  private final ElevatorSubsystem elevator;
  private final CoralSubsystem coral;
  private final AlgaeSubsystem algae;
  private final double  positionElevator;
  private final double  positionCoral;
  private final double  positionAlgae;

  // Constructor
  public ElevatorPosition(ElevatorSubsystem elevator, double positionElevator, 
                                CoralSubsystem coral,  double positionCoral,
                                AlgaeSubsystem algae, double positionAlgae)
  {
    this.elevator = elevator;
    this.coral = coral;
    this.algae = algae;

    this.positionElevator = positionElevator;
    this.positionCoral = positionCoral;
    this.positionAlgae = positionAlgae;
  
 //   addRequirements(elevator);
 //   addRequirements(coral);
 //   addRequirements(algae);
  }

  // Called once when the command is initially scheduled.
  @Override
  public void initialize()
  {
    elevator.setPositionInches(positionElevator);
    if(coral != null) coral.setPositionArm(positionCoral);
    if(algae != null) algae.setPositionAngle(positionAlgae);
  }

}
