package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * A command to rezero the elevator
 */
public class ElevatorPositionAfterHome extends Command {
  enum State {start, moving_up, start_down, homing, position, finished};

  private final ElevatorSubsystem m_elevator;
  private final CoralSubsystem m_coral;
  private final AlgaeSubsystem m_algae;

  private final double  m_positionElevator;
  private final double  m_positionCoral;
  private final double  m_positionAlgae;

  private double m_startHeightInInches;
  private State m_state;

  // Constructor
  public ElevatorPositionAfterHome(ElevatorSubsystem elevator, double positionElevator, 
                                CoralSubsystem coral,  double positionCoral,
                                AlgaeSubsystem algae, double positionAlgae)
  {
    this.m_elevator = elevator;
    this.m_coral = coral;
    this.m_algae = algae;

    this.m_positionElevator = positionElevator;
    this.m_positionCoral = positionCoral;
    this.m_positionAlgae = positionAlgae;
  
    addRequirements(elevator);
    addRequirements(coral);
    addRequirements(algae);
  }

  // Called once when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_state = State.start;
    m_elevator.disableSoftLimits();
    m_startHeightInInches = m_elevator.getHeightInches();
  }

  // Called every cycle while command is active
  @Override
  public void execute()
  {
    double currentHeightInInches = m_elevator.getHeightInches();
    
    switch (m_state) {
      case start:
        m_elevator.setManualPower(0.1);
        m_state = State.moving_up;
        break;
      case moving_up:
        if(currentHeightInInches > (m_startHeightInInches + 2.0))
        {
          m_state = State.start_down;
          m_elevator.setManualPower(-0.1);
        }
        break;
      case start_down:
        if(currentHeightInInches < (m_startHeightInInches + 1.0))
        {
          m_state = State.homing;
        }
        break;
      case homing:
        if(m_elevator.getVelocity() > -0.1)
        {
          m_elevator.setHome();
          m_state = State.position;
        }
        break;
      case position:
        m_elevator.setPositionInches(m_positionElevator);
        if(m_coral != null) m_coral.setPositionArm(m_positionCoral);
        if(m_algae != null) m_algae.setPositionAngle(m_positionAlgae);
        m_state = State.finished; 
      default:
        break;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return m_state == State.finished;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    System.out.println("PositionAfterHoming Ended!");
  }

}
