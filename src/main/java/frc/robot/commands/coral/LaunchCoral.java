package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * A command to control the elevator with a joystick axis
 */
public class LaunchCoral extends Command {
  enum State {start, launch, flick, elevator, finished};
  private final CoralSubsystem m_coral;
  private final ElevatorSubsystem m_elevator;
  private final Double m_power;
  private final Double m_elevatorPosition;

  private double m_startTime;
  private State m_state;

  // Constructor for launch 
  public LaunchCoral(CoralSubsystem coral, Double power)
  {
    this.m_coral = coral;
    this.m_power = power;
    this.m_elevator = null;
    this.m_elevatorPosition = 0.0;
    
    addRequirements(coral);
  }

  public LaunchCoral(CoralSubsystem coral, Double power, ElevatorSubsystem elevator, double positionElevator)
  {
    this.m_coral = coral;
    this.m_power = power;
    this.m_elevator = elevator;
    this.m_elevatorPosition = positionElevator;
    
    addRequirements(coral);
    addRequirements(elevator);
  }

  // Called once when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_startTime = System.currentTimeMillis();
    m_state = State.start;
  }
  
  // Called every cycle while command is active
  @Override
  public void execute()
  {
    double currentTime = System.currentTimeMillis();

    switch (m_state) {
      case start:
        m_coral.setManualPowerLaunch(-m_power);
        m_state = State.launch;
        break;
      case launch:
        m_coral.setManualPowerLaunch(-m_power);  
        if(currentTime > m_startTime + 150)
        {
          m_state = State.flick;
        }
        break;
      case flick:
        m_coral.setManualPowerLaunch(-m_power); 
        m_coral.setPositionArm(Constants.CoralConstants.positionStation);
        if (m_coral.isAtSetPointArm())
        {
          m_state = State.elevator;
        }
        // m_state = State.elevator;
      case elevator:
        if(m_elevator != null)
        {
          m_elevator.setPositionInches(m_elevatorPosition);
        }
        m_state = State.finished;
        break;
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
    m_coral.setManualPowerLaunch(0);
  }

}
