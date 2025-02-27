package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * A command to rezero the elevator
 */
public class ElevatorHome extends Command {
  enum State {start, moving_up, start_down, homing, finished};

  private final ElevatorSubsystem m_elevator;
  private double m_startHeightInInches;
  private double m_currentHeightInInches;
  private boolean m_isInitialized = false;
  private State m_state;

  // Constructor
  public ElevatorHome(ElevatorSubsystem elevator)
  {
    m_elevator = elevator;
  
    addRequirements(m_elevator);
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
        if(currentHeightInInches > (m_startHeightInInches + 3.0))
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
          m_state = State.finished;
        }
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
    System.out.println("Homing Ended!");
  }

}
