package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * A command to rezero the elevator
 */
public class ElevatorHome extends Command {
  enum State {start, start_down, homing, finished};

  private final ElevatorSubsystem m_elevator;
  private double m_startTime;
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
  }

  // Called every cycle while command is active
  @Override
  public void execute()
  {
    double currentTime = System.currentTimeMillis();
    
    switch (m_state) {
      case start:
        m_elevator.setManualPower(-0.1);
        m_state = State.start_down;
        break;
      case start_down:
      if(currentTime > m_startTime + 100)
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
