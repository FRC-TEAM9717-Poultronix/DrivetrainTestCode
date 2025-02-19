package frc.robot.commands.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * An example command that uses an example subsystem.
 */
public class ElevatorHome extends Command {
  private final ElevatorSubsystem m_elevator;
  private double m_startHeightInInches;
  private double m_currentHeightInInches;
  private boolean m_isInitialized = false;

  enum State {start, moving_up, start_down, homing, finished};

  private State m_state;
  // private boolean m_softLimitDisabled = false;

  public ElevatorHome(ElevatorSubsystem elevator)
  {
    m_elevator = elevator;
  
    addRequirements(m_elevator);
  }

    /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    m_state = State.start;
    m_elevator.disableSoftLimits();
    m_startHeightInInches = m_elevator.getHeightInches();
  }

  @Override
  public void execute()
  {
    long currentTime = System.currentTimeMillis();
    double currentHeightInInches = m_elevator.getHeightInches();
    
    // Move up for a short time, then start down, set initialized flag, then check homing
    // the m_initialized variable ensures it is moving before the homing function is called
    // overwise the velocity check during homing could trigger on first pass
    switch (m_state) {
      case start:
        m_elevator.setManualPower(0.1);
        m_state = State.moving_up;
        break;
      case moving_up:
        System.out.print("Start Height: "); System.out.println(m_startHeightInInches);
        System.out.print("Current Height: "); System.out.println(currentHeightInInches);
        if(currentHeightInInches > (m_startHeightInInches + 3.0))
        {
          m_state = State.start_down;
          m_elevator.setManualPower(-0.1);
        }
        break;
      case start_down:
        System.out.print("Start Height: "); System.out.println(m_startHeightInInches);
        System.out.print("Current Height: "); System.out.println(currentHeightInInches);
        if(currentHeightInInches < (m_startHeightInInches + 1.0))
        {
          m_state = State.homing;
        }
        break;
      case homing:
        System.out.print("Velocity"); System.out.println(m_elevator.getVelocity());
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
    return m_state == State.finished;  // Move down until we stall;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    System.out.println("Homing Ended!");
  }

}
