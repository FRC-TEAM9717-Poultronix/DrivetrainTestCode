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
  private final ElevatorSubsystem elevator;
  private long startTime;

  public ElevatorHome(ElevatorSubsystem elevator)
  {
    this.elevator = elevator;
  
    addRequirements(elevator);
  }

    /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    startTime = System.currentTimeMillis();
  }

  @Override
  public void execute()
  {
    long currentTime = System.currentTimeMillis();
    
    // Move up a short distance then down
    if(currentTime - startTime < 100) elevator.setManualPower(0.1);
      else elevator.setManualPower(-0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return elevator.isStalled();  // Move down until we stall;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

}
