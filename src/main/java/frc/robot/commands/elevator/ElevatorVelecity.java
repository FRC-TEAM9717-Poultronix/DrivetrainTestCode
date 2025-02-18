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
public class ElevatorVelecity extends Command {
  private final ElevatorSubsystem elevator;
  private final DoubleSupplier  vZ;

  public ElevatorVelecity(ElevatorSubsystem elevator, DoubleSupplier vZ)
  {
    this.elevator = elevator;
    this.vZ = vZ;
  
    addRequirements(elevator);
  }

  @Override
  public void execute()
  {

    // Get the desired speeds based on a joystick module.
    Double desiredVelocity = vZ.getAsDouble();

    // Limit velocity to prevent tippy
    SmartDashboard.putString("elevator/cmd_vel", desiredVelocity.toString());

    // Make the robot move
    elevator.setManualPower(desiredVelocity);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }


}
