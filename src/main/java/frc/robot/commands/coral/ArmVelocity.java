package frc.robot.commands.coral;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralSubsystem;
import java.util.function.DoubleSupplier;

/**
 * A command to control the elevator with a joystick axis
 */
public class ArmVelocity extends Command {
  private final CoralSubsystem coral;
  private final DoubleSupplier  vZ;

  // Constructor
  public ArmVelocity(CoralSubsystem coral, DoubleSupplier vZ)
  {
    this.coral = coral;
    this.vZ = vZ;
  
    addRequirements(coral);
  }

  // Called once when the command is initially scheduled.
  @Override
  public void initialize()
  {

  }
  
  // Called every cycle while command is active
  @Override
  public void execute()
  {
    // Get the desired speeds based on a joystick module.
    Double desiredVelocity = vZ.getAsDouble();
    SmartDashboard.putString("coral/arm/cmd_vel", desiredVelocity.toString());

    // Make the robot move
    coral.setManualPowerArm(desiredVelocity);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false; // Should run continuously
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

}
