package frc.robot.commands.algae;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;
import java.util.function.DoubleSupplier;

/**
 * A command to control the elevator with a joystick axis
 */
public class AngleVelocity extends Command {
  private final AlgaeSubsystem algae;
  private final DoubleSupplier  vZ;

  // Constructor
  public AngleVelocity(AlgaeSubsystem algae, DoubleSupplier vZ)
  {
    this.algae = algae;
    this.vZ = vZ;
  
    addRequirements(algae);
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
  
    // Make the robot move
    algae.setManualPowerAngle(desiredVelocity);
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
    algae.setManualPowerAngle(0.0);
  }

}
