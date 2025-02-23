package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

/**
 * A command to control the elevator with a joystick axis
 */
public class ElevatorVelocity extends Command {
  private final ElevatorSubsystem elevator;
  private final DoubleSupplier  vZ;

  // Constructor
  public ElevatorVelocity(ElevatorSubsystem elevator, DoubleSupplier vZ)
  {
    this.elevator = elevator;
    this.vZ = vZ;
  
    addRequirements(elevator);
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
    SmartDashboard.putString("elevator/cmd_vel", desiredVelocity.toString());

    // Make the robot move
    elevator.setManualPower(desiredVelocity);
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
