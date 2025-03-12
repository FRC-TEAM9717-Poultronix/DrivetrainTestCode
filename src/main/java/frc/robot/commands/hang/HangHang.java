// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hang;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HangerConstants;
import frc.robot.subsystems.Hangersubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HangHang extends Command {
  /** Creates a new HangHang. */
  private Hangersubsystem m_hanger;

  public HangHang(Hangersubsystem hanger)
  {
    m_hanger = hanger; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_hanger);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_hanger.isHanging()) {
      System.out.println("is at set point");

    }
    if ( !m_hanger.isAtSetPoint()) {
      System.out.println("is at set point2");
    }
      if (m_hanger.isHanging() || !m_hanger.isAtSetPoint()) {

      System.out.println("ishanging");
      return;
    }
    System.out.println("ishanging2");

    m_hanger.setPosition(HangerConstants.HangAngle,true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("end");

    m_hanger.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_hanger.isHanging();
  }
}
