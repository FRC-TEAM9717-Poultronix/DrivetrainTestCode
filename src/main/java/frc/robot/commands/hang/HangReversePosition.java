// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hang;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.HangerConstants;
import frc.robot.subsystems.Hangersubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HangReversePosition extends InstantCommand {
  private Hangersubsystem m_hanger;

  public HangReversePosition(Hangersubsystem hanger)
  {
    m_hanger = hanger; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_hanger);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Hang reverse position initialize");
    if (m_hanger.isAtSetPoint() || m_hanger.isHanging()) {
      return;
    }
    System.out.println("Hang reverse position initialize 2");

    m_hanger.setPosition(HangerConstants.ReverseAngle,false);
  }
}
