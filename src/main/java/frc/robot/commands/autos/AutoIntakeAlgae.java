package frc.robot.commands.autos;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.targeting.TargetingSubsystem;

/**
 * A command to rezero the elevator
 */
public class AutoIntakeAlgae extends Command {
  enum State {start, move_forward, move_backward, finished};

  private final SwerveSubsystem m_swerve;
  private final TargetingSubsystem m_targeting;
  private final AlgaeSubsystem m_algae;

  private final double m_distance;
  private final double m_tolTrans;
  private final double m_tolRot;
  private final double m_power;

  private double m_velX;
  private double m_velY;
  private double m_velYaw;

  private double m_startTime;
  private State m_state;

  // Constructor
  public AutoIntakeAlgae(SwerveSubsystem swerve, 
                         TargetingSubsystem targeting, double distance, double tolTrans, double tolRot, 
                         AlgaeSubsystem algae, double power)
  {
    m_swerve = swerve;
    m_targeting = targeting;
    m_algae = algae;

    m_distance = distance;
    m_tolTrans = tolTrans;
    m_tolRot = tolRot;
    m_power = power;

    m_velX = 0.0;
    m_velY = 0.0;
    m_velYaw = 0.0;
  
    addRequirements(swerve);
    addRequirements(algae);
  }

  // Called once when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_state = State.start;
    m_startTime = System.currentTimeMillis();
    m_algae.setManualPowerPower(-m_power);
  }

  // Called every cycle while command is active
  @Override
  public void execute()
  {
    double currentTime = System.currentTimeMillis();
    
    switch (m_state) {
      case start:
        if(currentTime > m_startTime + 100)
        {
          m_state = State.move_forward;
        }
        break;
      case move_forward:
      if(currentTime > m_startTime + 100)
        {
          Boolean isReadyX = false;
          Boolean isReadyY = false;
          Boolean isReadyYaw = false;

          m_velX = 0.0;
          m_velY = 0.0;
          m_velYaw = 0.0;
          
          // Get target
          Optional<Pose2d> poseTarget = m_targeting.getPoseForNearestTargetInRobotFrame();
          
          if(poseTarget.isPresent())
          {
            // Get distances from target
            double        targetX   = poseTarget.get().getX();
            double        targetY   = poseTarget.get().getY();
            Rotation2d    targetYaw = poseTarget.get().getRotation();
            // Check Distances
            if((targetX - m_distance) < m_tolTrans) isReadyX = true;
            if(targetY < m_tolTrans) isReadyY = true;
            if(targetYaw.getRadians() < m_tolRot) isReadyYaw = true;
            if(isReadyX && isReadyY && isReadyYaw) m_state = State.move_backward;
            // Update pids
            if(!isReadyX) m_velX = m_swerve.getSwerveController().xTranslationCalculate(m_distance, targetX);
            if(!isReadyY) m_velY = m_swerve.getSwerveController().yTranslationCalculate(0.0, targetY);
            if(!isReadyYaw) m_velYaw = m_swerve.getSwerveController().headingCalculate(0.0, targetYaw.getRadians());
            // Send speeds to drivetrain
            ChassisSpeeds speeds = new ChassisSpeeds(m_velX, m_velY, m_velYaw);
            m_swerve.getSwerveController().lastAngleScalar = m_swerve.getHeading().getRadians();
            m_swerve.drive(speeds);
          }
        }
        break;
      case move_backward:
        m_swerve.driveToDistanceCommand(1.0, -1.0);
        m_state = State.finished;
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
    System.out.println("Intake Algae Ended!");
  }

}
