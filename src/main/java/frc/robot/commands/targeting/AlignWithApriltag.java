package frc.robot.commands.targeting;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.targeting.TargetingSubsystem;

public class AlignWithApriltag extends Command {
    
    enum State {start, align, finished};

    private final SwerveSubsystem m_swerve;
    private final TargetingSubsystem m_targeting;

    private final double m_distanceForward;
    private final double m_distanceLateral;
    private final double m_tolTrans;
    private final double m_tolRot;

    private double m_velX;
    private double m_velY;
    private double m_velYaw;

    private double m_startTime;
    private State m_state;

    // Constructor
    public AlignWithApriltag(SwerveSubsystem swerve, 
                            TargetingSubsystem targeting, double distanceForward, double distanceLateral, double tolTrans, double tolRot, int fiducial)
    {
        m_swerve = swerve;
        m_targeting = targeting;

        m_distanceForward = distanceForward;
        m_distanceLateral = distanceLateral;
        m_tolTrans = tolTrans;
        m_tolRot = tolRot;

        m_targeting.setFiducial(fiducial);

        m_velX = 0.0;
        m_velY = 0.0;
        m_velYaw = 0.0;
    
        // addRequirements(swerve);
    }

    public AlignWithApriltag(SwerveSubsystem swerve, 
    TargetingSubsystem targeting, double distanceForward, double distanceLateral, double tolTrans, double tolRot)
    {
    m_swerve = swerve;
    m_targeting = targeting;

    m_distanceForward = distanceForward;
    m_distanceLateral = distanceLateral;
    m_tolTrans = tolTrans;
    m_tolRot = tolRot;

    m_targeting.setFiducial(-1);

    m_velX = 0.0;
    m_velY = 0.0;
    m_velYaw = 0.0;

    // addRequirements(swerve);
    }

      // Called once when the command is initially scheduled.
    @Override
    public void initialize()
    {
        m_state = State.start;
        m_startTime = System.currentTimeMillis();
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
                m_state = State.align;
            }
            break;
        case align:
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
                if(Math.abs(targetX - m_distanceForward) < m_tolTrans) isReadyX = true;
                if(Math.abs(targetY - m_distanceLateral) < m_tolTrans) isReadyY = true;
                if(targetYaw.getRadians() < m_tolRot) isReadyYaw = true;
                if(isReadyX && isReadyY && isReadyYaw) 
                {
                    m_state = State.finished;
                }
                // Update pids
                if(!isReadyX) m_velX = m_swerve.getSwerveController().xTranslationCalculate(m_distanceForward, targetX);
                if(!isReadyY) m_velY = m_swerve.getSwerveController().yTranslationCalculate(m_distanceLateral, targetY);
                if(!isReadyYaw) m_velYaw = m_swerve.getSwerveController().headingCalculate(0.0, targetYaw.getRadians());
                // Send speeds to drivetrain
                ChassisSpeeds speeds = new ChassisSpeeds(m_velX, m_velY, m_velYaw);
                m_swerve.drive(speeds);
            } else
            {
                ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
                 m_swerve.drive(speeds);
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
        ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        m_swerve.drive(speeds);
        System.out.println("Finished AlignWithApriltag!");
    }

}
