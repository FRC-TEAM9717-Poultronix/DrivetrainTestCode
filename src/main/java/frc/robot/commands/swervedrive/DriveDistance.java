package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveDistance extends Command{

    private final SwerveSubsystem m_swerve;

    private final Double m_distance;
    private final Double m_speed;

    private Translation2d m_startPose;

    //Constructor
    public DriveDistance(SwerveSubsystem swerve, Double distance, Double speed)
    {
        m_swerve = swerve;
        m_distance = distance;
        m_speed = speed;

        // addRequirements(swerve);
    }

    @Override
    public void initialize()
    {
        m_startPose = m_swerve.getPose().getTranslation();
        m_swerve.drive(new ChassisSpeeds(m_speed, 0, 0));
    }
    
    // Called every cycle while command is active
    @Override
    public void execute()
    {
        if(m_startPose.getDistance(m_swerve.getPose().getTranslation()) < m_distance)
        {
            m_swerve.drive(new ChassisSpeeds(m_speed, 0, 0));
        }
        else
        {
            m_swerve.drive(new ChassisSpeeds(0.0, 0, 0));
        }
    }
    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return m_startPose.getDistance(m_swerve.getPose().getTranslation()) > m_distance;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        m_swerve.drive(new ChassisSpeeds(0, 0, 0));
    }
    
}
