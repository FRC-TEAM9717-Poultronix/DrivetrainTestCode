package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coral.LaunchCoral;
import frc.robot.commands.elevator.ElevatorPosition;
import frc.robot.commands.swervedrive.DriveDistance;
import frc.robot.commands.targeting.AlignWithApriltag;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.targeting.TargetingSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;

public class AutoScoreCoral extends SequentialCommandGroup
{
    public AutoScoreCoral(TargetingSubsystem targeting,
                          Double distLateral,
                          Double distForward,
                          int fiducial,
                          SwerveSubsystem swerve,                           
                          ElevatorSubsystem elevator,
                          CoralSubsystem coral, 
                          AlgaeSubsystem algae) 
    {
        addCommands(
          new AlignWithApriltag(swerve, targeting, distForward * 2.0, distLateral, 0.05, 0.1, fiducial)
          ,new DriveDistance(swerve, distForward, 0.85)
          ,new LaunchCoral(coral, 1.0)
          ,new ElevatorPosition(elevator, Constants.ElevatorConstants.positionDown, coral, Constants.CoralConstants.positionUp, algae, Constants.AlgaeArmConstants.positionUp)
        );
    }
}