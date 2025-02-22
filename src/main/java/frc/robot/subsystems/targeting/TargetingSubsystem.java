package frc.robot.subsystems.targeting;

import java.lang.StackWalker.Option;
import java.util.Optional;

import org.photonvision.PhotonCamera;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class TargetingSubsystem extends SubsystemBase {

    private PhotonCamera m_centerCamera;
    private Optional<Transform3d> m_nearestTarget;  // Transform to targets pose in Robot Frame
    private Optional<Pose2d> m_poseForNearestTarget; // Pose to align with nearest target (rotation is reversed) 
   
    // Constructor
    public TargetingSubsystem() {
        m_centerCamera = new PhotonCamera("center");
    }

    // Returns Pose of nearest Target in Robot Frame
    public  Optional<Transform3d> getTransformToNearestTargetInRobotFrame()
    {
        return m_nearestTarget;
    }

    // Returns Pose that aligns with nearest target in Robot Frame
    public  Optional<Pose2d> getPoseForNearestTargetInRobotFrame()
    {
        return m_poseForNearestTarget;
    }

    // Returns Pose of nearest target in Map Frame
    public Optional<Pose2d> getPoseForNearestTargetInMapFrame(Pose2d poseOfRobot)
    {
        if(m_poseForNearestTarget.isPresent())
        {
            Transform2d transform = new Transform2d(poseOfRobot.getTranslation(), poseOfRobot.getRotation());
            return Optional.of(m_poseForNearestTarget.get().transformBy(transform));
        }
        else
        {
            return Optional.empty();
        }
    }

    @Override
    public void periodic() {

        // Clear targets
        m_nearestTarget = Optional.empty();
        m_poseForNearestTarget = Optional.empty();

        // Read in relevant data from the Camera
        double area = 0.0;
        var results = m_centerCamera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    // Find largest AprilTag
                    if (target.area > area) {
                        area = target.area;
                        m_nearestTarget = Optional.of(target.getBestCameraToTarget());
                    }
                }
            }
        }
        
        // Send target to drivetrain
        if(m_nearestTarget.isPresent())
        {
            Rotation2d correction = new Rotation2d(Math.PI);
            Rotation2d rotationOfTarget = m_nearestTarget.get().getRotation().toRotation2d().plus(correction);
            Pose2d poseOfTarget = new Pose2d(m_nearestTarget.get().getX(),m_nearestTarget.get().getY(),rotationOfTarget);
            m_poseForNearestTarget = Optional.of(poseOfTarget);
        }
    }

    private void updateTelemetry() {
        SmartDashboard.putBoolean("targeting/nearest_target/visible", m_nearestTarget.isPresent());
        SmartDashboard.putNumber("targeting/nearest_target/x", m_nearestTarget.get().getX());
        SmartDashboard.putNumber("targeting/nearest_target/y", m_nearestTarget.get().getY());
        SmartDashboard.putNumber("targeting/nearest_target/z", m_nearestTarget.get().getZ());
        SmartDashboard.putNumber("targeting/nearest_target/roll", m_nearestTarget.get().getRotation().getX());
        SmartDashboard.putNumber("targeting/nearest_target/pitch", m_nearestTarget.get().getRotation().getY());
        SmartDashboard.putNumber("targeting/nearest_target/yaw", m_nearestTarget.get().getRotation().getZ());
    }

}