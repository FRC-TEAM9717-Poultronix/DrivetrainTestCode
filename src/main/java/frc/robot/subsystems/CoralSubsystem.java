package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
    private final SparkMax m_launchMotor;
    private final SparkFlex m_armMotor;
    private final RelativeEncoder m_encoderLauncher;
    private final AbsoluteEncoder m_encoderArm;
    private final SparkClosedLoopController m_closedLoopControllerLaunch;
    private final SparkClosedLoopController m_closedLoopControllerArm;

    private boolean m_atSetPointLaunch = true;
    private boolean m_atSetPointArm = true;
    private boolean m_isManualLaunch = true;
    private boolean m_isManualArm = true;

    private SparkMaxConfig m_launchConfig = new SparkMaxConfig();
    private SparkFlexConfig m_armConfig = new SparkFlexConfig();

    private double m_SetpointLaunch = 0.0;
    private double m_currentVelocityLaunch;
    private double m_currentPositionLaunch;
    private double m_currentCurrentLaunch;

    private double m_SetpointArm = 0.0;
    private double m_currentVelocityArm;
    private double m_currentPositionArm;
    private double m_currentCurrentArm;

    public enum ArmPosition {
        UP(Constants.CoralConstants.up),
        POSITION_1(Constants.CoralConstants.U1),
        POSITION_2(Constants.CoralConstants.forward),
        POSITION_3(Constants.CoralConstants.D1),
        DOWN(Constants.CoralConstants.down);

        private final double positionDegrees;

        ArmPosition(double positionDegrees) {
                this.positionDegrees = positionDegrees;
        }
    }

        // Constructor
    public CoralSubsystem() {
            m_launchMotor = new SparkMax(Constants.CoralConstants.launchID, MotorType.kBrushless);
            m_armMotor = new SparkFlex(Constants.CoralConstants.armID, MotorType.kBrushless);
        
            m_encoderLauncher = m_launchMotor.getEncoder();
            m_encoderArm = m_armMotor.getAbsoluteEncoder();

            m_closedLoopControllerLaunch = m_launchMotor.getClosedLoopController();
            m_closedLoopControllerArm = m_armMotor.getClosedLoopController();

            configureMotors();
    }

   // Initialize motor settings
    private void configureMotors() {
        m_launchConfig.idleMode(IdleMode.kBrake)
                    .inverted(false)
                    .smartCurrentLimit(Constants.CoralConstants.MaxCurrentLimitLaunch)
                    .voltageCompensation(12.0);

        m_armConfig.idleMode(IdleMode.kBrake)
                    .inverted(false)
                    .smartCurrentLimit(Constants.CoralConstants.MaxCurrentLimitArm)
                    .voltageCompensation(12.0);

        m_launchConfig.encoder.positionConversionFactor(CoralConstants.countsPerDegreeLaunch)
                    .velocityConversionFactor(CoralConstants.countsPerDegreeLaunch/60);

        m_armConfig.absoluteEncoder.zeroCentered(true)
                    .positionConversionFactor(Constants.CoralConstants.countsPerDegreeArm)
                    .velocityConversionFactor(Constants.CoralConstants.countsPerDegreeArm/60);

        m_launchConfig.softLimit.forwardSoftLimitEnabled(false)
                    .reverseSoftLimitEnabled(false);

        m_armConfig.softLimit.forwardSoftLimitEnabled(true).forwardSoftLimit(Constants.CoralConstants.down)
                    .reverseSoftLimitEnabled(true).reverseSoftLimit(Constants.CoralConstants.up);


        // Special Leader settings
        m_launchConfig.closedLoop.pid(Constants.CoralConstants.kP_launch, Constants.CoralConstants.kI_launch, Constants.CoralConstants.kD_launch)
                                .outputRange(-1.0, 1.0);

        m_armConfig.closedLoop.maxMotion.maxVelocity(Constants.CoralConstants.maxVelocity)
                                        .maxAcceleration(Constants.CoralConstants.maxAcceleration)
                                        .allowedClosedLoopError(Constants.CoralConstants.posTolerance);  
                                        
        m_armConfig.closedLoop.pid(Constants.CoralConstants.kP_arm, Constants.CoralConstants.kI_arm, Constants.CoralConstants.kD_arm)
                                .outputRange(-1.0, 1.0);

        // Send setting to motors
            m_launchMotor.configure(m_launchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            m_armMotor.configure(m_armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            // Zero elevator motor assuming it is min position on startup 
            m_encoderLauncher.setPosition(0.0);
    }

    @Override
    public void periodic() {

        // Gather Telemetry
        m_currentVelocityLaunch = m_encoderLauncher.getVelocity();
        m_currentPositionLaunch = m_encoderLauncher.getPosition();
        m_currentCurrentLaunch  = m_launchMotor.getOutputCurrent();

        m_currentVelocityArm = m_encoderArm.getVelocity();
        m_currentPositionArm = m_encoderArm.getPosition();
        m_currentCurrentArm  = m_armMotor.getOutputCurrent();

        m_atSetPointLaunch = Math.abs(m_currentPositionLaunch - m_SetpointLaunch) < CoralConstants.posTolerance;
        m_atSetPointArm = Math.abs(m_currentPositionArm - m_SetpointArm) < CoralConstants.posTolerance;

        // Update SmartDashboard
        updateTelemetry();

        if(!m_isManualLaunch)
        {
            m_closedLoopControllerLaunch.setReference(m_SetpointLaunch, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

        if(!m_isManualArm)
        {
            m_closedLoopControllerArm.setReference(m_SetpointArm, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, Constants.CoralConstants.kAF_arm);
        }
    }

    private void updateTelemetry() {
        SmartDashboard.putBoolean("coral/launch/is_manual", m_isManualLaunch);
        SmartDashboard.putNumber("coral/launch/position", m_currentPositionLaunch);
        SmartDashboard.putNumber("coral/launch/set_point", m_SetpointLaunch);
        SmartDashboard.putNumber("coral/launch/velocity", m_currentVelocityLaunch);
        SmartDashboard.putNumber("elevator/motor_current", m_currentCurrentLaunch);
        
        SmartDashboard.putBoolean("coral/arm/is_manual", m_isManualArm);
        SmartDashboard.putNumber("coral/arm/position", m_currentPositionArm);
        SmartDashboard.putNumber("coral/arm/set_point", m_SetpointArm);
        SmartDashboard.putNumber("elevator/velocity", m_currentVelocityArm);
        SmartDashboard.putNumber("elevator/motor_current", m_currentCurrentArm);
    }

    public boolean isAtSetPointLaunch() {
        return m_atSetPointLaunch;
    }

    public boolean isAtSetPointArm() {
        return m_atSetPointArm;
    }

    public void setPositionLaunch(double degree) {

        m_isManualLaunch = false;

        m_SetpointLaunch = degree;
    }

    public void setPositionArm(double degree) {

        m_isManualArm = false;

        m_SetpointArm = MathUtil.clamp(
            degree,
            CoralConstants.up,
            CoralConstants.down
        );
    }

    public void setManualPowerLaunch(double power) {
        // Disable PID control when in manual mode
        m_isManualLaunch = true;
        
        m_armMotor.set(MathUtil.clamp(power, -CoralConstants.maxOutput, CoralConstants.maxOutput));
    }

    public void setManualPowerArm(double power) {
        // Disable PID control when in manual mode
        m_isManualArm = true;
        
        m_armMotor.set(MathUtil.clamp(power + CoralConstants.kAF_arm * Math.cos(m_currentCurrentArm), -CoralConstants.maxOutput, CoralConstants.maxOutput));
    }
}
