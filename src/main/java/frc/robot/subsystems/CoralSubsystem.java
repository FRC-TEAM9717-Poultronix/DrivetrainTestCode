package frc.robot.subsystems;

import org.dyn4j.UnitConversion;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
    private final SparkMax m_motorLaunch;
    private final SparkFlex m_motorArm;
    private final RelativeEncoder m_encoderLaunch;
    private final AbsoluteEncoder m_encoderArm;
    private final SparkClosedLoopController m_closedLoopControllerLaunch;
    private final SparkClosedLoopController m_closedLoopControllerArm;

    // private boolean m_atSetPointLaunch = true;
    private boolean m_atSetPointArm = true;
    private boolean m_isManualLaunch = true;
    private boolean m_isManualArm = false;

    private SparkMaxConfig m_configLaunch = new SparkMaxConfig();
    private SparkFlexConfig m_configArm = new SparkFlexConfig();

    // private double m_SetpointLaunch = 0.0;
    private double m_currentVelocityLaunch;
    private double m_currentPositionLaunch;
    private double m_currentCurrentLaunch;

    private double m_SetpointArm = Constants.CoralConstants.positionStation;
    private double m_currentVelocityArm;
    private double m_currentPositionArm;
    private double m_currentCurrentArm;

    public enum ArmPosition {
        UP(Constants.CoralConstants.positionMax),
        POSITION_1(Constants.CoralConstants.positionStation),
        POSITION_2(Constants.CoralConstants.positionHorizontal),
        POSITION_3(Constants.CoralConstants.positionReef),
        DOWN(Constants.CoralConstants.positionMin);

        private final double positionDegrees;

        ArmPosition(double positionDegrees) {
                this.positionDegrees = positionDegrees;
        }
    }

        // Constructor
    public CoralSubsystem() {
            m_motorLaunch = new SparkMax(Constants.CoralConstants.launchID, MotorType.kBrushless);
            m_motorArm = new SparkFlex(Constants.CoralConstants.armID, MotorType.kBrushless);
        
            m_encoderLaunch = m_motorLaunch.getEncoder();
            m_encoderArm = m_motorArm.getAbsoluteEncoder();

            m_closedLoopControllerLaunch = m_motorLaunch.getClosedLoopController();
            m_closedLoopControllerArm = m_motorArm.getClosedLoopController();

            configureMotors();
    }

   // Initialize motor settings
    private void configureMotors() {
        m_configLaunch.idleMode(IdleMode.kBrake)
                    .inverted(false)
                    .smartCurrentLimit(Constants.CoralConstants.MaxCurrentLimitLaunch)
                    .voltageCompensation(12.0);

        m_configArm.idleMode(IdleMode.kBrake)
                    .inverted(true)
                    .smartCurrentLimit(Constants.CoralConstants.MaxCurrentLimitArm)
                    .voltageCompensation(12.0);

        m_configLaunch.encoder.positionConversionFactor(CoralConstants.countsPerDegreeLaunch)
                    .velocityConversionFactor(CoralConstants.countsPerDegreeLaunch/60);

        m_configArm.encoder.positionConversionFactor(1.0)
                    .velocityConversionFactor(1.0);        
                    
        m_configArm.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);

        m_configArm.absoluteEncoder.zeroCentered(true)
                    .inverted(true)
                    .positionConversionFactor(Constants.CoralConstants.countsPerDegreeArm)
                    .velocityConversionFactor(Constants.CoralConstants.countsPerDegreeArm/60);

        m_configLaunch.softLimit.forwardSoftLimitEnabled(false)
                                .reverseSoftLimitEnabled(false);

        m_configArm.softLimit.forwardSoftLimitEnabled(true).forwardSoftLimit(Constants.CoralConstants.positionMax)
                    .reverseSoftLimitEnabled(true).reverseSoftLimit(Constants.CoralConstants.positionMin);

        // Special Leader settings
        // m_configLaunch.closedLoop.pid(Constants.CoralConstants.kP_launch, Constants.CoralConstants.kI_launch, Constants.CoralConstants.kD_launch)
        //                         .outputRange(-1.0, 1.0);

        m_configArm.closedLoop.maxMotion.maxVelocity(Constants.CoralConstants.maxVelocity)
                                        .maxAcceleration(Constants.CoralConstants.maxAcceleration)
                                        .allowedClosedLoopError(Constants.CoralConstants.posTolerance);  
                                        
        m_configArm.closedLoop.pid(Constants.CoralConstants.kP_arm, Constants.CoralConstants.kI_arm, Constants.CoralConstants.kD_arm)
                                .outputRange(-1.0, 1.0);

        // Send setting to motors
        m_motorLaunch.configure(m_configLaunch, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motorArm.configure(m_configArm, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {

        // Gather Telemetry
        m_currentVelocityLaunch = m_encoderLaunch.getVelocity();
        m_currentPositionLaunch = m_encoderLaunch.getPosition();
        m_currentCurrentLaunch  = m_motorLaunch.getOutputCurrent();

        m_currentVelocityArm = m_encoderArm.getVelocity();
        m_currentPositionArm = m_encoderArm.getPosition();
        m_currentCurrentArm  = m_motorArm.getOutputCurrent();

        // m_atSetPointLaunch = Math.abs(m_currentPositionLaunch - m_SetpointLaunch) < CoralConstants.posTolerance;
        m_atSetPointArm = Math.abs(m_currentPositionArm - m_SetpointArm) < CoralConstants.posTolerance;

        // Update SmartDashboard
        updateTelemetry();

        // if(!m_isManualLaunch)
        // {
        //     m_closedLoopControllerLaunch.setReference(m_SetpointLaunch, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        // }

        if(!m_isManualArm)
        {
            m_closedLoopControllerArm.setReference(m_SetpointArm, 
                                                SparkBase.ControlType.kMAXMotionPositionControl, 
                                                ClosedLoopSlot.kSlot0, 
                                                CoralConstants.kAF_arm * Math.cos(Units.degreesToRadians(m_currentPositionArm)),
                                                ArbFFUnits.kPercentOut);
        }
    }

    private void updateTelemetry() {
        SmartDashboard.putBoolean("coral/launch/is_manual", m_isManualLaunch);
        SmartDashboard.putNumber ("coral/launch/position", m_currentPositionLaunch);
        // SmartDashboard.putNumber("coral/launch/set_point", m_SetpointLaunch);
        SmartDashboard.putNumber ("coral/launch/velocity", m_currentVelocityLaunch);
        SmartDashboard.putNumber ("coral/motor_current", m_currentCurrentLaunch);
        
        SmartDashboard.putBoolean("coral/arm/is_manual", m_isManualArm);
        SmartDashboard.putNumber ("coral/arm/position", m_currentPositionArm);
        SmartDashboard.putNumber ("coral/arm/set_point", m_SetpointArm);
        SmartDashboard.putNumber ("coral/arm/velocity", m_currentVelocityArm);
        SmartDashboard.putNumber ("coral/arm/motor_current", m_currentCurrentArm);
    }

    public void stopMotors() {
        m_motorLaunch.set(0);
        m_motorArm.set(0);
    }

    // public boolean isAtSetPointLaunch() {
    //     return m_atSetPointLaunch;
    // }

    public boolean isAtSetPointArm() {
        return Math.abs(m_currentPositionArm - m_SetpointArm) < CoralConstants.posTolerance;
    }

    public void setPositionArm(double degree) {

        m_isManualArm = false;

        m_SetpointArm = MathUtil.clamp(
            degree,
            CoralConstants.positionMin,
            CoralConstants.positionMax
        );
    }

    public void setManualPowerLaunch(double power) {
        SmartDashboard.putNumber("coral/launch/cmd_vel", power);
        // Disable PID control when in manual mode
        if(Math.abs(power) > 0.01)
        {
            m_isManualLaunch = true;
            m_motorLaunch.set(MathUtil.clamp(power, -CoralConstants.maxOutput, CoralConstants.maxOutput));
        }
        else
        {
        //     m_SetpointLaunch = m_currentPositionLaunch;
            m_isManualLaunch = false;
            m_motorLaunch.set(MathUtil.clamp(power, -CoralConstants.maxOutput, CoralConstants.maxOutput));
        }
    }

    public void setManualPowerArm(double power) {
        SmartDashboard.putNumber("coral/arm/cmd_vel", power);
        // Disable PID control when in manual mode
        if(Math.abs(power) > 0.01)
        {
            m_isManualArm = true;
            m_motorArm.set(MathUtil.clamp(power + CoralConstants.kAF_arm * Math.cos(Units.degreesToRadians(m_currentPositionArm)), -CoralConstants.maxOutput, CoralConstants.maxOutput));
        }
        else
        {
            m_SetpointArm = m_currentPositionArm;
            m_isManualArm = false;
        }
    }
}
