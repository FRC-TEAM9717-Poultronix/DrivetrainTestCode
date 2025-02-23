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

public class AlgaeSubsystem  extends SubsystemBase {
    private final SparkMax m_angleMotor;
    private final SparkFlex m_powerMotor;
    private final RelativeEncoder m_encoderpower;
    private final AbsoluteEncoder m_encoderangle;
    private final SparkClosedLoopController m_closedLoopControllerpower;
    private final SparkClosedLoopController m_closedLoopControllerangle;

    private boolean m_atSetPointAngle = true;
    private boolean m_atSetPointPower = true;
    private boolean m_isManualAngle = true;
    private boolean m_isManualPower = true;

    private SparkMaxConfig m_powerConfig = new SparkMaxConfig();
    private SparkFlexConfig m_angleConfig = new SparkFlexConfig();

    private double m_setPointAngle = 0.0;
    private double m_currentVelocityAngle;
    private double m_currentPositionAngle;
    private double m_currentCurrentAngle;

    private double m_setPointPower = 0.0;
    private double m_currentVelocityPower;
    private double m_currentPositionPower;
    private double m_currentCurrentPower;    

    public enum anglePosition {
        UP(Constants.AlgaeArmConstants.up),
        POSITION_1(Constants.AlgaeArmConstants.Processor),
               DOWN(Constants.AlgaeArmConstants.down);

        private final double positionDegrees;

        anglePosition(double positionDegrees) {
                this.positionDegrees = positionDegrees;
        }
    }

        // Constructor
    public AlgaeSubsystem() {
            m_powerMotor = new SparkFlex(Constants.AlgaeArmConstants.powerID, MotorType.kBrushless);
            m_angleMotor = new SparkMax(Constants.AlgaeArmConstants.angleID, MotorType.kBrushless);
        
            m_encoderpower = m_powerMotor.getEncoder();
            m_encoderangle = m_angleMotor.getAbsoluteEncoder();

            m_closedLoopControllerpower = m_powerMotor.getClosedLoopController();
            m_closedLoopControllerangle = m_angleMotor.getClosedLoopController();

            configureMotors();
    }

       // Initialize motor settings
    private void configureMotors() {
        m_powerConfig.idleMode(IdleMode.kBrake)
                    .inverted(false)
                    .smartCurrentLimit(Constants.AlgaeArmConstants.MaxCurrentLimitPower)
                    .voltageCompensation(12.0);

        m_angleConfig.idleMode(IdleMode.kBrake)
                    .inverted(false)
                    .smartCurrentLimit(Constants.AlgaeArmConstants.MaxCurrentLimitAngle)
                    .voltageCompensation(12.0);

        m_powerConfig.encoder.positionConversionFactor(Constants.AlgaeArmConstants.countsPerDegreeAngle)
                    .velocityConversionFactor(Constants.AlgaeArmConstants.countsPerDegreeAngle/60);
                
        m_angleConfig.absoluteEncoder.zeroCentered(true)
                                     .positionConversionFactor(Constants.AlgaeArmConstants.countsPerDegreeAngle)
                                     .velocityConversionFactor(Constants.AlgaeArmConstants.countsPerDegreeAngle/60);

        m_angleConfig.softLimit.forwardSoftLimitEnabled(true).forwardSoftLimit(Constants.AlgaeArmConstants.down)
                    .reverseSoftLimitEnabled(true).reverseSoftLimit(Constants.AlgaeArmConstants.up);

        m_powerConfig.softLimit.forwardSoftLimitEnabled(false)
                    .reverseSoftLimitEnabled(false);

        // Special Leader settings
        m_powerConfig.closedLoop.pid(Constants.AlgaeArmConstants.kP_power, Constants.AlgaeArmConstants.kI_power, Constants.AlgaeArmConstants.kD_power)
                                .outputRange(-1.0, 1.0);

        m_angleConfig.closedLoop.maxMotion.maxVelocity(Constants.AlgaeArmConstants.maxVelocity)
                                        .maxAcceleration(Constants.AlgaeArmConstants.maxAcceleration)
                                        .allowedClosedLoopError(Constants.AlgaeArmConstants.posTolerance);  
                                        
        m_angleConfig.closedLoop.pid(Constants.AlgaeArmConstants.kP_angle, Constants.AlgaeArmConstants.kI_angle, Constants.AlgaeArmConstants.kD_angle)
                                .outputRange(-1.0, 1.0);

        // Send setting to motors
            m_powerMotor.configure(m_powerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            m_angleMotor.configure(m_angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            // Zero elevator motor assuming it is min position on startup 
            m_encoderpower.setPosition(0.0);
    }

    @Override
    public void periodic() {

        // Gather Telemetry
        m_currentVelocityPower = m_encoderpower.getVelocity();
        m_currentPositionPower = m_encoderpower.getPosition();
        m_currentCurrentPower  = m_powerMotor.getOutputCurrent();

        m_currentVelocityAngle = m_encoderangle.getVelocity();
        m_currentPositionAngle = m_encoderangle.getPosition();
        m_currentCurrentAngle  = m_angleMotor.getOutputCurrent();

        m_atSetPointPower = Math.abs(m_currentPositionPower - m_setPointPower) < AlgaeArmConstants.posTolerance;
        m_atSetPointAngle = Math.abs(m_currentPositionAngle - m_setPointAngle) < AlgaeArmConstants.posTolerance;

        // Update SmartDashboard
        updateTelemetry();

        if(!m_isManualPower)
        {
            m_closedLoopControllerpower.setReference(m_setPointPower, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

        if(!m_isManualAngle)
        {
            m_closedLoopControllerangle.setReference(m_setPointAngle, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, Constants.AlgaeArmConstants.kAF_angle);
        }
    }

    private void updateTelemetry() {
        SmartDashboard.putBoolean("algae/launch/is_manual", m_isManualPower);
        SmartDashboard.putNumber("algae/launch/position", m_currentPositionPower);
        SmartDashboard.putNumber("algae/launch/set_point", m_setPointPower);
        SmartDashboard.putNumber("algae/launch/velocity", m_currentVelocityPower);
        SmartDashboard.putNumber("elevator/motor_current", m_currentCurrentPower);
        
        SmartDashboard.putBoolean("algae/arm/is_manual", m_isManualAngle);
        SmartDashboard.putNumber("algae/arm/position", m_currentPositionAngle);
        SmartDashboard.putNumber("algae/arm/set_point", m_setPointAngle);
        SmartDashboard.putNumber("algae/arm/velocity", m_currentVelocityAngle);
        SmartDashboard.putNumber("algae/arm/motor_current", m_currentCurrentAngle);
    }

    public boolean isAtSetPointPower() {
        return m_atSetPointPower;
    }

    public boolean isAtSetPointAngle() {
        return m_atSetPointAngle;
    }

    public void setPositionPower(double degree) {

        m_isManualPower = false;

        m_setPointPower = degree;
    }

    public void setPositionAngle(double degree) {

        m_isManualAngle = false;

        m_setPointAngle = MathUtil.clamp(
            degree,
            CoralConstants.up,
            CoralConstants.down
        );
    }

    public void setManualPowerPower(double power) {
        // Disable PID control when in manual mode
        m_isManualPower = true;
        
        m_powerMotor.set(MathUtil.clamp(power, -AlgaeArmConstants.maxOutput, AlgaeArmConstants.maxOutput));
    }

    public void setManualPowerAngle(double power) {
        // Disable PID control when in manual mode
        m_isManualAngle = true;
        
        m_angleMotor.set(MathUtil.clamp(power + AlgaeArmConstants.kAF_angle * Math.cos(m_currentCurrentAngle), -AlgaeArmConstants.maxOutput, AlgaeArmConstants.maxOutput));
    }
}
