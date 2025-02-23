package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_primaryMotor;
    private final SparkMax m_followerMotor;
    private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_closedLoopController;

    private boolean m_atSetPoint = true;
    private boolean m_isHomed = false;
    private boolean m_isManual = true;
    private boolean m_isStalled = false;

    private SparkMaxConfig m_leaderConfig = new SparkMaxConfig();
    private SparkMaxConfig m_followerConfig = new SparkMaxConfig();

    private double m_setpoint = 0.0;
    private double m_currentVelocity;
    private double m_currentPosition;
    private double m_currentCurrent;

    // enum of pre-defined positions
    public enum ElevatorPosition {
        DOWN(Constants.ElevatorConstants.downPos),
        POSITION_1(Constants.ElevatorConstants.L1),
        POSITION_2(Constants.ElevatorConstants.L2),
        POSITION_3(Constants.ElevatorConstants.L3),
        POSITION_4(Constants.ElevatorConstants.L4),
        Processor_Position(Constants.AlgaeArmConstants.Processor);
        public final double positionInches;
        
        ElevatorPosition(double positionInches) {
            this.positionInches = positionInches;
        }
    }

    // Constructor
    public ElevatorSubsystem() {
        m_primaryMotor = new SparkMax(ElevatorConstants.rightElevatorID, MotorType.kBrushless);
        m_followerMotor = new SparkMax(ElevatorConstants.leftElevatorID, MotorType.kBrushless);

        m_encoder = m_primaryMotor.getEncoder();
        m_closedLoopController = m_primaryMotor.getClosedLoopController();
        
        configureMotors();
    }

    // Initialize motor settings
    private void configureMotors() {

        // Settings
        m_leaderConfig.idleMode(IdleMode.kBrake)
                      .inverted(false)
                      .smartCurrentLimit(Constants.ElevatorConstants.MaxCurrentLimit)
                      .voltageCompensation(12.0);

        m_followerConfig.idleMode(IdleMode.kBrake)
                        .inverted(false)
                        .smartCurrentLimit(Constants.ElevatorConstants.MaxCurrentLimit)
                        .voltageCompensation(12.0);   

        m_leaderConfig.encoder.positionConversionFactor(ElevatorConstants.countsPerInch)
                              .velocityConversionFactor(ElevatorConstants.countsPerInch/60);

        m_followerConfig.encoder.positionConversionFactor(ElevatorConstants.countsPerInch)
                                .velocityConversionFactor(ElevatorConstants.countsPerInch/60);
    
        m_leaderConfig.softLimit.forwardSoftLimitEnabled(true).forwardSoftLimit(Constants.ElevatorConstants.maxPos)
                                .reverseSoftLimitEnabled(true).reverseSoftLimit(Constants.ElevatorConstants.minPos);

        m_followerConfig.softLimit.forwardSoftLimitEnabled(false)
                                  .reverseSoftLimitEnabled(false);

        // Special Leader settings
        m_leaderConfig.closedLoop.maxMotion.maxVelocity(Constants.ElevatorConstants.maxVelocity)
                                        .maxAcceleration(Constants.ElevatorConstants.maxAcceleration)
                                        .allowedClosedLoopError(Constants.ElevatorConstants.posTolerance);  
                                        
        m_leaderConfig.closedLoop.pid(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD)
                                 .iZone(ElevatorConstants.kIz)
                              .outputRange(-1.0, 1.0);

        // Special follower settings
        m_followerConfig.follow(m_primaryMotor, true);  

        // Send setting to motors
        m_primaryMotor.configure(m_leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_followerMotor.configure(m_followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Zero elevator motor assuming it is min position on startup 
        m_encoder.setPosition(Constants.ElevatorConstants.minPos);
    }

    @Override
    public void periodic() {

        // Gather Telemetry
        m_currentVelocity = m_encoder.getVelocity();
        m_currentPosition = m_encoder.getPosition();
        m_currentCurrent  = m_primaryMotor.getOutputCurrent();
        m_isStalled = m_primaryMotor.getWarnings().stall;
        m_atSetPoint = Math.abs(m_currentPosition - m_setpoint) < ElevatorConstants.posTolerance;

        // Update SmartDashboard
        updateTelemetry();

        if(m_isManual) return;

        if (!m_isHomed) return;

        m_closedLoopController.setReference(m_setpoint, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, Constants.ElevatorConstants.kAF);
    }

    private void updateTelemetry() {
        SmartDashboard.putBoolean("elevator/is_homed", m_isHomed);
        SmartDashboard.putBoolean("elevator/is_manual", m_isManual);

        SmartDashboard.putNumber("elevator/height", getHeightInches());
        SmartDashboard.putNumber("elevator/set_point", m_setpoint);

        SmartDashboard.putNumber("elevator/velocity", getVelocity());
        SmartDashboard.putNumber("elevator/motor_current", getCurrent());
    }

    public void stopMotors() {
        m_primaryMotor.set(0);
    }

    public double getVelocity() {
        return m_currentVelocity;
    }

    public double getCurrent() {
        return m_currentCurrent;
    }

    public double getHeightInches() {
        return m_currentPosition;
    }

    public boolean isStalled() {
        return m_isStalled;
    }

    public boolean isHomed() {
        return m_isHomed;
    }

    public boolean isAtSetPoint() {
        return m_atSetPoint;
    }
    
    public boolean isAtPosition(ElevatorPosition position) {
        return Math.abs(m_currentPosition - position.positionInches) < ElevatorConstants.posTolerance;
    }

    public boolean disableSoftLimits() {
        m_isHomed = false;
        
        SoftLimitConfig newLimit = new SoftLimitConfig();
        newLimit.forwardSoftLimitEnabled(false)
                .reverseSoftLimitEnabled(false);
        
        m_leaderConfig.apply(newLimit);
        m_primaryMotor.configure(m_leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        return true;
    }

    public boolean enableSoftLimits() {
        SoftLimitConfig newLimit = new SoftLimitConfig();
        newLimit.forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);
        
        m_leaderConfig.apply(newLimit);
        m_primaryMotor.configure(m_leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        return true;
    }

    public boolean setHome() {
        
        REVLibError error =  m_encoder.setPosition(Constants.ElevatorConstants.minPos);
        System.out.print("  Homed to "); System.out.println(Constants.ElevatorConstants.minPos);
        enableSoftLimits();
        m_isHomed = true;

        setPositionInches(Constants.ElevatorConstants.downPos);
        
        return true;
    }

    public void setPositionInches(double inches) {
        if (!m_isHomed && inches > 0) {
            System.out.println("Warning: Elevator not homed! Home first before moving to positions.");
            return;
        }

        m_isManual = false;

        System.out.print("  setPositionInches: "); System.out.println(inches);

        m_setpoint = MathUtil.clamp(
            inches,
            ElevatorConstants.minPos,
            ElevatorConstants.maxPos
        );

        System.out.print("    setPoint: "); System.out.println(m_setpoint);
    }

    public void setManualPower(double power) {
        // Disable PID control when in manual mode
        m_isManual = true;
        
        m_primaryMotor.set(MathUtil.clamp(power + ElevatorConstants.kAF, -ElevatorConstants.maxOutput, ElevatorConstants.maxOutput));
    }
}