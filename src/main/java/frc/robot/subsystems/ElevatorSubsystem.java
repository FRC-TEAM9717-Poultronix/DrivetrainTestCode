package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
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
    private final SparkMax m_motorPrimary;
    private final SparkMax m_motorFollower;
    private final RelativeEncoder m_encoderPrimary;
    private final RelativeEncoder m_encoderFollower;
    private final SparkClosedLoopController m_closedLoopController;

    private boolean m_atSetPoint = false;
    private boolean m_isHomed = true;
    private boolean m_isManual = false;
    private boolean m_isStalled = false;

    private SparkMaxConfig m_leaderConfig = new SparkMaxConfig();
    private SparkMaxConfig m_followerConfig = new SparkMaxConfig();

    private double m_setpoint = ElevatorConstants.positionDown;
    private double m_velocityPrimary;
    private double m_positionPrimary;
    private double m_velocityFollower;
    private double m_positionFollower;
    private double m_outputPrimary;
    private double m_currentPrimary;
    private double m_outputFollower;
    private double m_currentFollower;

    // enum of pre-defined positions
    public enum ElevatorPosition {
        DOWN(Constants.ElevatorConstants.positionDown),
        POSITION_1(Constants.ElevatorConstants.positionL1),
        POSITION_2(Constants.ElevatorConstants.positionL2),
        POSITION_3(Constants.ElevatorConstants.positionL3),
        POSITION_4(Constants.ElevatorConstants.positionL4);
        public final double positionInches;
        
        ElevatorPosition(double positionInches) {
            this.positionInches = positionInches;
        }
    }

    // Constructor
    public ElevatorSubsystem() {
        m_motorPrimary = new SparkMax(ElevatorConstants.rightElevatorID, MotorType.kBrushless);
        m_motorFollower = new SparkMax(ElevatorConstants.leftElevatorID, MotorType.kBrushless);

        m_encoderPrimary = m_motorPrimary.getEncoder();
        m_encoderFollower = m_motorFollower.getEncoder();
        m_closedLoopController = m_motorPrimary.getClosedLoopController();
        
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
    
        m_leaderConfig.softLimit.forwardSoftLimitEnabled(true).forwardSoftLimit(Constants.ElevatorConstants.positionMax)
                                .reverseSoftLimitEnabled(true).reverseSoftLimit(Constants.ElevatorConstants.positionMin);

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
        m_followerConfig.follow(m_motorPrimary, true);  

        // Send setting to motors
        m_motorPrimary.configure(m_leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motorFollower.configure(m_followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Zero elevator motor assuming it is min position on startup 
        m_encoderPrimary.setPosition(Constants.ElevatorConstants.positionMin);
    }

    @Override
    public void periodic() {

        // Gather Telemetry
        m_velocityPrimary = m_encoderPrimary.getVelocity();
        m_positionPrimary = m_encoderPrimary.getPosition();
        m_velocityFollower = m_encoderFollower.getVelocity();
        m_positionFollower = m_encoderFollower.getPosition();

        m_outputPrimary = m_motorPrimary.getAppliedOutput();
        m_currentPrimary  = m_motorPrimary.getOutputCurrent();
        m_outputFollower = m_motorFollower.getAppliedOutput();
        m_currentFollower  = m_motorFollower.getOutputCurrent();

        m_isStalled = m_motorPrimary.getWarnings().stall;
        m_atSetPoint = Math.abs(m_positionPrimary - m_setpoint) < ElevatorConstants.posTolerance;

        // Update SmartDashboard
        updateTelemetry();

        if(m_isManual) return;

        if (!m_isHomed) return;

        m_closedLoopController.setReference(m_setpoint, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, Constants.ElevatorConstants.kAF, ArbFFUnits.kPercentOut);
    }

    private void updateTelemetry() {
        SmartDashboard.putBoolean("elevator/is_homed", m_isHomed);
        SmartDashboard.putBoolean("elevator/is_manual", m_isManual);

        SmartDashboard.putNumber("elevator/height", getHeightInches());
        SmartDashboard.putNumber("elevator/set_point", m_setpoint);

        SmartDashboard.putNumber("elevator/primary/position", m_positionPrimary);
        SmartDashboard.putNumber("elevator/primary/velocity", m_velocityPrimary);
        SmartDashboard.putNumber("elevator/primary/motor_output", m_outputPrimary);
        SmartDashboard.putNumber("elevator/primary/motor_current", m_currentPrimary);

        SmartDashboard.putNumber("elevator/follower/position", m_positionFollower);
        SmartDashboard.putNumber("elevator/follower/velocity", m_velocityFollower);
        SmartDashboard.putNumber("elevator/follower/motor_output", m_outputFollower);
        SmartDashboard.putNumber("elevator/follower/motor_current", m_currentFollower);
    }

    public void stopMotors() {
        m_motorPrimary.set(0);
    }

    public double getVelocity() {
        return m_velocityPrimary;
    }

    public double getOutput() {
        return m_outputPrimary;
    }

    public double getCurrent() {
        return m_currentPrimary;
    }

    public double getHeightInches() {
        return m_positionPrimary;
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
        return Math.abs(m_positionPrimary - position.positionInches) < ElevatorConstants.posTolerance;
    }

    public boolean disableSoftLimits() {
        m_isHomed = false;
        
        SoftLimitConfig newLimit = new SoftLimitConfig();
        newLimit.forwardSoftLimitEnabled(false)
                .reverseSoftLimitEnabled(false);
        
        m_leaderConfig.apply(newLimit);
        m_motorPrimary.configure(m_leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        return true;
    }

    public boolean enableSoftLimits() {
        SoftLimitConfig newLimit = new SoftLimitConfig();
        newLimit.forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);
        
        m_leaderConfig.apply(newLimit);
        m_motorPrimary.configure(m_leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        return true;
    }

    public boolean setHome() {
        
        REVLibError error =  m_encoderPrimary.setPosition(Constants.ElevatorConstants.positionMin);
        System.out.print("  Homed to "); System.out.println(Constants.ElevatorConstants.positionMin);
        enableSoftLimits();
        m_isHomed = true;

        setPositionInches(Constants.ElevatorConstants.positionDown);
        
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
            ElevatorConstants.positionMin,
            ElevatorConstants.positionMax
        );

        System.out.print("    setPoint: "); System.out.println(m_setpoint);
    }

    public void setManualPower(double power) {
        // Disable PID control when in manual mode
        m_isManual = true;
        
        m_motorPrimary.set(MathUtil.clamp(power + ElevatorConstants.kAF, -ElevatorConstants.maxOutput, ElevatorConstants.maxOutput));
    }

    public double getElevatorThrottle()
    {
        double numerator = (m_positionPrimary - ElevatorConstants.positionMin);
        double denominator = (ElevatorConstants.positionMax - ElevatorConstants.positionMin);
        
        return (1.0 - numerator/denominator);
    }
}