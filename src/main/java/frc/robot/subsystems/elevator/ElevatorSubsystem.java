package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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

    private ElevatorPosition m_currentTarget = ElevatorPosition.DOWN;
    private boolean m_atSetPoint = true;
    private boolean m_isHomed = false;
    private double m_setpoint = 0.0;
    private boolean m_isManual = true;
    private SparkMaxConfig m_leaderConfig = new SparkMaxConfig();
    private SparkMaxConfig m_followerConfig = new SparkMaxConfig();
    double currentPos;

    public enum ElevatorPosition {
        DOWN(Constants.ElevatorConstants.downPos),
        POSITION_1(Constants.ElevatorConstants.L1),
        POSITION_2(Constants.ElevatorConstants.L2),
        POSITION_3(Constants.ElevatorConstants.L3),
        POSITION_4(Constants.ElevatorConstants.L4);

        public final double positionInches;
        
        ElevatorPosition(double positionInches) {
            this.positionInches = positionInches;
        }
    }
 
    public ElevatorSubsystem() {
        m_primaryMotor = new SparkMax(ElevatorConstants.rightElevatorID, MotorType.kBrushless);
        m_followerMotor = new SparkMax(ElevatorConstants.leftElevatorID, MotorType.kBrushless);

        m_encoder = m_primaryMotor.getEncoder();
        m_closedLoopController = m_primaryMotor.getClosedLoopController();
        
        configureMotors();
    }

    private void configureMotors() {

        m_leaderConfig.idleMode(IdleMode.kBrake)
                   .smartCurrentLimit(Constants.ElevatorConstants.MaxCurrentLimit)
                   .voltageCompensation(12.0);

        m_followerConfig.idleMode(IdleMode.kBrake)
                   .smartCurrentLimit(Constants.ElevatorConstants.MaxCurrentLimit)
                   .voltageCompensation(12.0);   

        m_leaderConfig.encoder.positionConversionFactor(ElevatorConstants.countsPerInch);
        m_leaderConfig.encoder.velocityConversionFactor(ElevatorConstants.countsPerInch/60);

        m_followerConfig.encoder.positionConversionFactor(ElevatorConstants.countsPerInch);
        m_followerConfig.encoder.velocityConversionFactor(ElevatorConstants.countsPerInch/60);
    
        m_leaderConfig.softLimit.forwardSoftLimitEnabled(true).forwardSoftLimit(Constants.ElevatorConstants.maxPos);
        m_leaderConfig.softLimit.reverseSoftLimitEnabled(true).reverseSoftLimit(Constants.ElevatorConstants.minPos);

        m_followerConfig.softLimit.forwardSoftLimitEnabled(false);
        m_followerConfig.softLimit.reverseSoftLimitEnabled(false);
        
        m_leaderConfig.closedLoop.maxMotion.maxVelocity(Constants.ElevatorConstants.maxVelocity)
                                        .maxAcceleration(Constants.ElevatorConstants.maxAcceleration)
                                        .allowedClosedLoopError(Constants.ElevatorConstants.posTolerance);  
                                        
        m_leaderConfig.closedLoop.pid(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD)
                              .outputRange(-1.0, 1.0);

        // Configure follower
        m_followerConfig.follow(m_primaryMotor, true);  

        // Primary motor configuration
        m_primaryMotor.configure(m_leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Follower motor configuration
        m_followerMotor.configure(m_followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_primaryMotor.setInverted(false);
        m_followerMotor.setInverted(false);

        m_encoder.setPosition(0.0);
    }

    @Override
    public void periodic() {

        currentPos = m_encoder.getPosition();

        m_atSetPoint = Math.abs(currentPos - m_setpoint) < ElevatorConstants.posTolerance;

        if(!m_isManual)
        {
        // if (isHomed) {
            m_closedLoopController.setReference(m_setpoint, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, Constants.ElevatorConstants.kAF);
        // }
        }

        // Update SmartDashboard
        updateTelemetry();
    }

    public void stopMotors() {
        m_primaryMotor.set(0);
    }

    public boolean isAtSetPoint(double targetHeightInches) {
        // Check if the elevator is within a small tolerance of the target height
        return m_atSetPoint;
    }

    public void setPositionInches(double inches) {
        // if (!isHomed && inches > 0) {
        //     System.out.println("Warning: Elevator not homed! Home first before moving to positions.");
        //     return;
        // }

        m_isManual = false;

        System.out.print("  setPositionInches: "); System.out.println(inches);

        m_setpoint = MathUtil.clamp(
            inches,
            ElevatorConstants.minPos,
            ElevatorConstants.maxPos
        );

        System.out.print("    setPoint: "); System.out.println(m_setpoint);
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("elevator/height", getHeightInches());
        SmartDashboard.putNumber("elevator/target", m_setpoint);
        SmartDashboard.putBoolean("elevator/is_homed", m_isHomed);
        // SmartDashboard.putString("elevator/state", currentTarget.toString());
        SmartDashboard.putNumber("elevator/motor_current", m_primaryMotor.getOutputCurrent());
        // SmartDashboard.putNumber("elevator/velocity", currentState.velocity);
    }

    public double getHeightInches() {
        return m_encoder.getPosition();
    }

    public boolean isStalled() {
        return m_primaryMotor.getWarnings().stall;
    }
    
    public boolean isAtPosition(ElevatorPosition position) {
        return m_atSetPoint = Math.abs(currentPos - position.positionInches) < ElevatorConstants.posTolerance;
    }

    public boolean isM_isHomed() {
        return m_isHomed;
    }

    public ElevatorPosition getM_currentTarget() {
        return m_currentTarget;
    }

    public void setManualPower(double power) {
        // Disable PID control when in manual mode
        m_isManual = true;

        // if (getHeightInches() >= ElevatorConstants.maxPos && power > 0) {
        //     power = 0;
        // }
        
        // if (getHeightInches() <= ElevatorConstants.minPos && power < 0) {
        //     power = 0;
        // }
        
        m_primaryMotor.set(MathUtil.clamp(power, -ElevatorConstants.maxOutput, ElevatorConstants.maxOutput));
    }
}