package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.HangerConstants;
import frc.robot.subsystems.AlgaeSubsystem.anglePosition;

public class Hangersubsystem extends SubsystemBase {
        private final SparkMax m_primaryMotor;
        private final SparkMax m_followerMotor;
        private final AbsoluteEncoder m_AbsoluteEncoder;
        private final SparkClosedLoopController m_closedLoopController;
        private boolean m_isManual = false;
        private boolean m_atSetPoint = false;
        private boolean m_isStalled = false;
        private double m_setpoint;
        private double m_currentVelocity;
        private double m_currentPosition;
        private double m_currentCurrent;
        private SparkMaxConfig m_leaderConfig = new SparkMaxConfig();
        private SparkMaxConfig m_followerConfig = new SparkMaxConfig();

 public Hangersubsystem() {
        m_primaryMotor = new SparkMax(HangerConstants.RightHangerID, MotorType.kBrushless);
        m_followerMotor = new SparkMax(HangerConstants.LeftHangerID, MotorType.kBrushless);

        m_AbsoluteEncoder = m_primaryMotor.getAbsoluteEncoder();
        m_closedLoopController = m_primaryMotor.getClosedLoopController();
        
        configureMotors();
    }

    private void configureMotors() {

        // Settings
        m_leaderConfig.idleMode(IdleMode.kBrake)
                      .inverted(true)
                      .smartCurrentLimit((int) Constants.HangerConstants.MaxCurrentLimit)
                      .voltageCompensation(12.0)
                      .openLoopRampRate(HangerConstants.HangRampRate);

        m_followerConfig.idleMode(IdleMode.kBrake)
                        .inverted(true)
                        .smartCurrentLimit((int) Constants.HangerConstants.MaxCurrentLimit)
                        .voltageCompensation(12.0)
                        .openLoopRampRate(HangerConstants.HangRampRate);   

        m_leaderConfig.absoluteEncoder.positionConversionFactor(HangerConstants.DegreesPerRevolution)
                              .velocityConversionFactor(HangerConstants.DegreesPerRevolution/60);

        
        m_leaderConfig.softLimit.forwardSoftLimitEnabled(true).forwardSoftLimit(Constants.HangerConstants.positionMax)
                                .reverseSoftLimitEnabled(true).reverseSoftLimit(Constants.HangerConstants.positionMin);

        m_followerConfig.softLimit.forwardSoftLimitEnabled(false)
                                  .reverseSoftLimitEnabled(false);

        // Special Leader settings
        m_leaderConfig.closedLoop.maxMotion.maxVelocity(Constants.HangerConstants.maxVelocity)
                                        .maxAcceleration(Constants.HangerConstants.maxAcceleration)
                                        .allowedClosedLoopError(Constants.ElevatorConstants.posTolerance);  
                                        
        m_leaderConfig.closedLoop.pid(Constants.HangerConstants.kP, Constants.HangerConstants.kI, Constants.HangerConstants.kD)
                                 .iZone(HangerConstants.kIz)
                              .outputRange(-1.0, 1.0);

        // Special follower settings
        m_followerConfig.follow(m_primaryMotor, true);  

        // Send setting to motors
        m_primaryMotor.configure(m_leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_followerMotor.configure(m_followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_primaryMotor.getEncoder().setPosition(Constants.HangerConstants.StartingAngle);
    }
  //      @Override
 //   public void periodic() {
 //       // Update telemetry
 //   }
@Override
    public void periodic() {
        m_currentVelocity = m_AbsoluteEncoder.getVelocity();
        m_currentPosition = m_AbsoluteEncoder.getPosition();
        m_currentCurrent  = m_primaryMotor.getOutputCurrent();
        m_isStalled = m_primaryMotor.getWarnings().stall;
        m_atSetPoint = Math.abs(m_currentPosition - m_setpoint) < HangerConstants.AllowableError;

        // Update SmartDashboard
        updateTelemetry();

        if(m_isManual) return;

        //m_closedLoopController.setReference(m_setpoint, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, Constants.ElevatorConstants.kAF, ArbFFUnits.kPercentOut);
    }
    private void updateTelemetry() {
        SmartDashboard.putBoolean("hanger/is_manual", m_isManual);
        SmartDashboard.putNumber("hanger/set_angle", m_setpoint);
        SmartDashboard.putNumber("hanger/velocity", getVelocity());
        SmartDashboard.putNumber("hanger/motor_current", getCurrent());
    }
    public void setPosition(double position, boolean isHang) {
        if (m_isManual) {
            return;
        }
        if (isHang) {
            m_primaryMotor.set(HangerConstants.HangPower);
        } else {
            m_closedLoopController.setReference(position, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
        }
    }
    public void stopMotors() {
        m_primaryMotor.set(0);
    }

    public void setPower(double power) {
        m_primaryMotor.set(power);
    }

    public double getVelocity() {
        return m_currentVelocity;
    }

    public double getCurrent() {
        return m_currentCurrent;
    }

    public boolean isStalled() {
        return m_isStalled;
    }
    
    public boolean isAtSetPoint() {
        return m_atSetPoint;
    }
    
   // public boolean isAtPosition(anglePosition position) {
   //     return Math.abs(m_hangerangle) < HangerConstants.AllowableError;
    


    public boolean enableSoftLimits(Boolean enable) {
        m_leaderConfig.softLimit.forwardSoftLimitEnabled(enable).reverseSoftLimitEnabled(enable);
        m_primaryMotor.configure(m_leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        return true;
    }

   
  //  public void setManualPower(double power) {
        // Disable PID control when in manual mode
   //     m_isManual = true;
  //      
  //      m_primaryMotor.set(MathUtil.clamp(power + HangerConstants.kAF, -HangerConstants.MaxCurrentLimit, HangerConstants.MaxCurrentLimit));
    //}


    
}