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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.Constants.CoralConstants;

public class AlgaeSubsystem  extends SubsystemBase {
    private final SparkFlex m_motorAngle;
    private final SparkFlex m_motorPowerLeader;
    private final SparkFlex m_motorPowerFollower;
    private final RelativeEncoder m_encoderpower;
    private final AbsoluteEncoder m_encoderangle;
    private final SparkClosedLoopController m_closedLoopControllerPower;
    private final SparkClosedLoopController m_closedLoopControllerAngle;

    // private boolean m_atSetPointPower = true;
    private boolean m_atSetPointAngle = true;
    private boolean m_isManualAngle = true;
    private boolean m_isManualPower = true;

    private SparkFlexConfig m_configPowerLeader = new SparkFlexConfig();
    private SparkFlexConfig m_configPowerFollower = new SparkFlexConfig();
    private SparkFlexConfig m_configAngle = new SparkFlexConfig();

    // private double m_setPointPower = 0.0;
    private double m_currentVelocityPower;
    private double m_currentPositionPower;
    private double m_currentCurrentPower;  

    private double m_setPointAngle = 0.0;
    private double m_currentVelocityAngle;
    private double m_currentPositionAngle;
    private double m_currentCurrentAngle;

    public enum anglePosition {
        UP(Constants.AlgaeArmConstants.positionMax),
        POSITION_1(Constants.AlgaeArmConstants.positionProcessor),
        POSITION_2(Constants.AlgaeArmConstants.positionFloor),
        DOWN(Constants.AlgaeArmConstants.positionMin);

        private final double positionDegrees;

        anglePosition(double positionDegrees) {
                this.positionDegrees = positionDegrees;
        }
    }

        // Constructor
    public AlgaeSubsystem() {
            m_motorPowerLeader = new SparkFlex(Constants.AlgaeArmConstants.powerLeaderID, MotorType.kBrushless);
            m_motorPowerFollower = new SparkFlex(Constants.AlgaeArmConstants.powerFollowerID, MotorType.kBrushless);
            m_motorAngle = new SparkFlex(Constants.AlgaeArmConstants.angleID, MotorType.kBrushless);
        
            m_encoderpower = m_motorPowerLeader.getEncoder();
            m_encoderangle = m_motorAngle.getAbsoluteEncoder();

            m_closedLoopControllerPower = m_motorPowerLeader.getClosedLoopController();
            m_closedLoopControllerAngle = m_motorAngle.getClosedLoopController();

            configureMotors();
    }

       // Initialize motor settings
    private void configureMotors() {
        m_configPowerLeader.idleMode(IdleMode.kBrake)
                    .inverted(false)
                    .smartCurrentLimit(Constants.AlgaeArmConstants.MaxCurrentLimitPower)
                    .voltageCompensation(12.0);

        m_configPowerFollower.idleMode(IdleMode.kBrake)
                    .inverted(false)
                    .smartCurrentLimit(Constants.AlgaeArmConstants.MaxCurrentLimitPower)
                    .voltageCompensation(12.0);                    

        m_configAngle.idleMode(IdleMode.kBrake)
                    .inverted(true)
                    .smartCurrentLimit(Constants.AlgaeArmConstants.MaxCurrentLimitAngle)
                    .voltageCompensation(12.0);

        m_configPowerLeader.encoder.positionConversionFactor(Constants.AlgaeArmConstants.countsPerDegreePower)
                    .velocityConversionFactor(Constants.AlgaeArmConstants.countsPerDegreePower/60);

        m_configPowerFollower.encoder.positionConversionFactor(Constants.AlgaeArmConstants.countsPerDegreePower)
                    .velocityConversionFactor(Constants.AlgaeArmConstants.countsPerDegreePower/60);                    

        m_configAngle.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);            

        m_configAngle.absoluteEncoder.zeroCentered(true)
                                     .positionConversionFactor(Constants.AlgaeArmConstants.countsPerDegreeAngle)
                                     .velocityConversionFactor(Constants.AlgaeArmConstants.countsPerDegreeAngle/60);

        m_configAngle.softLimit.forwardSoftLimitEnabled(true).forwardSoftLimit(Constants.AlgaeArmConstants.positionMax)
                    .reverseSoftLimitEnabled(true).reverseSoftLimit(Constants.AlgaeArmConstants.positionMin);

        m_configPowerLeader.softLimit.forwardSoftLimitEnabled(false)
                    .reverseSoftLimitEnabled(false);

        m_configPowerFollower.softLimit.forwardSoftLimitEnabled(false)
                    .reverseSoftLimitEnabled(false);

        // Special Leader settings
        // m_configPowerLeader.closedLoop.pid(Constants.AlgaeArmConstants.kP_power, Constants.AlgaeArmConstants.kI_power, Constants.AlgaeArmConstants.kD_power)
        //                         .outputRange(-1.0, 1.0);

        m_configAngle.closedLoop.maxMotion.maxVelocity(Constants.AlgaeArmConstants.maxVelocity)
                                        .maxAcceleration(Constants.AlgaeArmConstants.maxAcceleration)
                                        .allowedClosedLoopError(Constants.AlgaeArmConstants.posTolerance);  
                                        
        m_configAngle.closedLoop.pid(Constants.AlgaeArmConstants.kP_angle, Constants.AlgaeArmConstants.kI_angle, Constants.AlgaeArmConstants.kD_angle)
                                .outputRange(-1.0, 1.0);

        // Special follower settings
        m_configPowerFollower.follow(m_motorPowerLeader, true);                                  

        // Send setting to motors
        m_motorPowerLeader.configure(m_configPowerLeader, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motorPowerFollower.configure(m_configPowerFollower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motorAngle.configure(m_configAngle, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {

        // Gather Telemetry
        m_currentVelocityPower = m_encoderpower.getVelocity();
        m_currentPositionPower = m_encoderpower.getPosition();
        m_currentCurrentPower  = m_motorPowerLeader.getOutputCurrent();

        m_currentVelocityAngle = m_encoderangle.getVelocity();
        m_currentPositionAngle = m_encoderangle.getPosition();
        m_currentCurrentAngle  = m_motorAngle.getOutputCurrent();

        // m_atSetPointPower = Math.abs(m_currentPositionPower - m_setPointPower) < AlgaeArmConstants.posTolerance;
        m_atSetPointAngle = Math.abs(m_currentPositionAngle - m_setPointAngle) < AlgaeArmConstants.posTolerance;

        // Update SmartDashboard
        updateTelemetry();

        // if(!m_isManualPower)
        // {
        //     m_closedLoopControllerPower.setReference(m_setPointPower, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        // }

        if(!m_isManualAngle)
        {
            m_closedLoopControllerAngle.setReference(m_setPointAngle, 
                                                    SparkBase.ControlType.kMAXMotionPositionControl,
                                                    ClosedLoopSlot.kSlot0, 
                                                    Constants.AlgaeArmConstants.kAF_angle * Math.cos(Units.degreesToRadians(m_currentPositionAngle)));
        }
    }

    private void updateTelemetry() {
        SmartDashboard.putBoolean("algae/power/is_manual", m_isManualPower);
        SmartDashboard.putNumber ("algae/power/position", m_currentPositionPower);
        // SmartDashboard.putNumber("algae/launch/set_point", m_setPointPower);
        SmartDashboard.putNumber ("algae/power/velocity", m_currentVelocityPower);
        SmartDashboard.putNumber ("algae/power/motor_current", m_currentCurrentPower);
        
        SmartDashboard.putBoolean("algae/angle/is_manual", m_isManualAngle);
        SmartDashboard.putNumber ("algae/angle/position", m_currentPositionAngle);
        SmartDashboard.putNumber ("algae/angle/set_point", m_setPointAngle);
        SmartDashboard.putNumber ("algae/angle/velocity", m_currentVelocityAngle);
        SmartDashboard.putNumber ("algae/angle/motor_current", m_currentCurrentAngle);
    }

    public void stopMotors() {
        m_motorPowerLeader.set(0);
        m_motorPowerFollower.set(0);
        m_motorAngle.set(0);
    }

    // public boolean isAtSetPointPower() {
    //     return m_atSetPointPower;
    // }

    public boolean isAtSetPointAngle() {
        return m_atSetPointAngle;
    }

    // public void setPositionPower(double degree) {

    //     m_isManualPower = false;

    //     m_setPointPower = degree;
    // }

    public void setPositionAngle(double degree) {

        m_isManualAngle = false;

        m_setPointAngle = MathUtil.clamp(
            degree,
            CoralConstants.positionMin,
            CoralConstants.positionMax
        );
    }

    public void setManualPowerPower(double power) {
        SmartDashboard.putNumber("algae/power/cmd_vel", power);
        // Disable PID control when in manual mode
        if(Math.abs(power) > 0.01)
        {
            m_isManualPower = true;
            m_motorPowerLeader.set(MathUtil.clamp(power, -AlgaeArmConstants.maxOutput, AlgaeArmConstants.maxOutput));
        }
        else
        {
            // m_setPointPower = m_currentPositionPower;
            m_isManualPower = false;
            m_motorPowerLeader.set(MathUtil.clamp(power, -CoralConstants.maxOutput, CoralConstants.maxOutput));
         }
    }

    public void setManualPowerAngle(double power) {
        SmartDashboard.putNumber("algae/angle/cmd_vel", power);
        // Disable PID control when in manual mode
        m_isManualAngle = true;
        
        m_motorAngle.set(MathUtil.clamp(power + AlgaeArmConstants.kAF_angle * Math.cos(Units.degreesToRadians(m_currentPositionAngle)), -AlgaeArmConstants.maxOutput, AlgaeArmConstants.maxOutput));
    }
}
