// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.algae.AngleVelocity;
import frc.robot.commands.algae.AnglePosition;
import frc.robot.commands.algae.IntakeAlgae;
import frc.robot.commands.algae.LaunchAlgae;
import frc.robot.commands.autos.AutoIntakeAlgae;
import frc.robot.commands.coral.ArmPosition;
import frc.robot.commands.coral.ArmVelocity;
import frc.robot.commands.coral.IntakeCoral;
import frc.robot.commands.coral.LaunchCoral;
import frc.robot.commands.elevator.ElevatorHome;
import frc.robot.commands.elevator.ElevatorPosition;
import frc.robot.commands.elevator.ElevatorPositionAfterHome;
import frc.robot.commands.elevator.ElevatorVelocity;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.targeting.TargetingSubsystem;

import java.io.File;
import java.lang.invoke.ConstantCallSite;
import java.security.AlgorithmConstraints;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  public double throttleTrans;
  public double throttleAngle;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandJoystick m_driverSwitch = new CommandJoystick(0);
  final         CommandXboxController m_driver2Xbox = new CommandXboxController(1);
  final         CommandJoystick       m_buttonBox = new CommandJoystick(2);
  // The robot's subsystems and commands are defined here...
  final TargetingSubsystem    m_targeting = new TargetingSubsystem();
  
  final SwerveSubsystem       m_drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
  private final ElevatorSubsystem     m_elevator = new ElevatorSubsystem();
  private final AlgaeSubsystem m_algae = new AlgaeSubsystem();
  private final CoralSubsystem m_coral = new CoralSubsystem();
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
                                                                () -> m_driverSwitch.getRawAxis(1) * -1 * throttleTrans ,
                                                                () -> m_driverSwitch.getRawAxis(0) * -1 * throttleTrans)
                                                            .withControllerRotationAxis(() -> m_driverSwitch.getRawAxis(2) * -0.7 * throttleAngle)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true)
                                                            .alignWhile(m_driverSwitch.button(4))
                                                            .align(() -> m_targeting.getPoseForNearestTargetInRobotFrame().orElse(null));

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> m_driverSwitch.getRawAxis(2) * -1,
                                                                                             () -> m_driverSwitch.getRawAxis(3) * -1)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(true)
                                                             .alignWhile(m_driverSwitch.button(4))
                                                             .align(() -> m_targeting.getPoseForNearestTargetInRobotFrame().orElse(null));


  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
                                                                        () -> -m_driverSwitch.getRawAxis(1),
                                                                        () -> -m_driverSwitch.getRawAxis(0))
                                                                    .withControllerRotationAxis(() -> m_driverSwitch.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                m_driverSwitch.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                m_driverSwitch.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);



  // Create SmartDashboard chooser for autonomous and teleop routines
  private final SendableChooser<Command> m_chooserTeleop = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    throttleTrans = 1.0;
    throttleAngle = 1.0;
    
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle      = m_drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = m_drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = m_drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = m_drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = m_drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = m_drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = m_drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    // Setup SmartDashboard chooser options
    m_chooserTeleop.setDefaultOption("driveFieldOrientedDirectAngle", driveFieldOrientedDirectAngle);
    m_chooserTeleop.addOption("driveFieldOrientedAnglularVelocity", driveFieldOrientedAnglularVelocity);
    m_chooserTeleop.addOption("driveRobotOrientedAngularVelocity", driveRobotOrientedAngularVelocity);
    SmartDashboard.putData("Teleop Mode", m_chooserTeleop);

    if (RobotBase.isSimulation())
    {
      m_drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      m_drivebase.setDefaultCommand(getTeleopDriveCommand());
    }

    if (Robot.isSimulation())
    {
      m_driverSwitch.button(8).onTrue(Commands.runOnce(() -> m_drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      m_driverSwitch.button(1).whileTrue(m_drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      m_driverSwitch.button(3).whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly());
      m_driverSwitch.button(4).whileTrue(m_drivebase.driveToDistanceCommand(1.0, 0.2));
      m_driverSwitch.button(8).onTrue((Commands.runOnce(m_drivebase::zeroGyroWithAlliance)));
      m_driverSwitch.button(7).whileTrue(m_drivebase.centerModulesCommand());
      m_driverSwitch.button(5).onTrue(Commands.none());
      m_driverSwitch.button(6).onTrue(Commands.none());
    } else
    {
      // BUTTON CONTROLS
      m_driverSwitch.button(2).onTrue((Commands.runOnce(m_drivebase::zeroGyroWithAlliance)));
      // m_driverXbox.leftBumper().whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly());
      m_driverSwitch.button(3).onTrue(Commands.runOnce(m_drivebase::addFakeVisionReading));
      m_driverSwitch.button(10).whileTrue(new AutoIntakeAlgae(m_drivebase, m_targeting, 0.3, 0.1, 0.1, m_algae, Constants.AlgaeArmConstants.powerIntake));

      m_driverSwitch.button(5).whileTrue(new IntakeCoral(m_coral, Constants.CoralConstants.powerIntake));
      m_driverSwitch.button(7).onTrue(new LaunchCoral(m_coral, Constants.CoralConstants.powerLaunch, m_elevator, Constants.ElevatorConstants.positionDown));

      m_driverSwitch.button(6).whileTrue(new IntakeAlgae(m_algae, Constants.AlgaeArmConstants.powerIntake));
      m_driverSwitch.button(8).whileTrue(new LaunchAlgae(m_algae, Constants.AlgaeArmConstants.powerLaunch));


      m_driver2Xbox.button(7).onTrue(new ElevatorHome(m_elevator));
      m_driver2Xbox.button(5).whileTrue(new ElevatorVelocity(m_elevator, () -> m_driver2Xbox.getLeftY() * -0.3));
      m_driver2Xbox.button(5).whileTrue(new ArmVelocity (m_coral, () -> m_driver2Xbox.getRightY() * -0.3));
      m_driver2Xbox.button(5).whileTrue(new AngleVelocity (m_algae, () -> m_driver2Xbox.getRightX() * -0.3));

      //  Coral Positions
        // L4
        m_driver2Xbox.button(4).onTrue(new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionL4, m_coral, Constants.CoralConstants.positionReef, m_algae, Constants.AlgaeArmConstants.positionUp));
        m_buttonBox.button(2).onTrue(new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionL4, m_coral, Constants.CoralConstants.positionReef, m_algae, Constants.AlgaeArmConstants.positionUp));
        // L3 Positions
        m_driver2Xbox.button(3).onTrue(new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionL3, m_coral, Constants.CoralConstants.positionReef, m_algae, Constants.AlgaeArmConstants.positionUp));
        m_buttonBox.button(3).onTrue(new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionL3, m_coral, Constants.CoralConstants.positionReef, m_algae, Constants.AlgaeArmConstants.positionUp));
        // L2 Position
        m_driver2Xbox.button(2).onTrue(new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionL2, m_coral, Constants.CoralConstants.positionReef, m_algae, Constants.AlgaeArmConstants.positionUp));
        m_buttonBox.button(10).onTrue(new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionL2, m_coral, Constants.CoralConstants.positionReef, m_algae, Constants.AlgaeArmConstants.positionUp));
        // L1 Position
        m_driver2Xbox.button(1).onTrue(new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionL1, m_coral, Constants.CoralConstants.positionReef, m_algae, Constants.AlgaeArmConstants.positionUp));
        m_buttonBox.button(11).onTrue(new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionL1, m_coral, Constants.CoralConstants.positionReef, m_algae, Constants.AlgaeArmConstants.positionUp));
        // Coral Station Position
        m_driver2Xbox.button(6).onTrue(new ElevatorPositionAfterHome(m_elevator, Constants.ElevatorConstants.positionDown, m_coral, Constants.CoralConstants.positionStation, m_algae, Constants.AlgaeArmConstants.positionUp));
        m_buttonBox.button(5).onTrue(new ElevatorPositionAfterHome(m_elevator, Constants.ElevatorConstants.positionDown, m_coral, Constants.CoralConstants.positionStation, m_algae, Constants.AlgaeArmConstants.positionUp));

      // Algae Positions
        // A3
        m_driver2Xbox.pov(0).onTrue(new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionA3, m_coral, Constants.CoralConstants.positionUp, m_algae, Constants.AlgaeArmConstants.positionReef));
        // A2
        m_driver2Xbox.pov(90).onTrue(new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionA2, m_coral, Constants.CoralConstants.positionUp, m_algae, Constants.AlgaeArmConstants.positionReef));
        // Processor
        m_driver2Xbox.pov(270).onTrue(new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionProcessor, m_coral, Constants.CoralConstants.positionUp, m_algae, Constants.AlgaeArmConstants.positionProcessor));
        // Floor
        m_driver2Xbox.pov(180).onTrue(new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionFloor, m_coral, Constants.CoralConstants.positionUp, m_algae, Constants.AlgaeArmConstants.positionFloor));
        // net
        m_driver2Xbox.button(8).onTrue(new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionNet, m_coral, Constants.CoralConstants.positionUp, m_algae, Constants.AlgaeArmConstants.positionNet ));
        // lollipop
        m_driver2Xbox.button(9).onTrue(new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionLollipop, m_coral, Constants.CoralConstants.positionUp, m_algae, Constants.AlgaeArmConstants.positionLollipop));

      // m_driver2Xbox.button(8).onTrue(hook down); 
   // m_driver2Xbox.button(9).onTrue(coral intake); 
   // m_driver2Xbox.button(10).onTrue(coral outtake); 
   // m_driver2Xbox.button(11).onTrue(algae intake); 
   // m_driver2Xbox.button(12).onTrue(algae outtake); 
   // m_driver2Xbox.button(13).onTrue(side algae down); 
   // m_driver2Xbox.button(14).onTrue(side algae up); 
   // m_driver2Xbox.button(15).onTrue(side algae intake); 
   // m_driver2Xbox.button(16).onTrue(side algae outtake); 
   // m_driver2Xbox.button(17).onTrue(hook up?); 
   // m_driver2Xbox.button(18).onTrue(shoot algae full speed and set angle to the net (possibly using april tags to find the right angle depending on position)); 

      NamedCommands.registerCommand("LowerToProcessor", new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionProcessor, m_coral, Constants.CoralConstants.positionUp, m_algae, Constants.AlgaeArmConstants.positionProcessor));
      NamedCommands.registerCommand("RaiseToHighAlgae", new ElevatorPosition(m_elevator, Constants.ElevatorConstants.positionA3, m_coral, Constants.CoralConstants.positionUp, m_algae, Constants.AlgaeArmConstants.positionReef));
      NamedCommands.registerCommand("IntakeAlgae", new AutoIntakeAlgae(m_drivebase, m_targeting, 0.2, 0.1, 0.1, m_algae, Constants.AlgaeArmConstants.powerIntake));

    }

  }

  public double calcThrottle()
  {
    return Constants.minThrottle + (Constants.maxThrottle - Constants.minThrottle) * m_elevator.getElevatorThrottle();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return m_drivebase.getAutonomousCommand("New Auto");
  }

  /**
 * Use this to pass the teleop command to the main {@link Robot} class.
 *
 * @return the command to run in teleop
 */
  public Command getTeleopDriveCommand() {
    return m_chooserTeleop.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    m_drivebase.setMotorBrake(brake);
  }
}
