


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.waist.*;
import frc.robot.subsystems.*;
import frc.robot.commands.claw.*;
import frc.robot.commands.estop.*;
import frc.robot.commands.lights.DisableBlinkLights;
import frc.robot.commands.lights.DisableLights;
import frc.robot.commands.lights.SetLightsColor;
import frc.robot.commands.shoulder.*;
import frc.robot.commands.teleop_auto.GroundPickup;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import frc.robot.commands.arm.*;
import frc.robot.commands.waist.ScoringWaistControl;

/**
 * This class is where the bulk of the robot should be declared.
 * Since Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).
 * Instead, the robot structure (including subsystems, commands,
 * and trigger mappings) should be declared here.
 */
public class RobotContainer implements Loggable {
  // Instantiate subsystems, controllers, and commands.
  private final CommandXboxController m_driverController = new CommandXboxController(
      Constants.OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      Constants.OperatorConstants.kOperatorControllerPort);

  private final Drivetrain m_drivetrain = Drivetrain.getInstance();
  private final Waist m_waist = Waist.getInstance();
  private final Shoulder m_shoulder = Shoulder.getInstance();
  private final Vision m_vision = Vision.getInstance();
  private final Arm m_arm = Arm.getInstance();
  private final ClawGrasp m_ClawGrasp = ClawGrasp.getInstance();

  private final Lights m_lights = Lights.getInstance();
  private final ClawBelt m_clawBelt = ClawBelt.getInstance();

  private Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    Logger.configureLoggingAndConfig(this, false);
    m_compressor.enableDigital();
    // Configure the trigger bindings.
    configureBindings();

    // Configure default commands.
    // Set the default drive command to tank drive.
    m_drivetrain.setDefaultCommand(
        Commands.run(
            () -> m_drivetrain.arcadeDrive(
                 -m_driverController.getLeftY(),
               
                -m_driverController.getRightX() / 1.7),
                
            m_drivetrain));
  }

  /**
   * Use this method to define your trigger-> command mappings.
   * Triggers can be created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor
   * with an arbitrary predicate, or via the named factories in 
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses 
   * for {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    /** Driver controls */
    // Light control
    // m_driverController.rightTrigger().onTrue(new SetLightsColor(Lights.Color.PURPLE));
    // m_driverController.x().onTrue(new SetLightsColor(Lights.Color.PURPLE));
    // m_driverController.leftTrigger().onTrue(new SetLightsColor(Lights.Color.YELLOW));
    // m_driverController.y().onTrue(new SetLightsColor(Lights.Color.YELLOW));

    m_driverController.rightBumper().onTrue(new ScoringWaistControl(2.0));
    m_driverController.leftBumper().onTrue(new ScoringWaistControl(-2.0));
    m_driverController.back().onTrue(new StopDrivetrain());  // E-Stop the drivetrain when back button is pressed

    /** Operator Controls */
    // Lights
    m_operatorController.leftTrigger().onTrue(new SetLightsColor(Lights.Color.YELLOW));
    m_operatorController.rightTrigger().onTrue(new SetLightsColor(Lights.Color.PURPLE));

    // Claw
    m_operatorController.a().onTrue(new ClawOpenSpitWaitSuck(0.5));
    m_operatorController.a().onFalse(new SetClawBeltSpeed(() -> {return 1.0;}).andThen(
      new ClawClose()).andThen(
      new WaitCommand(0.4)).andThen(
      new SetClawBeltSpeed(() -> {return 0.0;})).andThen(
      new DisableBlinkLights()).andThen(
      new DisableLights()));

    
    // Arm locations

    // Stow.
    m_operatorController.povRight().onTrue(
      Commands.parallel(
        new SetArmExtension(Constants.Poses.ArmExtensions.RETRACTED),
        new BlockUntilArmLessThan(0.2).andThen(new SetShoulderPosition(Constants.Poses.ShoulderAngles.STOW))
      )
    );

    // Ground pickup.
    m_operatorController.povUp().onTrue(new GroundPickup());

    // Load station pickup.
    m_operatorController.y().onTrue(
      new SetShoulderPosition(Constants.Poses.ShoulderAngles.PIECE_GRAB).andThen(
        new SetArmExtension(Constants.Poses.ArmExtensions.RETRACTED)).alongWith(new ClawSensorGrab())
    );

    // Mid.
    m_operatorController.b().onTrue(
      new SetShoulderPosition(Constants.Poses.ShoulderAngles.MID_CONE).andThen(new SetArmExtension(Constants.Poses.ArmExtensions.MID))
    );
    m_operatorController.povLeft().onTrue(
      new SetShoulderPosition(Constants.Poses.ShoulderAngles.MID_CUBE).andThen(new SetArmExtension(Constants.Poses.ArmExtensions.MID))
    );

    // High.
    m_operatorController.x().onTrue(
      new SetShoulderPosition(Constants.Poses.ShoulderAngles.HIGH_CONE).andThen(new SetArmExtension(Constants.Poses.ArmExtensions.HIGH))
    );
    m_operatorController.povDown().onTrue(
      new SetShoulderPosition(Constants.Poses.ShoulderAngles.HIGH_CUBE).andThen(new SetArmExtension(Constants.Poses.ArmExtensions.HIGH))
    );    

    m_operatorController.rightBumper().onTrue(
      new SetArmExtension(Constants.Poses.ArmExtensions.RETRACTED).asProxy().andThen(
      new SetShoulderPosition(Constants.Poses.ShoulderAngles.ROTATE).asProxy()).andThen(
      new SetWaistPosition(0)).andThen(
      new SetShoulderPosition(Constants.Poses.ShoulderAngles.STOW).asProxy())
    );
    m_operatorController.leftBumper().onTrue(
      new SetArmExtension(Constants.Poses.ArmExtensions.RETRACTED).asProxy().andThen(
      new SetShoulderPosition(Constants.Poses.ShoulderAngles.ROTATE).asProxy()).andThen(
      new SetWaistPosition(180.0)).andThen(
      new SetShoulderPosition(Constants.Poses.ShoulderAngles.STOW).asProxy())
    );
    
    m_operatorController.rightStick().onTrue(new StopArmWaistShoulder());  // Stop arm/waist/shoulder when right stick is pressed in

    // Zero everything
    m_operatorController.back().onTrue(new PanicZeroEverything());
  }

  /**
   * Sets motors to coast or brake mode
   * 
   * @param coastEnabled Set coast enabled if true, otherwise set brake mode
   */
  public void setCoastEnabled(boolean coastEnabled) {
    if (coastEnabled) {
      m_drivetrain.setMotorsCoast();
    } else {
      m_drivetrain.setMotorsBrake();
    }
  }

  public void periodic() {
  }
}
