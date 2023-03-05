


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.waist.*;
import frc.robot.subsystems.*;
import frc.robot.utils.Transforms;
import frc.robot.utils.Util;
import frc.robot.commands.chargestation.*;
import frc.robot.commands.claw.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.fielddriving.DriveToAbsolutePosition;
import frc.robot.commands.fielddriving.DriveToTargetOffset;
import frc.robot.commands.estop.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.lights.SetLightsColor;
import frc.robot.commands.shoulder.*;
import frc.robot.commands.teleop_auto.DoubleSubstationPieceAcquire;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.commands.auto.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.IK.IKCoordinate;
import frc.robot.commands.IK.RobotRelativeIK;
import frc.robot.commands.arm.*;

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
  private final IntakeExtensionMotor m_IntakeExtensionMotor = IntakeExtensionMotor.getInstance();
  private final IntakeConveyorBelt m_IntakeConveyorBelt = IntakeConveyorBelt.getInstance();
  private final IntakeRoller m_IntakeRoller = IntakeRoller.getInstance();
  private final IntakeConveyorExtension m_IntakeConveyorExtension = IntakeConveyorExtension.getInstance();

  private final Lights m_lights = Lights.getInstance();
  private final ClawBelt m_clawBelt = ClawBelt.getInstance();

  private static DigitalInput m_practiceBotJumper = new DigitalInput(Constants.DiscreteInputs.PBOT_JUMPER_DI);
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
    //Driver Controls

    // Pick up piece from double substation

    m_driverController.povUp().onTrue(new DoubleSubstationPieceAcquire());

    // Light control
    m_driverController.rightTrigger().onTrue(new SetLightsColor(Lights.Color.PURPLE));
    m_driverController.x().onTrue(new SetLightsColor(Lights.Color.PURPLE));
    m_driverController.leftTrigger().onTrue(new SetLightsColor(Lights.Color.YELLOW));
    m_driverController.y().onTrue(new SetLightsColor(Lights.Color.YELLOW));

    m_driverController.back().onTrue(new StopDrivetrain());  // Stop the drivetrain when right stick is pressed in

    //Operator Controls
    m_operatorController.rightStick().onTrue(new StopArmWaistShoulder());  // Stop arm/waist/shoulder when right stick is pressed in

    // m_operatorController.povLeft().onTrue(new SetIntakeExtension(0.025));
  
    // m_operatorController.rightBumper().onTrue(new IntakeConveyorIn(true));
    // m_operatorController.leftBumper().onTrue(new IntakeConveyorIn(false));
    
    // //Intake game piece
    // m_operatorController.rightTrigger().onTrue (new SetIntakeRollerSpeed(1.0));
    // m_operatorController.rightTrigger().onFalse(new SetIntakeRollerSpeed(0.0));
    // m_operatorController.rightTrigger().onTrue(new IntakeConveyorBeltSpeed(-1.0));
    // m_operatorController.rightTrigger().onFalse(new IntakeConveyorBeltSpeed(0.0));
  
    // //Outtake game piece
    // m_operatorController.leftTrigger().onTrue (new SetIntakeRollerSpeed(-1.0));
    // m_operatorController.leftTrigger().onFalse(new SetIntakeRollerSpeed(0.0));
    // m_operatorController.leftTrigger().onTrue(new IntakeConveyorBeltSpeed(1.0));
    // m_operatorController.leftTrigger().onFalse(new IntakeConveyorBeltSpeed(0.0));
  
    m_operatorController.a().onTrue(new ClawOpen());
    m_operatorController.a().onFalse(new ClawClose());
    m_operatorController.povUp().onTrue(new ClawSensorGrab());
    m_operatorController.povLeft().onTrue(new ClawGrabCancel());

    //Arm locations
    m_operatorController.povRight().onTrue(new SetArmExtension(0.005).andThen(new SetShoulderPosition(-55.0))); // stowed/retracted position
    m_operatorController.x().onTrue(new SetShoulderPosition(22).andThen(new SetArmExtension(0.894))); // High cone scoring position
    m_operatorController.povDown().onTrue(new SetShoulderPosition(15).andThen(new SetArmExtension(0.894))); // High cube scoring position
    m_operatorController.b().onTrue(new SetShoulderPosition(10).andThen(new SetArmExtension(0.429))); // Mid scoring position
    m_operatorController.y().onTrue(new SetShoulderPosition(10).andThen(new SetArmExtension(0.18)));  //Get Game Piece from human / feed station

    m_operatorController.rightBumper().onTrue(new SetArmExtension(0.005).andThen(new SetShoulderPosition(-55.0)).andThen(new SetWaistPosition(0)));
    m_operatorController.leftBumper().onTrue(new SetArmExtension(0.005).andThen(new SetShoulderPosition(-55.0)).andThen(new SetWaistPosition(180)));

    //m_operatorController.y().onTrue(new RobotRelativeIK(Constants.IKPositions.HIGH_SCORING_POSITION));
    //m_operatorController.b().onTrue(new RobotRelativeIK(Constants.IKPositions.MID_SCORING_POSITION));
    //m_operatorController.a().onTrue(new RobotRelativeIK(Constants.IKPositions.LOW_SCORING_POSITION));
    //m_operatorController.povUp().onTrue(new RobotRelativeIK(Constants.IKPositions.ABOVE_INTAKE_BEFORE_ACQUISITION));

    // SmartDashboard.putData("shoulder 0 degrees", new SetShoulderPosition(0));
    // SmartDashboard.putData("shoulder -60 degrees", new SetShoulderPosition(-60));
    // SmartDashboard.putData("waist 0 degrees", new SetWaistPosition(0));
    // SmartDashboard.putData("waist 15 degrees", new SetWaistPosition(15));

    // SmartDashboard.putData("Above intake before acquisition", new RobotRelativeIK(Constants.IKPositions.ABOVE_INTAKE_BEFORE_ACQUISITION));
    // SmartDashboard.putData("Aquiring piece from intake", new RobotRelativeIK(Constants.IKPositions.ACQUIRING_PIECE_FROM_INTAKE));
    // SmartDashboard.putData("Stowed with game piece clear of intake", new RobotRelativeIK(Constants.IKPositions.STOWED_WITH_GAME_PIECE_CLEAR_OF_INTAKE));
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

  @Log
  public static boolean isPracticeBot() {
    return !m_practiceBotJumper.get();
  }

  public void periodic() {
    SmartDashboard.putNumber("Xposition", Util.getIKX(m_arm.getPosition(), m_waist.getPosition(), m_shoulder.getPosition()));
    SmartDashboard.putNumber("Yposition", Util.getIKY(m_arm.getPosition(), m_waist.getPosition(), m_shoulder.getPosition()));
    SmartDashboard.putNumber("Zposition", Util.getIKZ(m_arm.getPosition(), m_waist.getPosition(), m_shoulder.getPosition()));
    
    SmartDashboard.putNumber("Arm Length (m)", m_arm.getPosition() + Constants.IKOffsets.MINIMUM_ARM_LENGTH);
    SmartDashboard.putNumber("Waist Rotation (deg)", m_waist.getPosition());
    SmartDashboard.putNumber("Shoulder Rotation (deg)", m_shoulder.getPosition());
  }
}

