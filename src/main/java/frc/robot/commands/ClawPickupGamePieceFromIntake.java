// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IK.RobotRelativeIK;
import frc.robot.commands.claw.*;

public class ClawPickupGamePieceFromIntake extends SequentialCommandGroup {
  /** Creates a new AcquireGamePieceInClaw. */
  public ClawPickupGamePieceFromIntake() {

    addCommands(
      //Sequence starts with claw open & above gap between the back of the intake and the waist,
      // waiting for a game piece to be brought into the robot

      //Lower the open claw while running the wheels inward
      new ClawOpen(),
      new SetClawBeltSpeed(1.0),
      new RobotRelativeIK(Constants.IKPositions.ACQUIRING_PIECE_FROM_INTAKE),

      //Close the claw
      new ClawClose(),

      //Stop the wheels when the game piece is sensed in the gripper & raise the arm to the stowed location
      new WaitForGamePieceInClaw(),
      new SetClawBeltSpeed(0),
      new RobotRelativeIK(Constants.IKPositions.STOWED_WITH_GAME_PIECE_CLEAR_OF_INTAKE)
    );
  }
}
