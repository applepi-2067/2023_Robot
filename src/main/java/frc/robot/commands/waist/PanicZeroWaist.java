// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.waist;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.arm.ZeroArmPosition;
import frc.robot.commands.lights.BlinkLights;
import frc.robot.commands.lights.DisableBlinkLights;
import frc.robot.commands.lights.DisableLights;
import frc.robot.commands.lights.SetLightsColor;
import frc.robot.commands.shoulder.InitShoulderZero;
import frc.robot.commands.shoulder.ZeroShoulderPosition;
import frc.robot.subsystems.Lights.Color;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PanicZeroWaist extends SequentialCommandGroup {
  /** Creates a new PanicZeroWaist. */
  public PanicZeroWaist() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetLightsColor(Color.WHITE),
      new BlinkLights(0.5),
      new ZeroWaistPositionCoarse(),
      new ZeroWaistPosition(),
      new SetWaistPosition(0.0),
      Commands.parallel(
        new InitShoulderZero().andThen(new ZeroShoulderPosition()),
        new ZeroArmPosition().andThen(new SetArmExtension(0.0))
      ),
      new DisableBlinkLights(),
      new DisableLights()
    );
  }
}
