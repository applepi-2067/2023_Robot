package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeExtensionMotor;
import frc.robot.utils.Util;

public class ZeroTopRightIntake extends CommandBase {
    private IntakeExtensionMotor intakeExtensionMotor;
    private final double SPEED = -1.0;
    private final double zeroMotorCurrent = 10.0; //AMPS

    private double averaged_current = 0.0;
    private final double AVG_GAIN = 0.5;
  
    public ZeroTopRightIntake() {
     intakeExtensionMotor = IntakeExtensionMotor.getInstance();
     addRequirements(intakeExtensionMotor);
    }
  
    @Override
    public void initialize() {
        averaged_current = 0.0;
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      intakeExtensionMotor.setSpeedRight(SPEED);
      
      averaged_current = Util.runningAverage(intakeExtensionMotor.getRightMotorCurrent(), averaged_current,AVG_GAIN );
      SmartDashboard.putNumber("rightCurrent", averaged_current);
    }
  
    @Override
    public void end(boolean interrupted) {
        intakeExtensionMotor.setSpeedRight(0.0);
        if (!interrupted) {
            intakeExtensionMotor.zeroRightEncoder();
        }
        SmartDashboard.putNumber("rightCurrent", 0.0);
    }
    

    @Override
    public boolean isFinished() {
      return (averaged_current >= zeroMotorCurrent);
    }
  }
