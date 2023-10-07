// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoDrive extends CommandBase {
  private DriveTrain dt;
  private double setpoint;

  /** Creates a new AutoDrive. */
  public AutoDrive(DriveTrain dt, double setpoint) {
    this.dt = dt;
    this.setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.tankDrive(0.0,0.0);
    dt.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftPowerRaw = 1.0;

    double rightPowerRaw = 1.0;
    
    dt.tankDrive(leftPowerRaw*0.3, rightPowerRaw*0.3);
    
    double meters = (dt.getTicks()*Math.PI)/4096;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0.0,0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (meters >= setpoint){
      return true;
    }
    else {
      return false;
    }
}
