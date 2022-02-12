// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  WPI_TalonSRX leftMaster = new WPI_TalonSRX(Constants.LeftMasterID);
  WPI_TalonSRX rightMaster = new WPI_TalonSRX(Constants.RightMasterID);
  WPI_VictorSPX leftSlave = new WPI_VictorSPX(Constants.LeftSlaveID);
  WPI_VictorSPX rightSlave = new WPI_VictorSPX(Constants.RightSlaveID);

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    // invert setup
    leftMaster.setInverted(false);
    leftSlave.setInverted(InvertType.FollowMaster);
    rightMaster.setInverted(true);
    rightSlave.setInverted(InvertType.FollowMaster);

    leftMaster.clearStickyFaults(10);
    rightMaster.clearStickyFaults(10);

    leftMaster.configOpenloopRamp(1);
    rightMaster.configOpenloopRamp(1);
    leftSlave.configOpenloopRamp(1);
    rightSlave.configOpenloopRamp(1);

    leftMaster.setNeutralMode(NeutralMode.Coast);
    leftSlave.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);
    rightSlave.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  
  

  /** Makes Robot Go Brrrrrrr */
  public void DriveWithJoystick(Joystick driverJoystick) {
    double joy_y = driverJoystick.getRawAxis(Constants.joystickX)*Constants.speedX;
    double joy_x = driverJoystick.getRawAxis(Constants.joystickY)*Constants.speedY;
    double threshold = .2;
    double leftMotorOutput;
    double rightMotorOutput;

    double xSpeed = MathUtil.clamp(joy_x, -1.0, 1.0);
    xSpeed = applyDeadband(joy_x, threshold);
    double zRotation = MathUtil.clamp(joy_y, -1.0, 1.0);
    zRotation = applyDeadband(joy_y, threshold);
    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);
    if (xSpeed >= 0.0) {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      } else {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      } else {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      }
    }
    leftMaster.set(MathUtil.clamp(leftMotorOutput, -1.0, 1.0) * 1);
    rightMaster.set(MathUtil.clamp(rightMotorOutput, -1.0, 1.0) * 1);
  }
  
  /** Applys a Deadband */
  public double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  

  /**sets neutral mode
   * 
   * @param mode 1 = coast, 0 = brake
   */
  public void SetMotorMode(double mode){
    if(mode==1){

    leftMaster.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);
    leftSlave.setNeutralMode(NeutralMode.Coast);
    rightSlave.setNeutralMode(NeutralMode.Coast);

    }else if(mode==0){

    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftSlave.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);
    }
  }


  
  
}
