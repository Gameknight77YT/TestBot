// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {
  private TalonSRX leftMaster = new TalonSRX(Constants.LeftMasterID);
  private TalonSRX rightMaster = new TalonSRX(Constants.RightMasterID);
  private VictorSPX leftSlave = new VictorSPX(Constants.LeftSlaveID);
  private VictorSPX rightSlave = new VictorSPX(Constants.RightSlaveID);

  private DifferentialDrive drive = new DifferentialDrive(setLeft(), setRight());
  

  private SendableChooser<Boolean> chooser; 

  
  SlewRateLimiter limiter = new SlewRateLimiter(0.3);

  /** Creates a new DriveTrain. */
  public DriveTrain(SendableChooser<Boolean> chooser) {
    this.chooser = chooser;
    leftMaster.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightSlave.configFactoryDefault();
    leftSlave.configFactoryDefault();
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    // invert setup
    leftMaster.setInverted(false);
    leftSlave.setInverted(InvertType.FollowMaster);
    rightMaster.setInverted(true);
    rightSlave.setInverted(InvertType.FollowMaster);

    leftMaster.clearStickyFaults(10);
    rightMaster.clearStickyFaults(10);

    leftMaster.configOpenloopRamp(.1);
    rightMaster.configOpenloopRamp(.1);
    leftSlave.configOpenloopRamp(.1);
    rightSlave.configOpenloopRamp(.1);

    leftMaster.setNeutralMode(NeutralMode.Coast);
    leftSlave.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);
    rightSlave.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  private DoubleConsumer setLeft() {
    return new DoubleConsumer() {

      @Override
      public void accept(double value) {
        leftMaster.set(ControlMode.PercentOutput, value);
      }
      
    };
  }

  private DoubleConsumer setRight() {
    return new DoubleConsumer() {

      @Override
      public void accept(double value) {
        rightMaster.set(ControlMode.PercentOutput, value);
      }
      
    };
  }

  
  

  /** Makes Robot Go Brrrrrrr */
  public void DriveWithJoystick(Joystick driverJoystick) {
    boolean safe = chooser.getSelected();
    double joy_x = safe ? 
      driverJoystick.getX()*-(Constants.speedX - .2)
      : driverJoystick.getX()*-Constants.speedX;
    double joy_y = safe ? 
      driverJoystick.getY()*-(Constants.speedY - .2):
      driverJoystick.getY()*-Constants.speedY;

    drive.arcadeDrive(joy_y, joy_x);
    drive.feed();
    /*double threshold = .2;
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
    leftMotorOutput = MathUtil.clamp(leftMotorOutput, -1.0, 1.0) * 1;
    rightMotorOutput = MathUtil.clamp(rightMotorOutput, -1.0, 1.0) * 1;

    if(safe){
      leftMotorOutput = limiter.calculate(leftMotorOutput);
      rightMotorOutput = limiter.calculate(rightMotorOutput);
    }
    

    leftMaster.set(ControlMode.PercentOutput, leftMotorOutput);
    rightMaster.set(ControlMode.PercentOutput, rightMotorOutput);*/
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
