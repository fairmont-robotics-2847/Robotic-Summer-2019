/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {
  WPI_TalonSRX _rghtFront = new WPI_TalonSRX(1);
    WPI_VictorSPX _rghtFollower = new WPI_VictorSPX(1);
    WPI_TalonSRX _leftFront = new WPI_TalonSRX(0);
    WPI_VictorSPX _leftFollower = new WPI_VictorSPX(0);
  
  DifferentialDrive _diffDrive = new DifferentialDrive(_leftFront, _rghtFront);
  
  Joystick _joystick = new Joystick(0);

   DifferentialDrive m_myRobot;
   Joystick m_leftStick;
   Joystick m_rightStick;

  @Override
  public void robotInit() {
    m_myRobot = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
    
      //factory default values 
        _rghtFront.configFactoryDefault(1);
        _rghtFollower.configFactoryDefault(0);
        _leftFront.configFactoryDefault(1);
        _leftFollower.configFactoryDefault(0);
    
    //set up followers where does one find this?
        _rghtFollower.follow(_rghtFront);
        _leftFollower.follow(_leftFront);
    
       /* flip values so robot moves forward when stick-forward/LEDs-green */
        _rghtFront.setInverted(true);
        _leftFront.setInverted(false);
        _rghtFollower.setInverted(InvertType.FollowMaster);
        _leftFollower.setInverted(InvertType.FollowMaster);

        /*
          adjust sensor phase so sensor moves positive when Talon LEDs are green
         */
        _rghtFront.setSensorPhase(true);
        _leftFront.setSensorPhase(true);

        /*
         * WPI drivetrain classes defaultly assume left and right are opposite. call
         * this so we can apply + to both sides when moving forward. DO NOT CHANGE
         */
        _diffDrive.setRightSideInverted(false);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());

        // need gamepad stick values 
        // Final scalers are speed values
        double forw = -1 * _joystick.getRawAxis(1) * 0.5; 
        double turn = +1 * _joystick.getRawAxis(2) * 0.5; 

        /* deadband gamepad 10% */
        if (Math.abs(forw) < 0.10) {
            forw = 0;
        }
        if (Math.abs(turn) < 0.10) {
            turn = 0;
        }

        _diffDrive.arcadeDrive(forw, turn);
  }
}