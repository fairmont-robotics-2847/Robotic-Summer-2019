/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  WPI_TalonSRX _rghtFront = new WPI_TalonSRX(1);
  WPI_VictorSRX _rghtFollower = new WPI_VictorSRX(1);
  WPI_TalonSRX _leftFront = new WPI_TalonSRX(0);
  WPI_VictorSRX _leftFollower = new WPI_VictorSRX(0);

  DifferentialDrive _diffDrive = new DifferentialDrive(_leftFront, _rghtFront);

  Joystick _joystick = new Joystick(0);

    public void teleopPeriodic() {
      double forw = -1 * _joystick.getRawAxis(1);
      double turn = +1 * _joystick.getRawAxis(2);

        if (Math.abs(forw) < 0.10) {
          forw = 0;
      }
      if (Math.abs(turn) < 0.10) {
          turn = 0;
      }

      _diffDrive.arcadeDrive(forw, turn);

  }
}