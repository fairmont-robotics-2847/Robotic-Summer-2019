/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  /*
  this is how it connects to the talons by calling apon them 
  */
  /*
  WPI_TalonSRX values need to be changed to what they are signified as on the talon for the robot
  */
  WPI_TalonSRX _rghtFront = new WPI_TalonSRX(1);
    WPI_TalonSRX _rghtFollower = new WPI_TalonSRX(0);
    WPI_TalonSRX _leftFront = new WPI_TalonSRX(1);
    WPI_TalonSRX _leftFollower = new WPI_TalonSRX(0);
  
  DifferentialDrive _diffDrive = new DifferentialDrive(_leftFront, _rghtFront);
  
  Joystick _joystick = new Joystick(0);
  
  Faults _faults_L = new Faults();
    Faults _faults_R = new Faults();
  
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  @Override
  public void robotInit() {
    m_myRobot = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
    
      //factory default values 
        _rghtFront.configFactoryDefault();
        _rghtFollower.configFactoryDefault();
        _leftFront.configFactoryDefault();
        _leftFollower.configFactoryDefault();
    
    //set up followers where does one find this?
        _rghtFollower.follow(_rghtFront);
        _leftFollower.follow(_leftFront);
    
       /* flip values so robot moves forward when stick-forward/LEDs-green */
        _rghtFront.setInverted(true); // !< Update this
        _leftFront.setInverted(false); // !< Update this
         /*
         * set the invert of the followers to match their respective master controllers
         */
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
     String work = "";

        // need gamepad stick values 
        double forw = -1 * _joystick.getRawAxis(1); /* positive is forward */
        double turn = +1 * _joystick.getRawAxis(2); /* positive is right */
        boolean btn1 = _joystick.getRawButton(1); /* is button is down, print joystick values */
        // dead band means no action occurs
        /* deadband gamepad 10% */
        if (Math.abs(forw) < 0.10) {
            forw = 0;
        }
        if (Math.abs(turn) < 0.10) {
            turn = 0;
        }

        // states forward as the fist variable and turn as the second
        _diffDrive.arcadeDrive(forw, turn);

        /*
         *  Make sure Gamepad Forward is positive for forward on the robot, and GZ is positive for
         * RIGHT
         */
        work += " GF:" + forw + " GT:" + turn;

        /* get sensor values */
        // double leftPos = _leftFront.GetSelectedSensorPosition(0);
        // double rghtPos = _rghtFront.GetSelectedSensorPosition(0);
        double leftVelUnitsPer100ms = _leftFront.getSelectedSensorVelocity(0);
        double rghtVelUnitsPer100ms = _rghtFront.getSelectedSensorVelocity(0);

        work += " L:" + leftVelUnitsPer100ms + " R:" + rghtVelUnitsPer100ms;

        /*
         * drive motor at least 25%, Talons will auto-detect if sensor is out of phase
         */
        _leftFront.getFaults(_faults_L);
        _rghtFront.getFaults(_faults_R);

        if (_faults_L.SensorOutOfPhase) {
            work += " L sensor is out of phase";
        }
        if (_faults_R.SensorOutOfPhase) {
            work += " R sensor is out of phase";
        }

        /* print to console if btn1 is held down */
        if (btn1) {
            System.out.println(work);
  }
}
