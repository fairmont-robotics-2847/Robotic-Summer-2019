package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {
  WPI_TalonSRX _rightMaster = new WPI_TalonSRX(1);
  WPI_VictorSPX _rghtFollower = new WPI_VictorSPX(1);
  WPI_TalonSRX _leftMaster = new WPI_TalonSRX(0);
	WPI_VictorSPX _leftFollower = new WPI_VictorSPX(0);
	WPI_VictorSPX _tempMaster = new WPI_VictorSPX(6);

  PigeonIMU _pidgey = new PigeonIMU(3);
  
  DifferentialDrive _diffDrive = new DifferentialDrive(_leftMaster, _rightMaster);
  
  Joystick _gamepad = new Joystick(0);

  DifferentialDrive m_myRobot;


  boolean[] _btns = new boolean[Constants.kNumButtonsPlusOne];
  boolean[] btns = new boolean[Constants.kNumButtonsPlusOne];

  boolean _firstCall = false;
  boolean _state = false;

  MotionProfile  _motProfExample = new MotionProfile(_tempMaster);

  enum ButtonEvent {
    ButtonOff, 
    ButtonOffToOn, 
    ButtonOn, 
    ButtonOnToOff;
  }

  @Override
  public void robotInit() {
    m_myRobot = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));

			_rightMaster.configFactoryDefault(100);
			_rghtFollower.configFactoryDefault(100);
			_leftMaster.configFactoryDefault(100);
			_leftFollower.configFactoryDefault(100);

			_rghtFollower.follow(_rightMaster);
			_leftFollower.follow(_leftMaster);

			_rightMaster.setInverted(true);
			_leftMaster.setInverted(false);
			_rghtFollower.setInverted(InvertType.FollowMaster);
			_leftFollower.setInverted(InvertType.FollowMaster);

			_rightMaster.setSensorPhase(true);
			_leftMaster.setSensorPhase(true);

			_diffDrive.setRightSideInverted(false);
  }

  @Override
  public void teleopInit(){
		_rightMaster.set(ControlMode.PercentOutput, 0);
		_leftMaster.set(ControlMode.PercentOutput,  0);
		_tempMaster.set(ControlMode.PercentOutput, 0);

		_rightMaster.configFactoryDefault();
		_leftMaster.configFactoryDefault();
		_tempMaster.configFactoryDefault();
		_pidgey.configFactoryDefault();

		_rightMaster.setNeutralMode(NeutralMode.Coast);
		_leftMaster.setNeutralMode(NeutralMode.Coast);
		_tempMaster.setNeutralMode(NeutralMode.Brake);
		
		/** Feedback Sensor Configuration */
		
		/* Configure the left Talon's selected sensor as local QuadEncoder */
		_leftMaster.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,				// Local Feedback Source
													Constants.PID_PRIMARY,					// PID Slot for Source [0, 1]
													Constants.kTimeoutMs);					// Configuration Timeout

		/* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
		_rightMaster.configRemoteFeedbackFilter(_leftMaster.getDeviceID(),					// Device ID of Source
												RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
												Constants.REMOTE_0,							// Source number [0, 1]
												Constants.kTimeoutMs);						// Configuration Timeout
		
		// /* Configure the Pigeon IMU to the other Remote Slot on the Right Talon */
		 _rightMaster.configRemoteFeedbackFilter(_pidgey.getDeviceID(),
		 										RemoteSensorSource.Pigeon_Yaw,
		 										Constants.REMOTE_1,	
		 										Constants.kTimeoutMs);
		
		/* Setup Sum signal to be used for Distance */
		_rightMaster.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs);	// Feedback Device of Remote Talon
		_rightMaster.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);	// Quadrature Encoder of current Talon
		
		/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
		_rightMaster.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
													Constants.PID_PRIMARY,
													Constants.kTimeoutMs);
		
		/* Scale Feedback by 0.5 to half the sum of Distance */
		_rightMaster.configSelectedFeedbackCoefficient(	0.5, 						// Coefficient
														Constants.PID_PRIMARY,		// PID Slot of Source 
														Constants.kTimeoutMs);		// Configuration Timeout
		
		// /* Configure Remote Slot 1 [Pigeon IMU's Yaw] to be used for Auxiliary PID Index */
		 _rightMaster.configSelectedFeedbackSensor(	FeedbackDevice.RemoteSensor1,
		 											Constants.PID_TURN,
		 											Constants.kTimeoutMs);
		
		// /* Scale the Feedback Sensor using a coefficient (Configured for 3600 units of resolution) */
		 _rightMaster.configSelectedFeedbackCoefficient(	1,
		 												Constants.PID_TURN,
		 												Constants.kTimeoutMs);

		_tempMaster.configRemoteFeedbackFilter(_rightMaster.getDeviceID(),
												RemoteSensorSource.TalonSRX_SelectedSensor,
												Constants.REMOTE_0);
		_tempMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0,
												Constants.PID_PRIMARY,
												Constants.kTimeoutMs);

		_tempMaster.configRemoteFeedbackFilter(_pidgey.getDeviceID(),
												RemoteSensorSource.Pigeon_Yaw,
												Constants.REMOTE_1,	
												Constants.kTimeoutMs);
		_tempMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1,
												Constants.PID_TURN,
												Constants.kTimeoutMs);
		
		_tempMaster.setInverted(false);
		_leftMaster.setSensorPhase(true);
		_rightMaster.setInverted(true);
		_rightMaster.setSensorPhase(true);

		_tempMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
		_tempMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
		_tempMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);
		_tempMaster.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, Constants.kTimeoutMs);
		_leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

		_rightMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
		_tempMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
		
		/* Motion Magic Configurations */
		_tempMaster.configMotionAcceleration(2000, Constants.kTimeoutMs);
		_tempMaster.configMotionCruiseVelocity(2000, Constants.kTimeoutMs);

		/**
		 * Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		_tempMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		_tempMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
		_rightMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		_rightMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);

		/* FPID Gains for Motion Profile servo */
		_tempMaster.config_kP(Constants.kSlot_MotProf, Constants.kGains_MotProf.kP, Constants.kTimeoutMs);
		_tempMaster.config_kI(Constants.kSlot_MotProf, Constants.kGains_MotProf.kI, Constants.kTimeoutMs);
		_tempMaster.config_kD(Constants.kSlot_MotProf, Constants.kGains_MotProf.kD, Constants.kTimeoutMs);
		_tempMaster.config_kF(Constants.kSlot_MotProf, Constants.kGains_MotProf.kF, Constants.kTimeoutMs);
		_tempMaster.config_IntegralZone(Constants.kSlot_MotProf, Constants.kGains_MotProf.kIzone, Constants.kTimeoutMs);
		_tempMaster.configClosedLoopPeakOutput(Constants.kSlot_MotProf, Constants.kGains_MotProf.kPeakOutput, Constants.kTimeoutMs);

		/* FPID Gains for turn servo */
		_tempMaster.config_kP(Constants.kSlot_Turning, Constants.kGains_Turning.kP, Constants.kTimeoutMs);
		_tempMaster.config_kI(Constants.kSlot_Turning, Constants.kGains_Turning.kI, Constants.kTimeoutMs);
		_tempMaster.config_kD(Constants.kSlot_Turning, Constants.kGains_Turning.kD, Constants.kTimeoutMs);
		_tempMaster.config_kF(Constants.kSlot_Turning, Constants.kGains_Turning.kF, Constants.kTimeoutMs);
		_tempMaster.config_IntegralZone(Constants.kSlot_Turning, Constants.kGains_Turning.kIzone, Constants.kTimeoutMs);
		_tempMaster.configClosedLoopPeakOutput(Constants.kSlot_Turning, Constants.kGains_Turning.kPeakOutput, Constants.kTimeoutMs);

		/**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		_tempMaster.configClosedLoopPeriod(0, closedLoopTimeMs, Constants.kTimeoutMs);
		_tempMaster.configClosedLoopPeriod(1, closedLoopTimeMs, Constants.kTimeoutMs);

		/**
		 * configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		_tempMaster.configAuxPIDPolarity(false, Constants.kTimeoutMs);

		/* Initialize */
		_firstCall = true;
		_state = false;
		zeroSensors();
	}

  @Override
  public void teleopPeriodic() {
        double forward = -1 * _gamepad.getRawAxis(1); 
        double turn = +1 * _gamepad.getRawAxis(2); 
        forward = Deadband(forward) * 0.5;
        turn = Deadband(turn) * 0.5;
        _diffDrive.arcadeDrive(forward, turn);

		    ButtonEvent bExecuteAction = ButtonEvent.ButtonOff;
        getButtons(btns, _gamepad);	

      if (btns[2] && !_btns[2]) {
        _state =  !_state;
        _firstCall = true;
      }else if (btns[1] && !_btns[1]) {
        zeroSensors();
      }

      if (btns[6] && !_btns[6])
        bExecuteAction = ButtonEvent.ButtonOffToOn;
      else if (!btns[6] && _btns[6]) 
        bExecuteAction = ButtonEvent.ButtonOnToOff;
      else if (btns[6]) 
        bExecuteAction = ButtonEvent.ButtonOn;
      else 
        bExecuteAction = ButtonEvent.ButtonOff;

      System.arraycopy(btns, 0, _btns, 0, Constants.kNumButtonsPlusOne);
      
      if(!_state){
        if (_firstCall)
          System.out.println("This is a basic arcade drive.\n");
        
        _leftMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
        _rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
      }else{
        boolean bMoveForward = (forward >= 0) ? true : false;
        double finalHeading_units = Constants.kPigeonUnitsPerRotation * turn * -1.0 * 0.25;

        if (_firstCall) {
          System.out.println("This is Motion Profile Auxiliary, also known as MotionProfileArc using the Pigeon for turn");
          System.out.println("Additonal options for running Motion Profile, to be selected when Button 6 is pressed and held:");
          System.out.println("\t1.) Select direction (forward or reverse) of Motion Profile with Left Joystick Y-Axis");
          System.out.println("\t2.) Select final heading [-90, 90] deg to current heading of robot after Motion Profile completion with Right Joystick X-Axis");
          neutralMotors("Target not set yet.\n");

        } else if (bExecuteAction == ButtonEvent.ButtonOnToOff) {
        } else if (bExecuteAction == ButtonEvent.ButtonOffToOn) {
          neutralMotors("Motion Profile Initialized, Continue holding Button 6\n");
          zeroSensors();
          _motProfExample.reset();
          _motProfExample.start(finalHeading_units, bMoveForward);
        } else if (bExecuteAction == ButtonEvent.ButtonOn) {
          _tempMaster.set(ControlMode.MotionProfileArc, _motProfExample.getSetValue().value);
          _rightMaster.follow(_tempMaster, FollowerType.AuxOutput1);
        }
        _motProfExample.control();
      }
      _firstCall = false;
    }

	void zeroSensors() {
		_leftMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		_rightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		_pidgey.setYaw(0);
		System.out.println("[Quad Encoders + Pigeon] All sensors are zeroed.\n");
	}

	double Deadband(double value) {
		if (value >= +0.05) 
			return value;
		if (value <= -0.05)
			return value;
		return 0;
	}
	
	void neutralMotors(String reason) {
		_tempMaster.neutralOutput();
		_rightMaster.neutralOutput();

		if (reason != null && reason.length() > 0) {
			System.out.print("  Motors are neutral, ");
			System.out.println(reason);
		}
	}

	void getButtons(boolean[] btns, Joystick gamepad) {
		for (int i = 1; i < Constants.kNumButtonsPlusOne; ++i) {
			btns[i] = gamepad.getRawButton(i);
		}
	}
}