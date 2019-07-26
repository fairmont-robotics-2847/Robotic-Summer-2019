package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import com.ctre.phoenix.motion.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;

public class MotionProfile{ 
  private MotionProfileStatus _status = new MotionProfileStatus();
  double _pos=0,_vel=0,_heading=0;
  double _endHeading = 0;
  private IMotorController _motorController;
  private int _state = 0;
  private int _loopTimeout = -1;
  private boolean _bStart = false;
  private boolean _bForward = false;
  private SetValueMotionProfile _setValue = SetValueMotionProfile.Disable;
  private static final int kMinPointsInTalon = 20;
  private static final int kNumLoopsTimeout = 10;

  class PeriodicRunnable implements java.lang.Runnable {
     public void run() {  _motorController.processMotionProfileBuffer();    }
  }

  Notifier _notifer = new Notifier(new PeriodicRunnable());
  
  public MotionProfile(IMotorController motorController) {
    _motorController = motorController;
    _motorController.changeMotionControlFramePeriod(5);
		_notifer.startPeriodic(0.005);
    }

  public void reset() {
    _motorController.clearMotionProfileTrajectories();
    _setValue = SetValueMotionProfile.Disable;
    _state = 0;
    _loopTimeout = -1;
    _bStart = false;
  }

    boolean IsMotionProfile(ControlMode controlMode) {
        if (controlMode == ControlMode.MotionProfile)
            return true;
        if (controlMode == ControlMode.MotionProfileArc)
            return true;
        return false;
    }

    public void control() {
            if (_loopTimeout < 0) {
            } else {
                if (_loopTimeout == 0) {
                    Instrumentation.OnNoProgress();
                } else {
                    --_loopTimeout;
                }
            }
            if (false == IsMotionProfile(_motorController.getControlMode())) {
                _state = 0;
                _loopTimeout = -1;
            } else {
                switch (_state) {
                    case 0:
                        if (_bStart) {
                            _bStart = false;
        
                            _setValue = SetValueMotionProfile.Disable;
                            startFilling();
                            _state = 1;
                            _loopTimeout = kNumLoopsTimeout;
                        }
                        break;
                    case 1: 
                        _motorController.getMotionProfileStatus(_status);
                        if (_status.btmBufferCnt > kMinPointsInTalon) {
                            _setValue = SetValueMotionProfile.Enable;
                            _state = 2;
                            _loopTimeout = kNumLoopsTimeout;
                        }
                        break;
                    case 2:
                        _motorController.getMotionProfileStatus(_status);
                        if (_status.isUnderrun == false) {
                            _loopTimeout = kNumLoopsTimeout;
                        }
                        if (_status.activePointValid && _status.isLast) {
                            _setValue = SetValueMotionProfile.Hold;
                            _state = 0;
                            _loopTimeout = -1;
                        }
                        break;
                }
                _heading = _motorController.getActiveTrajectoryHeading();
                _pos = _motorController.getActiveTrajectoryPosition();
                _vel = _motorController.getActiveTrajectoryVelocity();

                Instrumentation.process(_status, _pos, _vel, _heading);
            }
        }

        private void startFilling() {
            startFilling(Profile.Points, Profile.kNumPoints);
        }

        private void startFilling(double[][] profile, int totalCnt) {
            TrajectoryPoint point = new TrajectoryPoint();
            if (_status.hasUnderrun) {
                Instrumentation.OnUnderrun();
                _motorController.clearMotionProfileHasUnderrun(Constants.kTimeoutMs);
            }

            _motorController.clearMotionProfileTrajectories();

            _motorController.configMotionProfileTrajectoryPeriod(Constants.kBaseTrajPeriodMs, Constants.kTimeoutMs);
            
            double finalPositionRot = profile[totalCnt-1][0];
            
            for (int i = 0; i < totalCnt; ++i) {
                double direction = _bForward ? +1 : -1;
                double positionRot = profile[i][0];
                double velocityRPM = profile[i][1];
                double heading = _endHeading * positionRot / finalPositionRot;

                point.position = direction * positionRot * Constants.kSensorUnitsPerRotation * 2;
                point.velocity = direction * velocityRPM * Constants.kSensorUnitsPerRotation / 600.0;
                point.auxiliaryPos = heading;
                point.profileSlotSelect0 = Constants.kSlot_MotProf;
                point.profileSlotSelect1 = Constants.kSlot_Turning;
                point.timeDur = (int)profile[i][2];
                point.zeroPos = false;
                if (i == 0)
                    point.zeroPos = true;
                point.useAuxPID = true;

                point.isLastPoint = false;
                if ((i + 1) == totalCnt)
                    point.isLastPoint = true;

                _motorController.pushMotionProfileTrajectory(point);
            }
        }

        void start(double endHeading, boolean bForward) {
            _bStart = true;
            _bForward = bForward;
            _endHeading = endHeading;
        }

        SetValueMotionProfile getSetValue() {
            return _setValue;
        }
}