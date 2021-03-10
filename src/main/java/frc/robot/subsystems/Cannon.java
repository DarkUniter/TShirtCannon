package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.drivers.LazyTalonSRX;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonSRXFactory;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

public class Cannon extends Subsystem {

    private static Cannon mInstance = null;
    public static Cannon getInstance() {
        if(mInstance == null) {
            mInstance = new Cannon();
        }
        return mInstance;
    }

    //hardware
    private final Solenoid mShooterSolenoid;
    private final LazyTalonSRX mCannonMotor; 
    private final AnalogInput mDistanceSensor;
    private final DigitalInput mPressureSwitch;
    private final Encoder mTurnEncoder; 
    

    //random variables
    private double mChamberNumber = 1;
    private boolean[] mChambers = { true, true, true, true, true, true, true, true };
    private double mAllChambersEmpty = 0;
    private double mShooterSolenoidDelay = 0;


    //control states
    private CannonState mCurrentState = CannonState.OFF;
    private boolean mStateChanged = false;
    private double mStateChangeTimestamp = 0;

    private void configureTurnMotor(final LazyTalonSRX talon) {
        PheonixUtil.checkError(talon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs), talon.getName() + " failed to set voltage compensation", true);
        talon.configContinuousCurrentLimit(15, Constants.kTimeOutMs);
        talon.configPeakCurrentLimit(15, Constants.kTimeOutMs);
        talon.enableVoltageCompensation(true);
        talon.enableCurrentLimit(true);
    }

    private Cannon () {
        mShooterSolenoid = new Solenoid(Ports.PCM_ID, Ports.SHOOTER_SOLENOID);
        mCannonMotor = TalonSRXFactory.createDefaultTalon("Cannon Motor", Ports.CANNON_MOTOR);
        mDistanceSensor = new AnalogInput(Ports.DISTANCE_SENSOR);
        mPressureSwitch = new DigitalInput(Ports.PRESSURE_SWITCH);
        mTurnEncoder = new Encoder(Ports.CANNON_ENCODER_A, Ports.CANNON_ENCODER_B);
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop(){

            @Override
            public void onStart(double timestamp) {
                mTurnEncoder.reset();
            }

            @Override
            public void onLoop(double timestamp) {
                if(mCurrentState == CannonState.CLOCKWISE) {
                    double ticksNeeded = (++mChamberNumber);
                    ticksNeeded = ticksNeeded * Constants.TICKS_PER_CHAMBER;
                    double rawTicks = mTurnEncoder.getRaw();
                    if(rawTicks >= ticksNeeded) {
                        conformToState(CannonState.OFF);
                        ++mChamberNumber;
                    }
                }
                if(mCurrentState == CannonState.COUNTERCLOCKWISE) {
                    double ticksNeeded = (--mChamberNumber);
                    ticksNeeded = ticksNeeded * Constants.TICKS_PER_CHAMBER;
                    double rawTicks = mTurnEncoder.getRaw();
                    if(rawTicks <= ticksNeeded) {
                        conformToState(CannonState.OFF);
                        --mChamberNumber;
                    }
                }
                if(mCurrentState == CannonState.SHOOTING) {
                    if(mDistanceSensor.getValue() <= 150) {
                        if(mPressureSwitch.get() == false) {
                        }
                        else {
                            setSolenoid(true);
                            mChambers[(int) mChamberNumber] = false;
                            ++mAllChambersEmpty;
                        }
                    }
                }
                if(mCurrentState == CannonState.AUTOENABLED) {
                    if(mAllChambersEmpty < 8) {
                        if(mChambers[(int) mChamberNumber] == true) {
                            if(mDistanceSensor.getValue() <= 150) {
                                if(mPressureSwitch.get() == false) {
                                }
                                else {
                                    setSolenoid(true);
                                    mChambers[(int) mChamberNumber] = false;
                                    ++mAllChambersEmpty;
                                }
                            }
                        }
                        else {
                            if(mShooterSolenoid.get() == true) {
                                if(mShooterSolenoidDelay == 10){
                                    setSolenoid(false);
                                    mShooterSolenoidDelay = 0;
                                }
                                else{
                                    ++mShooterSolenoidDelay;
                                }
                            }
                            setTurnSpeed(CannonState.CLOCKWISE.turningSpeed);
                            double ticksNeeded = (++mChamberNumber);
                            ticksNeeded = ticksNeeded * Constants.TICKS_PER_CHAMBER;
                            double rawTicks = mTurnEncoder.getRaw();
                            if(rawTicks >= ticksNeeded) {
                                setTurnSpeed(0.0);
                                ++mChamberNumber;
                            }
                        }
                    }
                    else {
                        conformToState(CannonState.OFF);
                    }
                    
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
            
        }
        );
        
        super.registerEnabledLoops(mEnabledLooper);
    }

    public enum CannonState { 
        OFF(false, 0.0),
        COUNTERCLOCKWISE(false, -0.2),
        CLOCKWISE(false, 0.2),
        SHOOTING(false, 0.0),
        AUTOENABLED(false, 0.0);

        public boolean isShooting;
        public double turningSpeed;

        private CannonState(boolean isShooting, double turningSpeed) {
            this.isShooting = isShooting;
            this.turningSpeed = turningSpeed;
        }
    }

    public synchronized void setState(CannonState newState) {
        if(newState != mCurrentState) {
            mStateChanged = true;
            mStateChangeTimestamp = Timer.getFPGATimestamp();
        }
        mCurrentState = newState;
    }

    public synchronized CannonState getState() {
        return mCurrentState;
    } 

    public synchronized void setSolenoid(boolean state) {
        mShooterSolenoid.set(state);
    }
    
    public synchronized void setTurnSpeed(double speed) {
        mCannonMotor.set(ControlMode.PercentOutput, speed);
    }

    public synchronized void conformToState(CannonState desiredState) {
        setState(desiredState);
        setSolenoid(desiredState.isShooting);
        setTurnSpeed(desiredState.turningSpeed);
    }




    @Override
    public void stop() {
        conformToState(CannonState.OFF);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {

    }

    
}