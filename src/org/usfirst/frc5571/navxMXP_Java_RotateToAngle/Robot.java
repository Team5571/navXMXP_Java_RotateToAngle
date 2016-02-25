package org.usfirst.frc5571.navxMXP_Java_RotateToAngle;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.hal.SerialPortJNI;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CANTalon;

/**
 * This is a demo program showing the use of the navX MXP to implement
 * a "rotate to angle" feature.
 *
 * This example will automatically rotate the robot to one of four
 * angles (0, 90, 180 and 270 degrees).
 *
 * This rotation can occur when the robot is still, but can also occur
 * when the robot is driving.  When using field-oriented control, this
 * will cause the robot to drive in a straight line, in whathever direction
 * is selected.
 *
 * This example also includes a feature allowing the driver to "reset"
 * the "yaw" angle.  When the reset occurs, the new gyro angle will be
 * 0 degrees.  This can be useful in cases when the gyro drifts, which
 * doesn't typically happen during a FRC match, but can occur during
 * long practice sessions.
 *
 * Note that the PID Controller coefficients defined below will need to
 * be tuned for your drive system.
 */

public class Robot extends SampleRobot implements PIDOutput {
    AHRS ahrs;
    RobotDrive myRobot;
    Joystick stick;
    PIDController turnController;
    double rotateToAngleRate;
    
    /* The following PID Controller coefficients will need to be tuned */
    /* to match the dynamics of your drive system.  Note that the      */
    /* SmartDashboard in Test mode has support for helping you tune    */
    /* controllers by displaying a form where you can enter new P, I,  */
    /* and D constants and test the mechanism.                         */
    
    static final double kP = 0.03;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
    
    static final double kToleranceDegrees = 2.0f;

    // Channels for the wheels
//    final static int frontLeftChannel	= 2;
//    final static int rearLeftChannel	= 3;
//    final static int frontRightChannel	= 1;
//    final static int rearRightChannel	= 0;
    
    private CANTalon LeftMotorMaster = new CANTalon(1);
    private CANTalon LeftMotorSlave = new CANTalon(3);
    private CANTalon RightMotorMaster = new CANTalon(2);
    private CANTalon RightMotorSlave = new CANTalon(4);
 
        
    public Robot() {
        
    	myRobot = new RobotDrive(LeftMotorMaster, RightMotorMaster);
    	LeftMotorSlave.changeControlMode(TalonControlMode.Follower);
    	LeftMotorSlave.set(LeftMotorMaster.getDeviceID());
    	RightMotorSlave.changeControlMode(TalonControlMode.Follower);
    	RightMotorSlave.set(RightMotorMaster.getDeviceID());
        myRobot.setExpiration(0.1);
        stick = new Joystick(0);
        
        
        
        try {
            /* Communicate w/navX MXP via the MXP SPI Bus.                                     */
            /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
            /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
       
            ahrs = new AHRS(SerialPort.Port.kUSB);
        
        	//ahrs = new AHRS(SPI.Port.kMXP); 
            //SerialPort serialPort = new SerialPortJNI();

         
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
        turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
        turnController.setInputRange(-180.0f,  180.0f);
        turnController.setOutputRange(-1.0, 1.0);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
        
        /* Add the PID Controller to the Test-mode dashboard, allowing manual  */
        /* tuning of the Turn Controller's P, I and D coefficients.            */
        /* Typically, only the P value needs to be modified.                   */
        LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
    }

    /**
     * Drive left & right motors for 2 seconds then stop
     */
    public void autonomous() {
        myRobot.setSafetyEnabled(false);
        myRobot.drive(0.0, 0.0);    // stop robot
        Timer.delay(2.0);		    //    for 2 seconds
        myRobot.drive(0.0, 0.0);	// stop robot
    }

    /**
     * Runs the motors with onnidirectional drive steering.
     * 
     * Implements Field-centric drive control.
     * 
     * Also implements "rotate to angle", where the angle
     * being rotated to is defined by one of four buttons.
     * 
     * Note that this "rotate to angle" approach can also 
     * be used while driving to implement "straight-line
     * driving".
     */
    public void operatorControl() {
        myRobot.setSafetyEnabled(true);
        while (isOperatorControl() && isEnabled()) {
            boolean rotateToAngle = false;
            if ( stick.getRawButton(1)) {
                ahrs.reset();
            }
            if ( stick.getRawButton(2)) {
                turnController.setSetpoint(0.0f);
                rotateToAngle = true;
            } else if ( stick.getRawButton(3)) {
                turnController.setSetpoint(90.0f);
                rotateToAngle = true;
            } else if ( stick.getRawButton(4)) {
                turnController.setSetpoint(179.9f);
                rotateToAngle = true;
            } else if ( stick.getRawButton(5)) {
                turnController.setSetpoint(-90.0f);
                rotateToAngle = true;
            }
            double currentRotationRate;
            if ( rotateToAngle ) {
                turnController.enable();
                currentRotationRate = rotateToAngleRate;
            } else {
//                turnController.disable();
//                currentRotationRate = stick.getTwist();
            	currentRotationRate = 0;
            	 /* Display 6-axis Processed Angle Data                                      */
                SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
                SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
                SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
                SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
                SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
                
                /* Display tilt-corrected, Magnetometer-based heading (requires             */
                /* magnetometer calibration to be useful)                                   */
                
                SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());
//                
//                /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
//                SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());
//
                /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
                /* path for upgrading from the Kit-of-Parts gyro to the navx MXP            */
                
                SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
                SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());
//
//                /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
//                
//                SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
//                SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
//                SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
//                SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());
//
//                /* Display estimates of velocity/displacement.  Note that these values are  */
//                /* not expected to be accurate enough for estimating robot position on a    */
//                /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
//                /* of these errors due to single (velocity) integration and especially      */
//                /* double (displacement) integration.                                       */
//                
//                SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX());
//                SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY());
//                SmartDashboard.putNumber(   "Displacement_X",       ahrs.getDisplacementX());
//                SmartDashboard.putNumber(   "Displacement_Y",       ahrs.getDisplacementY());
//                
//                /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
//                /* NOTE:  These values are not normally necessary, but are made available   */
//                /* for advanced users.  Before using this data, please consider whether     */
//                /* the processed data (see above) will suit your needs.                     */
//                
////                SmartDashboard.putNumber(   "RawGyro_X",            ahrs.getRawGyroX());
////                SmartDashboard.putNumber(   "RawGyro_Y",            ahrs.getRawGyroY());
////                SmartDashboard.putNumber(   "RawGyro_Z",            ahrs.getRawGyroZ());
////                SmartDashboard.putNumber(   "RawAccel_X",           ahrs.getRawAccelX());
////                SmartDashboard.putNumber(   "RawAccel_Y",           ahrs.getRawAccelY());
////                SmartDashboard.putNumber(   "RawAccel_Z",           ahrs.getRawAccelZ());
////                SmartDashboard.putNumber(   "RawMag_X",             ahrs.getRawMagX());
////                SmartDashboard.putNumber(   "RawMag_Y",             ahrs.getRawMagY());
////                SmartDashboard.putNumber(   "RawMag_Z",             ahrs.getRawMagZ());
////                SmartDashboard.putNumber(   "IMU_Temp_C",           ahrs.getTempC());
//                
//                /* Omnimount Yaw Axis Information                                           */
//                /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
//                AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
//                SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
//                SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
//                
                /* Sensor Board Information                                                 */
                SmartDashboard.putString(   "FirmwareVersion",      ahrs.getFirmwareVersion());
//                
//                /* Quaternion Data                                                          */
//                /* Quaternions are fascinating, and are the most compact representation of  */
//                /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
//                /* from the Quaternions.  If interested in motion processing, knowledge of  */
//                /* Quaternions is highly recommended.                                       */
////                SmartDashboard.putNumber(   "QuaternionW",          ahrs.getQuaternionW());
////                SmartDashboard.putNumber(   "QuaternionX",          ahrs.getQuaternionX());
////                SmartDashboard.putNumber(   "QuaternionY",          ahrs.getQuaternionY());
////                SmartDashboard.putNumber(   "QuaternionZ",          ahrs.getQuaternionZ());
//                
                /* Connectivity Debugging Support                                           */
                SmartDashboard.putNumber(   "IMU_Byte_Count",       ahrs.getByteCount());
                SmartDashboard.putNumber(   "IMU_Update_Count",     ahrs.getUpdateCount());
            }
            try {
                /* Use the joystick X axis for lateral movement,          */
                /* Y axis for forward movement, and the current           */
                /* calculated rotation rate (or joystick Z axis),         */
                /* depending upon whether "rotate to angle" is active.    */
//                myRobot.mecanumDrive_Cartesian(stick.getX(), stick.getY(), 
//                                               currentRotationRate, ahrs.getAngle());
//            	myRobot.mecanumDrive_Cartesian(0.0, 0.0, 
 //                     currentRotationRate, ahrs.getAngle());
            	myRobot.arcadeDrive(0.0, currentRotationRate);
            	
                
            } catch( RuntimeException ex ) {
                DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
            }
            Timer.delay(0.005);		// wait for a motor update time
        }
    }

    /**
     * Runs during test mode
     */
    public void test() {
    	
    }

    @Override
    /* This function is invoked periodically by the PID Controller, */
    /* based upon navX MXP yaw angle input and PID Coefficients.    */
    public void pidWrite(double output) {
        rotateToAngleRate = output;
    }
    
}
