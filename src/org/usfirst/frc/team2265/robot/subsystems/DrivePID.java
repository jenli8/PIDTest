package org.usfirst.frc.team2265.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DrivePID extends PIDSubsystem {

	private double Kp= 0.5; 
	private double Ki= 0.0; 
	private double Kd= 0.0; 
	private double desiredVel= 0.475;  // Should be converted into Velocity eventually. 
	
	private double frontLeftVal, frontRightVal;
	
	Encoder frontLeftEnc; 
	Encoder frontRightEnc;
	
	public Talon frontLeft; 
	public Talon frontRight; 
	public Talon rearLeft; 
	public Talon rearRight;  
	
	PIDController FrontLeftPID; 
	PIDController FrontRightPID; 
	/*PIDController rearLeft; 
	 * PIDController rearRight; 
	 * Not sure if we'll have 2 PIDControllers or 4. 
	 */
    // Initialize your subsystem here
    public DrivePID() {
    	super("DrivePID", 0.5,0.0,0.0); 
    	frontLeftEnc= new Encoder(0,1);
    	frontRightEnc= new Encoder(2,3); 
    	
    	frontLeftEnc.setDistancePerPulse(0.0008); 
    	frontRightEnc.setDistancePerPulse(0.0008);
    	
    	frontLeftEnc.startLiveWindowMode(); 
    	frontRightEnc.startLiveWindowMode();
    	
    	FrontLeftPID= new PIDController(Kp, Ki, Kd,frontLeftEnc, frontLeft ); 
    	FrontRightPID= new PIDController(Kp, Ki, Kd, frontRightEnc, frontRight); 
    	
        FrontLeftPID.enable(); 
        FrontRightPID.enable(); 
        
        FrontLeftPID.setInputRange(0,0.475); 
        FrontRightPID.setInputRange(0,0.475); 
        
        FrontLeftPID.setPercentTolerance(0.1); 
        FrontLeftPID.setPercentTolerance(0.1); 
    }
    
    public void getEncValues() {
    	frontLeftVal= frontLeftEnc.getRate(); 
    	frontRightVal= frontRightEnc.getRate();
    	
    	FrontLeftPID.setSetpoint(desiredVel);
    	FrontRightPID.setSetpoint(desiredVel);
    }
    
    public void equalize() {
    	if(!FrontLeftPID.onTarget()) {
    		double diffL= FrontLeftPID.getError();
    		frontLeft.set(desiredVel + diffL); //Need to convert velocity vals back to PWM vals. 
    	}
    	if(!FrontRightPID.onTarget()) {
    		double diffR= FrontRightPID.getError(); 
    		frontRight.set(desiredVel + diffR); 
    	}
    }
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
    	return 0.0;
    }
    
    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    }
}
