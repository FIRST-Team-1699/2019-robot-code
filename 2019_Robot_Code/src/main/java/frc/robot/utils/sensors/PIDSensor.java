/**
 * An interface for the requirements to work with our PIDLoop. Not much here.
 * 
 * @author thatging3rkid, FIRST Team 1699
 */
package frc.robot.utils.sensors;

/**
 * Requirements to be a PIDSensor.
 */
public interface PIDSensor {
	
	/**
	 * Read the value off of the sensor
	 * 
	 * @return value of the sensor
	 */
	public double get();

}