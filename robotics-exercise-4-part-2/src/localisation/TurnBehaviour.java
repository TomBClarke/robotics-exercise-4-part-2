package localisation;

import rp.robotics.mapping.Heading;
import rp.robotics.simulation.SimulatedRobot;
import lejos.robotics.subsumption.Behavior;

/**
 * The behaviour for turning at a junction when there is an obstacle in front
 * @author Rowan Cole
 *
 */
public class TurnBehaviour implements Behavior {
	Heading heading;
	SimulatedRobot robot;
	
	/**
	 * Constructor for turning behaviour
	 * @param robot The simulated robot being run
	 */
	public TurnBehaviour(SimulatedRobot robot){
		heading = Heading.PLUS_X;
		this.robot = robot;
	}
	
	@Override
	public boolean takeControl() {
		return true;
	}

	@Override
	public void action() {
		robot.rotate(-90);
		if(heading==Heading.PLUS_X){
			heading=Heading.MINUS_Y;
		} else if(heading==Heading.MINUS_Y){
			heading=Heading.MINUS_X;
		} else if(heading==Heading.MINUS_X){
			heading=Heading.PLUS_Y;
		} else {
			heading=Heading.PLUS_X;
		}
	}

	@Override
	public void suppress() {

	}
	
	/**
	 * Gets the current heading of the robot
	 * @return The heading
	 */
	public Heading getHeading(){
		return heading;
	}

}
