package localisation;

import rp.robotics.mapping.Heading;
import rp.robotics.simulation.SimulatedRobot;
import lejos.robotics.subsumption.Behavior;

public class TurnBehaviour implements Behavior {
	Heading heading;
	SimulatedRobot robot;
	
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
	
	public Heading getHeading(){
		return heading;
	}

}
