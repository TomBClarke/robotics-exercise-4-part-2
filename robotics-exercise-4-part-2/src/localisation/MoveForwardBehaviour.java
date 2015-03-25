package localisation;

import rp.robotics.localisation.ActionModel;
import rp.robotics.localisation.GridPositionDistribution;
import rp.robotics.localisation.SensorModel;

import rp.robotics.simulation.SimulatedRobot;
import rp.robotics.visualisation.GridPositionDistributionVisualisation;
import lejos.robotics.subsumption.Behavior;
import lejos.util.Delay;

public class MoveForwardBehaviour implements Behavior {
	private SimulatedRobot m_robot;
	private TurnBehaviour turning;
	private float translationAmount;
	private ActionModel actionModel;
	private SensorModel sensorModel;
	private GridPositionDistribution distribution;
	private GridPositionDistributionVisualisation mapVis;
	
	
	public MoveForwardBehaviour(SimulatedRobot robot,TurnBehaviour turning, float translationAmount, 
			ActionModel actionModel, SensorModel sensorModel,GridPositionDistribution distribution,GridPositionDistributionVisualisation mapVis){
		this.m_robot = robot;
		this.turning = turning;
		this.translationAmount=translationAmount;
		this.actionModel = actionModel;
		this.sensorModel = sensorModel;
		this.distribution = distribution;
		this.mapVis = mapVis;
		
		
	}
	
	@Override
	public boolean takeControl() {
		return (m_robot.getRangeValues().getRange(0)>30);
		
	}

	@Override
	public void action() {	
		move();
	}

	@Override
	public void suppress() {
		
	}
	
	private void move() {
		// move robot
		m_robot.translate(translationAmount);

		// now update estimate of position using the action model
		distribution = actionModel.updateAfterMove(distribution, turning.getHeading());

		// if visualising, update the shown distribution
		if (mapVis != null) {
			mapVis.setDistribution(distribution);
		}

		// A short delay so we can see what's going on
		Delay.msDelay(1000);

		distribution = sensorModel.updateAfterSensing(distribution,
				turning.getHeading(), m_robot.getRangeValues());

		// if visualising, update the shown distribution
		if (mapVis != null) {
			mapVis.setDistribution(distribution);
		}

		// A short delay so we can see what's going on
		Delay.msDelay(1000);
	}

	
}
