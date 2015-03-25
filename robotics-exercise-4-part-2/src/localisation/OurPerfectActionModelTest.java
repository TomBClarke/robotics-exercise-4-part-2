package localisation;

import gridMap.GridMap;
import lejos.geom.Point;
import lejos.robotics.navigation.Pose;

import org.junit.Assert;
import org.testng.annotations.Test;

import rp.robotics.localisation.ActionModel;
import rp.robotics.localisation.GridPositionDistribution;
import rp.robotics.localisation.PerfectSensorModel;
import rp.robotics.localisation.SensorModel;
import rp.robotics.mapping.Heading;
import rp.robotics.mapping.IGridMap;
import rp.robotics.mapping.MapUtils;
import rp.robotics.mapping.RPLineMap;
import rp.robotics.simulation.SimulatedRobot;

public class OurPerfectActionModelTest {
	// Maps
	private RPLineMap m_lineMap;
	private IGridMap m_gridMap;

	// Probability distribution over the position of a robot on the given
	// grid map
	private GridPositionDistribution m_distribution;

	// Robot to drive around
	private SimulatedRobot m_robot;
	private float m_translationAmount;
	
	private void move(float distance, Heading _heading,
			ActionModel _actionModel, SensorModel _sensorModel) {
		m_robot.translate(m_translationAmount);
		m_distribution = _actionModel.updateAfterMove(m_distribution, _heading);
	}
	
	@Test
	public void updateAfterMoveTest() {
		setup();
		
		ActionModel actionModel = new OurPerfectActionModel(m_gridMap);
		SensorModel sensorModel = new PerfectSensorModel();

		
		int currentX = 2;
		int currentY = 1;
		float lastProb = 0.0f;
		
		int horizontal = 3;
		int vertical = 1;

		// Assuming all the moves go in this direction. This will not work once
		// the robot turns...
		Heading movementHeading;
		int moves;

		movementHeading = Heading.PLUS_X;
		moves = horizontal;
		
		for (int i = 0; i < moves; i++) {
			move(m_translationAmount, movementHeading, actionModel,
					sensorModel);
			currentX++;
			Assert.assertTrue(m_distribution.getProbability(currentX, currentY)>=lastProb);
			lastProb=m_distribution.getProbability(currentX, currentY);
		}

		m_robot.rotate(90);
		movementHeading = Heading.PLUS_Y;
		moves = vertical;
		for (int i = 0; i < moves; i++) {
			move(m_translationAmount, movementHeading, actionModel,
					sensorModel);
			currentY++;
			System.out.println(m_distribution.getProbability(currentX, currentY)+">="+lastProb);
			Assert.assertTrue(m_distribution.getProbability(currentX, currentY)>=lastProb);
			lastProb=m_distribution.getProbability(currentX, currentY);
		}

		m_robot.rotate(90);
		movementHeading = Heading.MINUS_X;
		moves = horizontal;
		for (int i = 0; i < moves; i++) {
			move(m_translationAmount, movementHeading, actionModel,
					sensorModel);
			currentX--;
			Assert.assertTrue(m_distribution.getProbability(currentX, currentY)>=lastProb);
			lastProb=m_distribution.getProbability(currentX, currentY);
		}

		m_robot.rotate(90);
		movementHeading = Heading.MINUS_Y;
		moves = vertical;
		for (int i = 0; i < moves; i++) {
			move(m_translationAmount, movementHeading, actionModel,
					sensorModel);
			currentY--;
			Assert.assertTrue(m_distribution.getProbability(currentX, currentY)>=lastProb);
			lastProb=m_distribution.getProbability(currentX, currentY);
		}

		m_robot.rotate(90);


	}
	
	public void setup() {

		// Work on this map
		m_lineMap = MapUtils.create2014Map2();

		// Grid map configuration

		// Grid junction numbers
		int xJunctions = 10;
		int yJunctions = 7;

		m_translationAmount = 30;

		int xInset = 14;
		int yInset = 31;

		m_gridMap = new GridMap(xJunctions, yJunctions, xInset, yInset,
				m_translationAmount, m_lineMap);

		// the starting position of the robot for the simulation. This is not
		// known in the action model or position distribution
		int startGridX = 2;
		int startGridY = 1;

		// this converts the grid position into the underlying continuous
		// coordinate frame
		Point startPoint = m_gridMap.getCoordinatesOfGridPosition(startGridX,
				startGridY);

		// starting heading
		float startTheta = Heading.toDegrees(Heading.PLUS_X);

		Pose startPose = new Pose(startPoint.x, startPoint.y, startTheta);

		// This creates a simulated robot with single, forward pointing distance
		// sensor with similar properties to the Lego ultrasonic sensor but
		// without the noise
		m_robot = SimulatedRobot.createSingleNoiseFreeSensorRobot(
				startPose, m_lineMap);

		// This does the same as above but adds noise to the range readings
		// SimulatedRobot robot = SimulatedRobot.createSingleSensorRobot(
		// startPose, lineMap);
		
		m_distribution = new GridPositionDistribution(m_gridMap);
	}
}
