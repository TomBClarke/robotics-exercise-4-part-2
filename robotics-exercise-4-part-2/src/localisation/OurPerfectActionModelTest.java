package localisation;

import gridMap.GridMap;
import lejos.geom.Point;
import lejos.robotics.navigation.Pose;

import org.junit.Assert;
import org.testng.annotations.Test;

import rp.robotics.localisation.ActionModel;
import rp.robotics.localisation.GridPositionDistribution;
import rp.robotics.mapping.Heading;
import rp.robotics.mapping.IGridMap;
import rp.robotics.mapping.MapUtils;
import rp.robotics.mapping.RPLineMap;
import rp.robotics.simulation.SimulatedRobot;

/**
 * TestNG class for testing the perfect action model.
 * @author Rowan Cole
 */
public class OurPerfectActionModelTest {
	
	private RPLineMap m_lineMap;
	private IGridMap m_gridMap;

	private GridPositionDistribution m_distribution;

	private SimulatedRobot m_robot;
	private float m_translationAmount;
	
	/**
	 * Translates the robot by 1 in the given heading and updates the action model.
	 * @param distance The distance to move (cell size)
	 * @param _heading The direction to be moved in
	 * @param _actionModel The action model to be updated
	 */
	private void move(float distance, Heading _heading,
			ActionModel _actionModel) {
		m_robot.translate(m_translationAmount);
		m_distribution = _actionModel.updateAfterMove(m_distribution, _heading);
	}
	
	@Test
	public void updateAfterMoveTest() {
		setup();
		
		ActionModel actionModel = new OurPerfectActionModel(m_gridMap);
		
		int currentX = 2;
		int currentY = 1;
		float lastProb = 0.0f;
		
		int horizontal = 3;
		int vertical = 1;

		Heading movementHeading;
		int moves;

		movementHeading = Heading.PLUS_X;
		moves = horizontal;
		
		for (int i = 0; i < moves; i++) {
			move(m_translationAmount, movementHeading, actionModel);
			currentX++;
			Assert.assertTrue(m_distribution.getProbability(currentX, currentY)>=lastProb);
			lastProb=m_distribution.getProbability(currentX, currentY);
		}

		m_robot.rotate(90);
		movementHeading = Heading.PLUS_Y;
		moves = vertical;
		for (int i = 0; i < moves; i++) {
			move(m_translationAmount, movementHeading, actionModel);
			currentY++;
			System.out.println(m_distribution.getProbability(currentX, currentY)+">="+lastProb);
			Assert.assertTrue(m_distribution.getProbability(currentX, currentY)>=lastProb);
			lastProb=m_distribution.getProbability(currentX, currentY);
		}

		m_robot.rotate(90);
		movementHeading = Heading.MINUS_X;
		moves = horizontal;
		for (int i = 0; i < moves; i++) {
			move(m_translationAmount, movementHeading, actionModel);
			currentX--;
			Assert.assertTrue(m_distribution.getProbability(currentX, currentY)>=lastProb);
			lastProb=m_distribution.getProbability(currentX, currentY);
		}

		m_robot.rotate(90);
		movementHeading = Heading.MINUS_Y;
		moves = vertical;
		for (int i = 0; i < moves; i++) {
			move(m_translationAmount, movementHeading, actionModel);
			currentY--;
			Assert.assertTrue(m_distribution.getProbability(currentX, currentY)>=lastProb);
			lastProb=m_distribution.getProbability(currentX, currentY);
		}

		m_robot.rotate(90);


	}
	
	/**
	 * Creates the grid map and robot to be used.
	 */
	public void setup() {

		RPLineMap lineMap = MapUtils.create2015Map1();

		// Grid map configuration

		// grid map dimensions for this line map
		int xJunctions = 14;
		int yJunctions = 8;
		float junctionSeparation = 30;

		// position of grid map 0,0
		int xInset = 15;
		int yInset = 15;

		m_gridMap = new GridMap(xJunctions, yJunctions, xInset, yInset,
				junctionSeparation, lineMap);

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

		
		m_distribution = new GridPositionDistribution(m_gridMap);
	}
}
