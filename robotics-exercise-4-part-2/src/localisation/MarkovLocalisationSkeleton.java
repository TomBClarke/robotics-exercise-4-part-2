package localisation;

import gridMap.GridMap;

import javax.swing.JFrame;

import lejos.geom.Point;
import lejos.robotics.mapping.LineMap;
import lejos.robotics.navigation.Pose;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;
import lejos.util.Delay;
import rp.robotics.localisation.ActionModel;
import rp.robotics.localisation.GridPositionDistribution;
import rp.robotics.localisation.PerfectActionModel;
import rp.robotics.localisation.PerfectSensorModel;
import rp.robotics.localisation.SensorModel;
import rp.robotics.mapping.Heading;
import rp.robotics.mapping.IGridMap;
import rp.robotics.mapping.MapUtils;
import rp.robotics.mapping.RPLineMap;
import rp.robotics.simulation.SimulatedRobot;
import rp.robotics.visualisation.GridMapViewer;
import rp.robotics.visualisation.GridPositionDistributionVisualisation;
import rp.robotics.visualisation.KillMeNow;

public class MarkovLocalisationSkeleton {

	// Maps
	private final LineMap m_lineMap;
	private final IGridMap m_gridMap;

	// Probability distribution over the position of a robot on the given
	// grid map
	private GridPositionDistribution m_distribution;

	// Robot to drive around
	private final SimulatedRobot m_robot;

	// Visualisation
	private GridPositionDistributionVisualisation m_mapVis;
	private final float m_translationAmount;


	public MarkovLocalisationSkeleton(SimulatedRobot _robot, LineMap _lineMap,
			IGridMap _gridMap, float _translationAmount) {

		m_robot = _robot;
		m_lineMap = _lineMap;
		m_gridMap = _gridMap;
		m_distribution = new GridPositionDistribution(m_gridMap);
		m_translationAmount = _translationAmount;
	}

	/**
	 * Optionally run the visualisation of the robot and localisation process.
	 * This is not necessary to run the localisation and could be removed once
	 * on the real robot.
	 */
	public void visualise() {
		JFrame frame = new JFrame("Map Viewer");
		frame.addWindowListener(new KillMeNow());

		// visualise the distribution on top of a line map
		m_mapVis = new GridPositionDistributionVisualisation(m_distribution,
				m_lineMap, 2);

		// Visualise the robot
		m_mapVis.addRobot(m_robot);

		frame.add(m_mapVis);
		frame.pack();
		frame.setSize(1050, 600);
		frame.setVisible(true);
	}

	/***
	 * Move the robot and update the distribution with the action model
	 * 
	 * @param distance
	 * @param _heading
	 * @param _sensorModel
	 */

	public void run() {

		ActionModel actionModel = new OurPerfectActionModel(m_gridMap);
		SensorModel sensorModel = new OurSensorModel(m_gridMap);

		// Assuming all the moves go in this direction. This will not work once
		// the robot turns...

		TurnBehaviour turning = new TurnBehaviour(m_robot);
		MoveForwardBehaviour forwards = new MoveForwardBehaviour(m_robot,turning,m_translationAmount,actionModel,sensorModel,m_distribution,m_mapVis);
		Arbitrator arby = new Arbitrator(new Behavior[] {turning,forwards});
		
		arby.start();
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {

		// Work on this map
		RPLineMap lineMap = MapUtils.create2015Map1();

		// Grid map configuration

		// grid map dimensions for this line map
		int xJunctions = 14;
		int yJunctions = 8;
		float junctionSeparation = 30;

		// position of grid map 0,0
		int xInset = 15;
		int yInset = 15;

		IGridMap gridMap = new GridMap(xJunctions, yJunctions, xInset, yInset,
				junctionSeparation, lineMap);

		// the starting position of the robot for the simulation. This is not
		// known in the action model or position distribution
		int startGridX = 2;
		int startGridY = 1;

		// this converts the grid position into the underlying continuous
		// coordinate frame
		Point startPoint = gridMap.getCoordinatesOfGridPosition(startGridX,
				startGridY);

		// starting heading
		float startTheta = Heading.toDegrees(Heading.PLUS_X);

		Pose startPose = new Pose(startPoint.x, startPoint.y, startTheta);

		// This creates a simulated robot with single, forward pointing distance
		// sensor with similar properties to the Lego ultrasonic sensor but
		// without the noise
		SimulatedRobot robot = SimulatedRobot.createSingleSensorRobot(
				startPose, lineMap);

		// This does the same as above but adds noise to the range readings
		// SimulatedRobot robot = SimulatedRobot.createSingleSensorRobot(
		// startPose, lineMap);

		MarkovLocalisationSkeleton ml = new MarkovLocalisationSkeleton(robot,
				lineMap, gridMap, junctionSeparation);
		ml.visualise();
		ml.run();

	}
}
