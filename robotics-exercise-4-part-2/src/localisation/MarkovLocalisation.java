package localisation;

import gridMap.GridMap;

import javax.swing.JFrame;

import lejos.geom.Point;
import lejos.robotics.mapping.LineMap;
import lejos.robotics.navigation.Pose;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;
import rp.robotics.localisation.ActionModel;
import rp.robotics.localisation.GridPositionDistribution;
import rp.robotics.localisation.PerfectSensorModel;
import rp.robotics.localisation.SensorModel;
import rp.robotics.mapping.Heading;
import rp.robotics.mapping.IGridMap;
import rp.robotics.mapping.MapUtils;
import rp.robotics.mapping.RPLineMap;
import rp.robotics.simulation.SimulatedRobot;
import rp.robotics.visualisation.GridPositionDistributionVisualisation;
import rp.robotics.visualisation.KillMeNow;

/**
 * @author Rowan Cole
 * Main class for localisation
 */
public class MarkovLocalisation {
	
	private static int type = 3;
	// Change above value to see:
	// 0 - perfect/perfect
	// 1 - imperfect robot / perfect sensor
	// 2 - imperfect/imperfect
	// 3 - action mode only

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

	/**
	 * Constructor method for MarkovLocalisation
	 * @param _robot The simulated robot to be run
	 * @param _lineMap The line map for the robot to be run on
	 * @param _gridMap The grid map constructed from the line map
	 * @param _translationAmount The distance between junctions
	 */
	public MarkovLocalisation(SimulatedRobot _robot, LineMap _lineMap,
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

	/**
	 * Start the arbitrator for the movement behaviours
	 * @param m_robot The simulated robot that is to be run.
	 * @param _sensorModel The sensor model object that is to be used to update the distribution.
	 */

	public void run(SimulatedRobot m_robot, SensorModel sensorModel) {

		ActionModel actionModel = new OurPerfectActionModel(m_gridMap);

		// Assuming all the moves go in this direction. This will not work once
		// the robot turns...

		TurnBehaviour turning = new TurnBehaviour(m_robot);
		MoveForwardBehaviour forwards = new MoveForwardBehaviour(m_robot,turning,m_translationAmount,actionModel,sensorModel,m_distribution,m_mapVis);
		Arbitrator arby = new Arbitrator(new Behavior[] {turning,forwards});
		
		arby.start();
	}

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

		SimulatedRobot robot = SimulatedRobot.createSingleSensorRobot(
				startPose, lineMap);
		SimulatedRobot perfectRobot = SimulatedRobot.createSingleNoiseFreeSensorRobot(
				startPose, lineMap);

		SensorModel perfectSensorModel = new OurPerfectSensorModel(gridMap);
		SensorModel sensorModel = new OurSensorModel(gridMap);
		SensorModel emptySensorModel = new PerfectSensorModel();
		
		if(type == 0) {
			MarkovLocalisation ml = new MarkovLocalisation(perfectRobot,
					lineMap, gridMap, junctionSeparation);
			ml.visualise();
			ml.run(perfectRobot,perfectSensorModel);
		} else if (type == 1) {
			MarkovLocalisation ml = new MarkovLocalisation(robot,
					lineMap, gridMap, junctionSeparation);
			ml.visualise();	
			ml.run(robot,perfectSensorModel);
		} else if (type == 2) {
			MarkovLocalisation ml = new MarkovLocalisation(robot,
					lineMap, gridMap, junctionSeparation);
			ml.visualise();
			ml.run(robot,sensorModel);
		} else {
			MarkovLocalisation ml = new MarkovLocalisation(robot,
					lineMap, gridMap, junctionSeparation);
			ml.visualise();
			ml.run(perfectRobot, emptySensorModel);
		}
	}
}
