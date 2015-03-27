package localisation;

import lejos.robotics.RangeReadings;
import rp.robotics.localisation.GridPositionDistribution;
import rp.robotics.localisation.SensorModel;
import rp.robotics.mapping.Heading;
import rp.robotics.mapping.IGridMap;

/**
 * Perfect sensor model that cancels out any probabilities that do no exactly match range reading.
 * @author Rowan Cole
 */
public class OurPerfectSensorModel implements SensorModel {

	private IGridMap gridMap;
	
	/**
	 * Creates perfect sensor map from gridMap given
	 * @param gridMap
	 */
	public OurPerfectSensorModel(IGridMap gridMap){
		this.gridMap=gridMap;
	}
	
	@Override
	public GridPositionDistribution updateAfterSensing(
			GridPositionDistribution _dist, Heading _heading,
			RangeReadings _readings) {
		GridPositionDistribution newDist = new GridPositionDistribution(_dist);
		if(_readings.getRange(0)<255){
			for (int y = 0; y < newDist.getGridHeight(); y++) {
				for (int x = 0; x < newDist.getGridWidth(); x++) {
					float actualRange = gridMap.rangeToObstacleFromGridPosition(x,y, Heading.toDegrees(_heading));
					if(actualRange!=_readings.getRange(0)){
						newDist.setProbability(x, y, 0.0f);
					}
				}
			}
			scaleProbabilities(newDist);
		}
		return newDist;
	
	}

	/**
	 * Method to normalise the probabilities so that they all add up to 1
	 * @param _to The distribution to be normalised
	 */
	private void scaleProbabilities(GridPositionDistribution _to){
		float a = 1/(_to.sumProbabilities());
		for (int y = 0; y < _to.getGridHeight(); y++) {
			for (int x = 0; x < _to.getGridWidth(); x++) {
				if (!_to.isObstructed(x, y)) {
					float prob = a*_to.getProbability(x, y);
					_to.setProbability(x, y, prob);
				}
			}
		}
	}
	

}