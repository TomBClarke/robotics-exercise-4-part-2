package localisation;

import lejos.robotics.RangeReadings;
import rp.robotics.localisation.GridPositionDistribution;
import rp.robotics.localisation.SensorModel;
import rp.robotics.mapping.Heading;
import rp.robotics.mapping.IGridMap;

/**
 * Model to deal with an imperfect sensor. Full sensor model not implemented, just thresholds used.
 * @author Rowan Cole
 */
public class OurSensorModel implements SensorModel {
	private IGridMap gridMap;
	
	/**
	 * Creates perfect sensor map from gridMap given
	 * @param gridMap
	 */
	public OurSensorModel(IGridMap gridMap){
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
					if((_readings.getRange(0)>actualRange+10)||(_readings.getRange(0)<actualRange-10)){
						newDist.setProbability(x, y, 0.0f);
					} else if((_readings.getRange(0)>actualRange+5)||(_readings.getRange(0)<actualRange-5)){
						float oldProb = _dist.getProbability(x,y);
						newDist.setProbability(x, y, oldProb/2);
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
				float prob = a*_to.getProbability(x, y);
				_to.setProbability(x, y, prob);
			}
		}
	}

}