package localisation;

import rp.robotics.localisation.ActionModel;
import rp.robotics.localisation.GridPositionDistribution;
import rp.robotics.mapping.Heading;
import rp.robotics.mapping.IGridMap;

/**
 * A perfect action mode implementing the ActionModel interface.
 * @author Rowan Cole
 *
 */
public class OurPerfectActionModel implements ActionModel {
	private IGridMap gridMap;
	
	/**
	 * Constructor for the perfect action model.
	 * @param gridMap The grid map to be use to check transitions against
	 */
	public OurPerfectActionModel(IGridMap gridMap){
		this.gridMap=gridMap;
	}
	
	@Override
	public GridPositionDistribution updateAfterMove(
			GridPositionDistribution _from, Heading _heading) {

		// Create the new distribution that will result from applying the action
		// model
		GridPositionDistribution to = new GridPositionDistribution(_from);

		// Move the probability in the correct direction for the action
		shiftProbabilities(_from, to, _heading);
		scaleProbabilities(to);
		return to;
	}

	/**
	 * Move probabilities from _from one cell in the plus x direction into _to
	 * @param _from The original distribution
	 * @param _to The next distribution
	 * @param heading The direction of the move
	 */
	private void shiftProbabilities(GridPositionDistribution _from,
			GridPositionDistribution _to, Heading heading) {
		
		for (int y = 0; y < _to.getGridHeight(); y++) {
			for (int x = 0; x < _to.getGridWidth(); x++) {
				if (!_to.isObstructed(x, y)) {
					int fromX;
					int fromY;
					if(heading == Heading.PLUS_X){
						fromX = x-1;
						fromY = y;
					}else if(heading == Heading.MINUS_X){
						fromX = x+1;
						fromY = y;
					}else if(heading == Heading.PLUS_Y){
						fromX = x;
						fromY = y-1;
					}else{
						fromX = x;
						fromY = y+1;
					}
					// position after move
					int toX = x;
					int toY = y;
					
					if(!gridMap.isValidTransition(fromX, fromY, toX, toY)){
						_to.setProbability(x, y, 0.0f);
					}
					else{
						float fromProb = _from.getProbability(fromX, fromY);
						// set probability for position after move
						_to.setProbability(toX, toY, fromProb);
					}
				}
			}
		}
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
		System.out.println(_to.sumProbabilities());
	}
	
	
	
	
}