package actionModel;

import rp.robotics.localisation.ActionModel;
import rp.robotics.localisation.GridPositionDistribution;
import rp.robotics.mapping.Heading;
import rp.robotics.mapping.IGridMap;

/**
 * Example structure for an action model that should move the probabilities 1
 * cell in the requested direction. In the case where the move would take the
 * robot into an obstacle or off the map, this model assumes the robot stayed in
 * one place. This is the same as the model presented in Robot Programming
 * lecture on action models.
 * 
 * Note that this class doesn't actually do this, instead it shows you a
 * <b>possible</b> structure for your action model.
 * 
 * @author Nick Hawes
 * 
 */
public class OurPerfectActionModel implements ActionModel {
	private IGridMap gridMap;
	
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
	 * 
	 * @param _from
	 * @param _to
	 */
	private void shiftProbabilities(GridPositionDistribution _from,
			GridPositionDistribution _to, Heading heading) {
		
		// iterate through points updating as appropriate
		for (int y = 0; y < _to.getGridHeight(); y++) {

			for (int x = 0; x < _to.getGridWidth(); x++) {

				// make sure to respect obstructed grid points
				if (!_to.isObstructed(x, y)) {

					// the action model should work out all of the different
					// ways (x,y) in the _to grid could've been reached based on
					// the _from grid and the move taken (in this case
					// HEADING.PLUS_X)

					// for example if the only way to have got to _to (x,y) was
					// from _from (x-1, y) (i.e. there was a PLUS_X move from
					// (x-1, y) then you write the value from _from (x-1, y) to
					// the _to (x, y) value

					// The below code does not translate the value, just copies
					// it to the same position

					// position before move
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