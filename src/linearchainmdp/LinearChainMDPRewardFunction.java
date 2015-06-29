package linearchainmdp;

import burlap.oomdp.core.ObjectInstance;
import burlap.oomdp.core.State;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;

public class LinearChainMDPRewardFunction implements RewardFunction {
	
	int goalX;
	
	public LinearChainMDPRewardFunction(int goalX){
		this.goalX = goalX;
	}

	@Override
	public double reward(State s, GroundedAction a, State sprime) {
		
		//get location of agent in next state
		ObjectInstance agent = sprime.getFirstObjectOfClass(LinearChainMDPDomain.CLASSAGENT);
		int ax = agent.getDiscValForAttribute(LinearChainMDPDomain.ATTX);
		
		//are they at goal location?
		if(ax == this.goalX-1){
			return 0;
		}
		
		return -1;
	}

}
