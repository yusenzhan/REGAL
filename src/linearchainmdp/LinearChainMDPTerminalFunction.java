package linearchainmdp;

import burlap.oomdp.core.ObjectInstance;
import burlap.oomdp.core.State;
import burlap.oomdp.core.TerminalFunction;

public class LinearChainMDPTerminalFunction implements TerminalFunction {
	
	int goalX;
	boolean tf; //The flag indicates whether or not the domain will terminate.
	
	public LinearChainMDPTerminalFunction(int goalX, boolean tf){
		this.goalX = goalX;
		this.tf=tf;
	}


	@Override
	public boolean isTerminal(State s) {
		
		//get location of agent in next state
		ObjectInstance agent = s.getFirstObjectOfClass(LinearChainMDPDomain.CLASSAGENT);
		int ax = agent.getDiscValForAttribute(LinearChainMDPDomain.ATTX);

		
		//are they at goal location?
		if(ax == this.goalX-1 && tf==true){
			return true;
		}
		
		return false;
	}

}
