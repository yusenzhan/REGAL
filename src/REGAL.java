import java.util.LinkedList;
import java.util.List;

import burlap.behavior.singleagent.Policy;
import burlap.behavior.singleagent.QValue;
import burlap.behavior.singleagent.learning.lspi.SARSData;
import burlap.behavior.singleagent.planning.OOMDPPlanner;
import burlap.behavior.singleagent.planning.QComputablePlanner;
import burlap.oomdp.core.AbstractGroundedAction;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.State;


public class REGAL extends OOMDPPlanner implements QComputablePlanner{
	
	
	
	protected LinkedList<double[][]> transitionSet;
	protected SARSData data;
	protected Domain domain;
	
	public Policy run(SARSData data){
		this.data=data;
		
		
		
		
	}
	
	public LinkedList<double[][]> getTransitionSet() {
		return transitionSet;U
	}

	public void setTransitionSet(LinkedList<double[][]> transitionSet) {
		this.transitionSet = transitionSet;
	}

	@Override
	public void planFromState(State initialState) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void resetPlannerResults() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public List<QValue> getQs(State s) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public QValue getQ(State s, AbstractGroundedAction a) {
		// TODO Auto-generated method stub
		return null;
	}

}

