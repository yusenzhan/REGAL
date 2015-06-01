import java.util.LinkedList;
import java.util.List;

import transitionupdate.StateTransition;
import burlap.behavior.singleagent.Policy;
import burlap.behavior.singleagent.QValue;
import burlap.behavior.singleagent.learning.lspi.SARSData;
import burlap.behavior.singleagent.planning.OOMDPPlanner;
import burlap.behavior.singleagent.planning.QComputablePlanner;
import burlap.oomdp.auxiliary.StateParser;
import burlap.oomdp.core.AbstractGroundedAction;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.State;



public class REGAL extends OOMDPPlanner implements QComputablePlanner{
	
	
	
	protected List<double[][]> transitionset;
	protected List<double[][]> constrainedtransitionset;
	protected SARSData data;
	protected Domain domain;
	protected StateParser stateparser;
	protected StateTransition statetransition;
	
	public Policy run(SARSData data){
		this.data=data;
		
		//pass the data set and transition set to the update class
		this.statetransition.setData(data);
		this.statetransition.setTransitionSet(transitionset);
		//get the new transition set
		transitionset=statetransition.getTransitionSet();
		
		//constructed the constrained transition set
		this.updateConstrainedTransitionSet();
		
		
		//Select the best transition dynamics based on the constraints
		double[][] max=maxGain();
		
		
	}
	
	
	/**
	 * max the gain by the regularization
	 * */
	public double[][] maxGain(){
		
		
		double[][] max=this.constrainedtransitionset.get(0);
		
		for(double[][] i:constrainedtransitionset){
			
			
			
		}
		
		return max;
		
	}
	
	public void updateConstrainedTransitionSet(){
		this.constrainedtransitionset.clear();	
		
	}
	
	/**
	 * @param data input data
	 * **/
	public void updateTrans(SARSData data){
		
		
	}
	
	public List<double[][]> getTransitionSet() {
		return transitionset;
	}

	public void setTransitionSet(List<double[][]> transitionSet) {
		this.transitionset = transitionSet;
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

