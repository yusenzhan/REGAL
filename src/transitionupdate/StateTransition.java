package transitionupdate;

import java.util.List;

import burlap.behavior.singleagent.learning.lspi.SARSData;
import burlap.behavior.singleagent.learning.lspi.SARSData.SARS;


/**
 * abstrct class for updating the transition probability
 * 
 * **/
public abstract class StateTransition {
	
	protected SARSData data;
	protected List<double[][]> transitionSet;
	
	/**constructor
	 *
	 *@param data input data
	 *@param transitionSet input transition set in which the transition dynamics will be updated according to the data
	 ***/
	StateTransition(SARSData data, List<double[][]> transitionSet){
		
		this.transitionSet=transitionSet;
		this.data=data;
	}
	
	
	/**
	 * abstract method for the domain to update its transition set.
	 * **/
	public abstract List<double[][]> updateTransitionSet();

	public SARSData getData() {
		return data;
	}

	public void setData(SARSData data) {
		this.data = data;
	}

	public List<double[][]> getTransitionSet() {
		return transitionSet;
	}

	public void setTransitionSet(List<double[][]> transitionSet) {
		this.transitionSet = transitionSet;
	}

	
	

}
