import java.util.List;
import java.util.Map;
import java.util.Set;

import transitionupdate.StateTransition;
import burlap.behavior.singleagent.Policy;
import burlap.behavior.singleagent.QValue;
import burlap.behavior.singleagent.learning.lspi.SARSData;
import burlap.behavior.singleagent.planning.OOMDPPlanner;
import burlap.behavior.singleagent.planning.QComputablePlanner;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.behavior.statehashing.StateHashTuple;
import burlap.oomdp.core.AbstractGroundedAction;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.State;

public class REGAL extends OOMDPPlanner implements QComputablePlanner {

	protected SARSData data;
	protected Domain domain;
	protected State initialState;
	protected Set<StateHashTuple> states;
	protected Map<StateHashTuple, StateHashTuple> mapToStateIndex;
	protected Map<StateHashTuple, Integer> mapToIntIndex;
	protected MyVI vi;
	protected StateTransition statetransition;
	
	
	/**
	 * Initial the REGAL object
	 * @param domain input the domain
	 * @param input the initial state
	 * 
	 * **/
	public REGAL(Domain domain, State initialstate) {
		super();
		this.domain = domain;
		this.initialState = initialstate;
		vi = new MyVI(domain, rf, tf, 1, hashingFactory, 0.001, 100);
		states = vi.getStateListFrom(this.initialState);
		this.mapToStateIndex = vi.getmapToStateIndex();

		// put the state to integer index
		int i = 0;
		for (StateHashTuple sa : states) {
			this.mapToIntIndex.put(sa, new Integer(i));
			i++;
		}

		statetransition = new StateTransition(this.states,
				this.mapToStateIndex, this.mapToIntIndex, vi.getActions(),
				vi.getHashingFactory());

	}
	

	public Policy run(SARSData data) {

		this.data = data;

		// pass the data set and transition set to the update class
		this.statetransition.setData(data);
		// update transition set
		statetransition.updateTransitionSet();

		// constructed the constrained transition set
		statetransition.updateConstrainedTransitionSet();

		// Select the best transition probability based on the constraints transition set
		vi.resetPlannerResults();
		vi.settransitionDynamics(statetransition.selectTP());
		
		//run the value iteration from the initial state
		vi.planFromState(initialState);
		
		//create a Q-greedy policy from the planner
		Policy p = new GreedyQPolicy((QComputablePlanner)vi);
		
		return p;
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
