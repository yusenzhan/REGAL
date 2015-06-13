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
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.singleagent.RewardFunction;

public class REGAL extends OOMDPPlanner implements QComputablePlanner {

	protected SARSData data;
	protected Domain domain;
	protected State initialState;
	protected Set<StateHashTuple> states;
	protected Map<StateHashTuple, StateHashTuple> mapToStateIndex;
	protected Map<StateHashTuple, Integer> mapToIntIndex;
	protected MyVI vi;
	protected TerminalFunction tf;
	protected RewardFunction rf;
	protected StateTransition statetransition;
	protected static double delta=0.8;
	protected static int maxInteration=1000;
	protected static double gamma=1.0;
	protected static double maxDelta=0.01;
	
	/**
	 * Initial the REGAL object
	 * @param domain input the domain
	 * @param input the initial state
	 * 
	 * **/
	public REGAL(Domain domain, State initialstate, TerminalFunction tf,RewardFunction rf) {
		super();
		this.domain = domain;
		this.initialState = initialstate;
		vi = new MyVI(domain, rf, tf, gamma, hashingFactory, maxDelta, maxInteration);
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
	

	public Policy experiment(SARSData data){ 

		this.data = data;

		// pass the data set and transition set to the update class
		this.statetransition.setData(this.data);
		// update transition set
		this.statetransition.updateTransitionSet();

		// constructed the constrained transition set
		statetransition.updateConstrainedTransitionSet(this.delta);

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
