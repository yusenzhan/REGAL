package REGAL;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import transitionupdate.StateTransition;
import burlap.behavior.singleagent.Policy;
import burlap.behavior.singleagent.QValue;
import burlap.behavior.singleagent.learning.lspi.SARSData;
import burlap.behavior.singleagent.planning.OOMDPPlanner;
import burlap.behavior.singleagent.planning.QComputablePlanner;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyDeterministicQPolicy;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.behavior.statehashing.StateHashFactory;
import burlap.behavior.statehashing.StateHashTuple;
import burlap.oomdp.core.AbstractGroundedAction;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.State;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.singleagent.RewardFunction;

public class REGAL{

	protected SARSData data;
	protected Domain domain;
	protected State initialState;
	protected List <StateHashTuple>states;
	protected Map<StateHashTuple, StateHashTuple> mapToStateIndex;
	protected Map<StateHashTuple, Integer> mapToIntIndex;
	protected MyVI vi;
	protected TerminalFunction tf;
	protected RewardFunction rf;
	protected StateHashFactory hashingFactory;
	protected StateTransition statetransition;
	protected static double delta = 0.8;
	protected static int maxInteration = 1000;
	protected static double gamma = 1.0;
	protected static double maxDelta = 0.01;
	protected double biasSpanBound;

	/**
	 * Initial the REGAL object
	 * 
	 * @param domain
	 *            input the domain
	 * @param input
	 *            the initial state
	 * 
	 * **/
	public REGAL(Domain domain, State initialstate, TerminalFunction tf, RewardFunction rf,
			StateHashFactory hashingFactory, double h) {
		super();
		this.domain = domain;
		this.tf=tf;
		this.rf=rf;
		this.hashingFactory=hashingFactory;
		this.initialState = initialstate;
		vi = new MyVI(domain, rf, tf, gamma, hashingFactory, maxDelta, maxInteration);
		//System.out.println("Before List");
		this.states = vi.getStateListFrom(this.initialState);
		//System.out.println("After List");
		this.mapToStateIndex =vi.getmapToStateIndex();
		this.mapToIntIndex=new HashMap<StateHashTuple,Integer>();
		this.biasSpanBound = h;

		// put the state to integer index
		int i = 0;
		for (StateHashTuple sa : states) {
			this.mapToIntIndex.put(sa, new Integer(i));
			i++;
		}

		statetransition = new StateTransition(this.states, this.mapToStateIndex, this.mapToIntIndex, vi.getActions(),
				hashingFactory, vi,this.initialState);
		statetransition.getInitialTransitionDynamics();

	}

	public Policy experiment(SARSData data) {

		this.data = data;
		//System.out.println("Data size="+data.dataset.size());

		// pass the data set and transition set to the update class
		this.statetransition.setData(this.data);
		// update transition set
		System.out.println("update TD!");
		this.statetransition.updateTransitionSetOne();
		

		// constructed the constrained transition set
		System.out.println("update CTD!");
		statetransition.updateConstrainedTransitionSet(REGAL.delta);
		

		// Select the best transition probability based on the constraints
		// transition set
	
		
		vi.settransitionDynamics(statetransition.selectTP(this.biasSpanBound));
		

		// run the value iteration from the initial state
		vi.resetPlannerResults();
		vi.setDisplayRounds(true);
		System.out.println("Planning!");
		vi.planFromState(initialState);
		vi.setDisplayRounds(false);

		// create a Q-greedy policy from the planner
		Policy p = new GreedyQPolicy((QComputablePlanner) vi);

		return p;
	}

}
