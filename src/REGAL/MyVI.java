package REGAL;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import burlap.behavior.singleagent.planning.ActionTransitions;
import burlap.behavior.singleagent.planning.HashedTransitionProbability;
import burlap.behavior.singleagent.planning.ValueFunctionPlanner;
import burlap.behavior.statehashing.StateHashFactory;
import burlap.behavior.statehashing.StateHashTuple;
import burlap.debugtools.DPrint;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.State;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.singleagent.RewardFunction;

/**
 * An implementation of asynchronous value iteration. Values of states are
 * updated using the Bellman operator in an arbitrary order and a complete pass
 * over the state space is performed on each iteration. VI can be set to
 * terminate under two possible conditions: when the maximum change in the value
 * function is smaller than some threshold or when a threshold of iterations is
 * passed. This implementation first determines the state space by finding all
 * reachable states from a source state. The worst case time complexity of the
 * reachability operation is equivalent to that of one VI iteration and has the
 * added benefit that VI does not pass over non-reachable states.
 * 
 * This implementation is compatible with options.
 * 
 * 
 * @author James MacGlashan
 *
 */
public class MyVI extends ValueFunctionPlanner {

	/**
	 * When the maximum change in the value function is smaller than this value,
	 * VI will terminate.
	 */
	protected double maxDelta;

	/**
	 * When the number of VI iterations exceeds this value, VI will terminate.
	 */
	protected int maxIterations;

	/**
	 * Indicates whether the reachable states has been computed yet.
	 */
	protected boolean foundReachableStates = false;

	/**
	 * When the reachability analysis to find the state space is performed, a
	 * breadth first search-like pass (spreading over all stochastic
	 * transitions) is performed. It can optionally be set so that the search is
	 * pruned at terminal states by setting this value to true. By default, it
	 * is false and the full reachable state space is found
	 */
	protected boolean stopReachabilityFromTerminalStates = false;

	protected boolean hasRunVI = false;

	protected boolean isDisplayRounds = false;

	/**
	 * When the VI sotps, it stores the number of iterations which is used to
	 * approximate the bias vector
	 * */
	protected int stopRun;

	/*
	 * The map stores the approximated optimal gain for the MDP
	 */
	protected Map<StateHashTuple, Double> optimalGain;

	/**
	 * Initializers the planner.
	 * 
	 * @param domain
	 *            the domain in which to plan
	 * @param rf
	 *            the reward function
	 * @param tf
	 *            the terminal state function
	 * @param gamma
	 *            the discount factor
	 * @param hashingFactory
	 *            the state hashing factor to use
	 * @param maxDelta
	 *            when the maximum change in the value function is smaller than
	 *            this value, VI will terminate.
	 * @param maxIterations
	 *            when the number of VI iterations exceeds this value, VI will
	 *            terminate.
	 */
	public MyVI(Domain domain, RewardFunction rf, TerminalFunction tf, double gamma, StateHashFactory hashingFactory,
			double maxDelta, int maxIterations) {
		// System.out.println("MYVI");
		this.VFPInit(domain, rf, tf, gamma, hashingFactory);
		this.maxDelta = maxDelta;
		this.maxIterations = maxIterations;
		this.optimalGain = new HashMap<StateHashTuple, Double>();

	}

	/**
	 * Calling this method will force the planner to recompute the reachable
	 * states when the {@link #planFromState(State)} method is called next. This
	 * may be useful if the transition dynamics from the last planning call have
	 * changed and if planning needs to be restarted as a result.
	 */
	public void recomputeReachableStates() {
		this.foundReachableStates = false;
		this.transitionDynamics = new HashMap<StateHashTuple, List<ActionTransitions>>();
	}

	/**
	 * Sets whether the state reachability search to generate the state space
	 * will be prune the search from terminal states. The default is not to
	 * prune.
	 * 
	 * @param toggle
	 *            true if the search should prune the search at terminal states;
	 *            false if the search should find all reachable states
	 *            regardless of terminal states.
	 */
	public void toggleReachabiltiyTerminalStatePruning(boolean toggle) {
		this.stopReachabilityFromTerminalStates = toggle;
	}

	@Override
	public void planFromState(State initialState) {
		this.initializeOptionsForExpectationComputations();
		if (this.performReachabilityFrom(initialState) || !this.hasRunVI) {
			this.runVI();
		}

	}

	@Override
	public void resetPlannerResults() {
		// super.resetPlannerResults();
		this.valueFunction.clear();
		// this.foundReachableStates = false;
		this.hasRunVI = false;
		this.stopRun = 0;
	}

	/**
	 * Runs VI until the specified termination conditions are met. In general,
	 * this method should only be called indirectly through the
	 * {@link #planFromState(State)} method. The
	 * {@link #performReachabilityFrom(State)} must have been performed at least
	 * once in the past or a runtime exception will be thrown. The
	 * {@link #planFromState(State)} method will automatically call the
	 * {@link #performReachabilityFrom(State)} method first and then this if it
	 * hasn't been run.
	 */
	public void runVI() {

		if (!this.foundReachableStates) {
			throw new RuntimeException(
					"Cannot run VI until the reachable states have been found. Use the planFromState or performReachabilityFrom method at least once before calling runVI.");
		}

		Set<StateHashTuple> states = mapToStateIndex.keySet();

		int i = 0;
		for (i = 0; i < this.maxIterations; i++) {

			double delta = 0.;
			double minV = 0.;
			this.optimalGain.clear();
			for (StateHashTuple sh : states) {

				double v = this.value(sh);
				double maxQ = this.performBellmanUpdateOn(sh);
				this.optimalGain.put(sh, maxQ - v);
				delta = Math.max(Math.abs(maxQ - v), delta);
				minV = Math.min(Math.abs(maxQ - v), minV);

			}
			/*
			 * if (delta < this.maxDelta) { System.out.println("Stop!! delta=" +
			 * delta); break; // approximated well enough; //stop iterating }
			 */

			// new stop criteria
			if (delta - minV < this.maxDelta) {

				break;
			}

		}

		this.stopRun = i;
		if (this.isDisplayRounds == true) {
			DPrint.cl(this.debugCode, "Passes: " + i);
		}

		this.hasRunVI = true;

	}

	/**
	 * This method will find all reachable states that will be used by the
	 * {@link #runVI()} method and will cache all the transition dynamics. This
	 * method will not do anything if all reachable states from the input state
	 * have been discovered from previous calls to this method.
	 * 
	 * @param si
	 *            the source state from which all reachable states will be found
	 * @return true if a reachability analysis had never been performed from
	 *         this state; false otherwise.
	 */
	public boolean performReachabilityFrom(State si) {

		StateHashTuple sih = this.stateHash(si);
		// if this is not a new state and we are not required to perform a new
		// reachability analysis, then this method does not need to do anything.
		if (mapToStateIndex.containsKey(sih) && this.foundReachableStates) {
			return false; // no need for additional reachability testing
		}

		DPrint.cl(this.debugCode, "Starting reachability analysis");

		// add to the open list
		LinkedList<StateHashTuple> openList = new LinkedList<StateHashTuple>();
		Set<StateHashTuple> openedSet = new HashSet<StateHashTuple>();
		openList.offer(sih);
		openedSet.add(sih);

		while (openList.size() > 0) {
			StateHashTuple sh = openList.poll();

			// skip this if it's already been expanded
			if (mapToStateIndex.containsKey(sh)) {
				continue;
			}

			mapToStateIndex.put(sh, sh);

			// do not need to expand from terminal states if set to prune
			if (this.tf.isTerminal(sh.s) && stopReachabilityFromTerminalStates) {
				continue;
			}

			// get the transition dynamics for each action and queue up new
			// states
			List<ActionTransitions> transitions = this.getActionsTransitions(sh);
			for (ActionTransitions at : transitions) {
				for (HashedTransitionProbability tp : at.transitions) {
					StateHashTuple tsh = tp.sh;
					if (!openedSet.contains(tsh) && !transitionDynamics.containsKey(tsh)) {
						openedSet.add(tsh);
						openList.offer(tsh);
					}
				}

			}

		}

		DPrint.cl(this.debugCode, "Finished reachability analysis; # states: " + mapToStateIndex.size());

		this.foundReachableStates = true;
		this.hasRunVI = false;

		return true;

	}

	/*
	 * get all state from the initial state si
	 * 
	 * @param si is the initial state
	 * 
	 * @return the state set
	 */
	public List<StateHashTuple> getStateListFrom(State si) {

		StateHashTuple sih = this.stateHash(si);

		DPrint.cl(this.debugCode, "Getting the states list");

		// add to the open list
		LinkedList<StateHashTuple> openList = new LinkedList<StateHashTuple>();
		List<StateHashTuple> openedSet = new ArrayList<StateHashTuple>();
		openList.offer(sih);
		openedSet.add(sih);

		while (openList.size() > 0) {
			StateHashTuple sh = openList.poll();

			// skip this if it's already been expanded
			if (mapToStateIndex.containsKey(sh)) {
				continue;
			}

			mapToStateIndex.put(sh, sh);

			// do not need to expand from terminal states if set to prune
			if (this.tf.isTerminal(sh.s) && stopReachabilityFromTerminalStates) {
				continue;
			}

			// get the transition dynamics for each action and queue up new
			// states
			List<ActionTransitions> transitions = this.getActionsTransitions(sh);
			for (ActionTransitions at : transitions) {
				for (HashedTransitionProbability tp : at.transitions) {
					StateHashTuple tsh = tp.sh;
					if (!openedSet.contains(tsh) && !transitionDynamics.containsKey(tsh)) {
						openedSet.add(tsh);
						openList.offer(tsh);
					}
				}

			}

		}

		DPrint.cl(this.debugCode, "The states List is constructed; # states: " + mapToStateIndex.size());

		this.foundReachableStates = true;
		this.hasRunVI = false;

		return openedSet;

	}

	public Map<StateHashTuple, StateHashTuple> getmapToStateIndex() {
		return this.mapToStateIndex;

	}

	public void setmapToStateIndex(Map<StateHashTuple, StateHashTuple> mapToStateIndex) {

		this.mapToStateIndex = mapToStateIndex;
	}

	public Map<StateHashTuple, List<ActionTransitions>> gettransitionDynamics() {
		return this.transitionDynamics;
	}

	public void settransitionDynamics(Map<StateHashTuple, List<ActionTransitions>> transitionDynamics) {

		this.transitionDynamics = transitionDynamics;
	}

	/**
	 * @return the stopRun
	 */
	public int getStopRun() {
		return stopRun;
	}

	/**
	 * @param stopRun
	 *            the stopRun to set
	 */
	public void setStopRun(int stopRun) {
		this.stopRun = stopRun;
	}

	/**
	 * @return the optimalGain
	 */
	public Map<StateHashTuple, Double> getOptimalGain() {
		return optimalGain;
	}

	/**
	 * @param optimalGain
	 *            the optimalGain to set
	 */
	public void setOptimalGain(Map<StateHashTuple, Double> optimalGain) {
		this.optimalGain = optimalGain;
	}

	/**
	 * @return the valueFunction
	 */
	public Map<StateHashTuple, Double> getValueFunction() {
		return valueFunction;
	}

	/**
	 * @param valueFunction
	 *            the valueFunction to set
	 */
	public void setValueFunction(Map<StateHashTuple, Double> valueFunction) {
		this.valueFunction = valueFunction;
	}

	/**
	 * @return the isDisplayRounds
	 */
	public boolean isDisplayRounds() {
		return isDisplayRounds;
	}

	/**
	 * @param isDisplayRounds
	 *            the isDisplayRounds to set
	 */
	public void setDisplayRounds(boolean isDisplayRounds) {
		this.isDisplayRounds = isDisplayRounds;
	}

}