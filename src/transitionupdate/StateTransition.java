package transitionupdate;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import burlap.behavior.singleagent.learning.lspi.SARSData;
import burlap.behavior.singleagent.learning.lspi.SARSData.SARS;
import burlap.behavior.singleagent.planning.ActionTransitions;
import burlap.behavior.singleagent.planning.HashedTransitionProbability;
import burlap.behavior.statehashing.StateHashFactory;
import burlap.behavior.statehashing.StateHashTuple;
import burlap.oomdp.core.TransitionProbability;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.GroundedAction;

/**
 * the class for updating the transition probability
 * 
 * **/
public class StateTransition {

	protected SARSData data;
	protected Set<StateHashTuple> states;
	protected Map<StateHashTuple, StateHashTuple> mapToStateIndex;
	protected Map<StateHashTuple, Integer> mapToIntIndex;
	protected List<Action> actions;
	protected StateHashFactory hashingFactory;
	protected List<Map<StateHashTuple, List<ActionTransitions>>> transitionList;
	protected List<Map<StateHashTuple, List<ActionTransitions>>> transitionCountList;
	protected Map<StateHashTuple, List<ActionTransitions>> InitialTD;

	/**
	 * constructor
	 *
	 * @param data
	 *            input data
	 * @param transitionSet
	 *            input transition set in which the transition dynamics will be
	 *            updated according to the data
	 ***/
	public StateTransition(Set<StateHashTuple> states,
			Map<StateHashTuple, StateHashTuple> mapToStateIndex,
			Map<StateHashTuple, Integer> mapToIntIndex, List<Action> actions,
			StateHashFactory hashingFactory) {
		this.data = null;
		this.states = states;
		this.mapToIntIndex = mapToIntIndex;
		this.actions = actions;
		this.hashingFactory = hashingFactory;
		this.transitionList = new ArrayList<Map<StateHashTuple, List<ActionTransitions>>>();
		this.transitionCountList = new ArrayList<Map<StateHashTuple, List<ActionTransitions>>>();
		this.InitialTD = null;
	}

	/**
	 * @return the initial transition dynamics in which transitions are zero
	 * **/
	public Map<StateHashTuple, List<ActionTransitions>> getInitialTransitionDynamics() {

		Map<StateHashTuple, List<ActionTransitions>> transitionDynamics = new HashMap<StateHashTuple, List<ActionTransitions>>(
				states.size());
		for (StateHashTuple sh : states) {
			// get all possible actions at state sh
			List<GroundedAction> gas = Action
					.getAllApplicableGroundedActionsFromActionList(
							this.actions, sh.s);
			// set all possible action transition as zero
			List<TransitionProbability> tps = new ArrayList<TransitionProbability>(
					states.size());
			for (StateHashTuple psh : states) {
				tps.add(new TransitionProbability(psh.s, 0.0));
			}

			List<ActionTransitions> allTransitions = new ArrayList<ActionTransitions>(
					gas.size());
			for (GroundedAction ga : gas) {
				ActionTransitions at = new ActionTransitions(ga, tps,
						hashingFactory);
				allTransitions.add(at);
			}

			// store the transitions
			transitionDynamics.put(sh, allTransitions);
		}

		InitialTD = transitionDynamics;
		return transitionDynamics;

	}

	/**
	 * update the transition set according to the data and transitions
	 * **/
	public void updateTransitionSet() {
		// if map is empty, we need to initalized it.
		if (this.transitionCountList.isEmpty()) {
			
			this.transitionList.add(InitialTD);
			this.transitionCountList.add(InitialTD);
		} else {// update the transition set
				// set the base as the last element of the list
			Map<StateHashTuple, List<ActionTransitions>> tempCountTD = new HashMap<StateHashTuple, List<ActionTransitions>>(
					this.transitionList.get(this.transitionList.size() - 1));
			for (SARS sars : data.dataset) {
				StateHashTuple sh = this.hashingFactory.hashState(sars.s);
				StateHashTuple shp = this.hashingFactory.hashState(sars.sp);
				List<ActionTransitions> tempCountAT = tempCountTD.get(sh);
				// find the transitions by action
				for (ActionTransitions ats : tempCountAT) {
					if (ats.matchingTransitions(sars.a)) {
						for (HashedTransitionProbability htp : ats.transitions) {
							if (htp.sh.equals(shp)) {
								htp.p+=1.0;
							}

						}
					}

				}

			}

		}

	}

	/**
	 * update the constrained transition set by the current transition set
	 * **/
	public void updateConstrainedTransitionSet() {

	}

	/**
	 * @return return the best transition dynamics according to the optimization
	 *         over bias and gain
	 * **/

	public Map<StateHashTuple, List<ActionTransitions>> selectTP() {

		return null;
	}

	/**
	 * @return the data
	 */
	public SARSData getData() {
		return data;
	}

	/**
	 * @param data
	 *            the data to set
	 */
	public void setData(SARSData data) {
		this.data = data;
	}

}
