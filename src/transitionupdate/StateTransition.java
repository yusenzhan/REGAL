package transitionupdate;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import REGAL.MyVI;
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
	protected Map<Integer, StateHashTuple> mapIndexToState;
	protected Map<Action, Integer> mapActionToIntIndex;
	protected List<Action> actions;
	protected StateHashFactory hashingFactory;
	protected List<Map<StateHashTuple, List<ActionTransitions>>> transitionList;
	protected List<Map<StateHashTuple, List<ActionTransitions>>> transitionCountList;
	protected List<Map<StateHashTuple, List<ActionTransitions>>> constrainedtransitionList;
	protected List<Map<StateHashTuple, List<ActionTransitions>>> constrainedtransitionCountList;
	protected List<double[][][]> transitionDoubleList;
	protected List<double[][][]> transitionDoubleCountList;
	protected Map<StateHashTuple, List<ActionTransitions>> InitialTD;
	protected MyVI vi;

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
			StateHashFactory hashingFactory, MyVI vi) {
		this.data = null;
		this.states = states;
		this.mapToIntIndex = mapToIntIndex;
		this.actions = actions;
		this.hashingFactory = hashingFactory;
		this.transitionList = new ArrayList<Map<StateHashTuple, List<ActionTransitions>>>();
		this.transitionCountList = new ArrayList<Map<StateHashTuple, List<ActionTransitions>>>();
		this.transitionDoubleList = new ArrayList<double[][][]>();
		this.transitionDoubleCountList = new ArrayList<double[][][]>();
		this.constrainedtransitionList.clear();
		this.constrainedtransitionCountList.clear();
		this.InitialTD = null;
		this.vi = vi;

		// initilized the map from index to state
		for (StateHashTuple s : states) {
			this.mapIndexToState.put(this.mapToIntIndex.get(s), s);

		}

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
	 * update the transition set according to the data and transitions class
	 * **/
	public void updateTransitionSet() {
		// if list is empty, we need to initialized it.
		if (this.transitionCountList.isEmpty()
				&& this.transitionCountList.isEmpty()) {

			this.transitionList.add(InitialTD);
			this.transitionCountList.add(InitialTD);
		}

		// update the transition set
		// set the base as the last element of the list
		Map<StateHashTuple, List<ActionTransitions>> tempCountTD = new HashMap<StateHashTuple, List<ActionTransitions>>(
				this.transitionCountList.get(this.transitionCountList.size() - 1));
		Map<StateHashTuple, List<ActionTransitions>> tempTD = new HashMap<StateHashTuple, List<ActionTransitions>>(
				this.transitionList.get(this.transitionList.size() - 1));
		for (SARS sars : data.dataset) {
			StateHashTuple sh = this.hashingFactory.hashState(sars.s);
			StateHashTuple shp = this.hashingFactory.hashState(sars.sp);
			List<ActionTransitions> tempCountATs = tempCountTD.get(sh);
			// find the transitions by action
			for (int i = 0; i < tempCountATs.size(); i++) {
				ActionTransitions tempCountAT = tempCountATs.get(i);
				if (tempCountAT.matchingTransitions(sars.a)) {

					// update the counting matrix
					double counter = 0.0;
					for (int j = 0; j < tempCountAT.transitions.size(); i++) {
						HashedTransitionProbability tempCountHTP = tempCountAT.transitions
								.get(j);
						counter += tempCountHTP.p;
						if (tempCountHTP.sh.equals(shp)) {
							tempCountHTP.p += 1.0;
						}
					}

					// update the probability transitions
					for (int k = 0; k < tempCountAT.transitions.size(); k++) {
						HashedTransitionProbability tempCountHTP = tempCountAT.transitions
								.get(k);
						HashedTransitionProbability tempHTP = tempTD.get(sh)
								.get(i).transitions.get(k);
						tempHTP.p = tempCountHTP.p / counter;
					}

				}

			}

		}

	}


	/**
	 * update the constrained transition set by the current transition set
	 * **/
	public void updateConstrainedTransitionSet(double delta) {

		// make sure the constrained lists are empty
		this.constrainedtransitionList.clear();
		this.constrainedtransitionCountList.clear();

		// get the last update for the MPD transitions
		Map<StateHashTuple, List<ActionTransitions>> finalCountTDs = new HashMap<StateHashTuple, List<ActionTransitions>>(
				this.transitionCountList.get(this.transitionCountList.size() - 1));
		Map<StateHashTuple, List<ActionTransitions>> finalTDs = new HashMap<StateHashTuple, List<ActionTransitions>>(
				this.transitionList.get(this.transitionList.size() - 1));

		double temp1 = 12
				* states.size()
				* Math.log(2.0 * actions.size() * this.transitionList.size()
						/ delta);
		// calculate the L1-norm equation (2). Scan every MDP in the list to
		// construct the constrained set
		for (int i = 0; i < this.transitionList.size(); i++) {
			boolean flag = true; // indicating that whether or not the MDP
									// satisfies the constrained
			Map<StateHashTuple, List<ActionTransitions>> tempCountTDs = new HashMap<StateHashTuple, List<ActionTransitions>>(
					this.transitionCountList.get(i));
			Map<StateHashTuple, List<ActionTransitions>> tempTDs = new HashMap<StateHashTuple, List<ActionTransitions>>(
					this.transitionList.get(i));

			for (StateHashTuple sh : states) {
				// for the last MDP estimation
				List<ActionTransitions> finalActionCounts = finalCountTDs
						.get(sh);
				List<ActionTransitions> finalActionTransitions = finalTDs
						.get(sh);
				// for the current estimation
				List<ActionTransitions> actionCounts = tempCountTDs.get(sh);
				List<ActionTransitions> actionTransitions = tempTDs.get(sh);

				for (int j = 0; j < actionTransitions.size(); j++) {
					// for the last MDP estimation
					ActionTransitions finalActionCount = finalActionCounts
							.get(j);
					ActionTransitions finalActionTransition = finalActionTransitions
							.get(j);
					// for the current estimation
					ActionTransitions actionCount = actionCounts.get(j);
					ActionTransitions actionTransition = actionTransitions
							.get(j);

					double nsa = 0.0;
					double sumsa = 0.0;
					for (int k = 0; k < actionCount.transitions.size(); k++) {

						// calculate N(s,a:t)
						nsa += finalActionCount.transitions.get(k).p;

						// calculate L1 norm
						sumsa += Math.abs(finalActionTransition.transitions
								.get(k).p
								- actionTransition.transitions.get(k).p);

					}
					// max operator between N(s,a;t) and 1
					if (nsa < 1) {
						nsa = 1.0;
					}

					double upperbound = Math.sqrt(temp1 / nsa);

					if (sumsa > upperbound) {// The MDP is not in the
												// constrained set
						flag = false;
						break;
					}

				}
				// The MDP is not in the constrained set
				if (flag == false) {
					break;
				}

			}

			// the MDP is in the constrained set and add it into the constrained
			// set
			if (flag == true) {
				this.constrainedtransitionList.add(tempTDs);
				this.constrainedtransitionCountList.add(tempCountTDs);
			}
		}

	}

	/**
	 * @return return the best transition dynamics according to the optimization
	 *         over bias and gain
	 * **/

	public Map<StateHashTuple, List<ActionTransitions>> selectTP(double h) {
		double spanbound=h;
		double optimalGain=0.;
		Map<StateHashTuple, List<ActionTransitions>> bestTP=null;
		for (int i = 0; i < this.constrainedtransitionList.size(); i++) {
			Map<StateHashTuple, List<ActionTransitions>> tempTDs = new HashMap<StateHashTuple, List<ActionTransitions>>(
					this.transitionList.get(i));
			vi.resetPlannerResults();
			vi.settransitionDynamics(tempTDs);
			vi.runVI();
			
			Map<StateHashTuple, Double> bias=this.calcBias(vi.getOptimalGain(), vi.getValueFunction(), vi.getStopRun());
			double span=this.calcSpan(bias);
			double gain=this.calcGain(vi.getOptimalGain());
			
			if(span>spanbound){//skip the estimation since it violate the upper bound
				break;
				
			}else{
				
				if(optimalGain<=gain){
					
					optimalGain=gain;
					bestTP=tempTDs;//Choosing the latest TDs as the return TDs
				}		
				
			}
			

		}
		return bestTP;
	}

	/**
	 * @return return the optimal gain by the vi
	 * **/
	public double calcGain(Map<StateHashTuple, Double> optimalGain) {
		double gain = 0;
		for (StateHashTuple s : this.states) {
			if(optimalGain.get(s)<gain){
				gain=optimalGain.get(s);
			}
		}
		return gain;
	}

	/*
	 * @return calculate the bias vector and return it
	 */
	public Map<StateHashTuple, Double> calcBias(
			Map<StateHashTuple, Double> optimalGain,
			Map<StateHashTuple, Double> valueFunction, int stopRun) {

		Map<StateHashTuple, Double> bias = new HashMap<StateHashTuple, Double>();
		for (StateHashTuple s : this.states) {

			bias.put(s, valueFunction.get(s) - stopRun * optimalGain.get(s));
		}

		return bias;
	}

	/**
	 * @param input the bias vector
	 *            
	 * @return calculate the
	 */
	public double calcSpan(Map<StateHashTuple, Double> bias) {
		double min = 0.;
		double max = 0.;
		for (StateHashTuple s : this.states) {
			if (bias.get(s) < min) {
				min = bias.get(s);
			}

			if (bias.get(s) > max) {
				max = bias.get(s);
			}

		}

		return max - min;
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
