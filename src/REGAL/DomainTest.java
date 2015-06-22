package REGAL;

import linearchainmdp.LinearChainMDPDomain;
import linearchainmdp.LinearChainMDPRewardFunction;
import linearchainmdp.LinearChainMDPTerminalFunction;
import burlap.behavior.singleagent.Policy;
import burlap.behavior.singleagent.planning.OOMDPPlanner;
import burlap.behavior.singleagent.planning.QComputablePlanner;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.behavior.statehashing.DiscreteStateHashFactory;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.State;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.singleagent.RewardFunction;

public class DomainTest {

	public static void main(String[] args) {
		LinearChainMDPDomain gwdg = new LinearChainMDPDomain(50, 0.5);
		// gwdg.setDeterministicTransitionDynamics();
		Domain domain = gwdg.generateDomain();

		// define the task
		RewardFunction rf = new LinearChainMDPRewardFunction(50);
		// TerminalFunction tf = new NullTerminalFunction();
		TerminalFunction tf = new LinearChainMDPTerminalFunction(50, true);

		State initialState = LinearChainMDPDomain.getState(domain);

		// set up the state hashing system
		DiscreteStateHashFactory hashingFactory = new DiscreteStateHashFactory();
		hashingFactory.setAttributesForClass(LinearChainMDPDomain.CLASSAGENT,
				domain.getObjectClass(LinearChainMDPDomain.CLASSAGENT).attributeList);

		// construct the teacher
		OOMDPPlanner planner = new MyVI(domain, rf, tf, 1, hashingFactory, 0.001, 20000);
		planner.planFromState(initialState);

		// create a Q-greedy policy from the planner
		Policy teacher = new GreedyQPolicy((QComputablePlanner) planner);
		
		// record the plan results to a file
		teacher.evaluateBehavior(initialState, rf, tf);
		

	}

}
