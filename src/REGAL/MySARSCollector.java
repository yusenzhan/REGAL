package REGAL;
//import java.util.List;

import burlap.behavior.singleagent.Policy;
import burlap.behavior.singleagent.learning.lspi.SARSCollector;
import burlap.behavior.singleagent.learning.lspi.SARSData;
//import burlap.debugtools.RandomFactory;
import burlap.oomdp.auxiliary.StateGenerator;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.State;
import burlap.oomdp.core.TerminalFunction;
//import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;

public class MySARSCollector extends SARSCollector {

	protected Policy p;// using policy p to collect data
	//protected Policy teacher; // the teacher policy
	//protected Policy student; // the student policy
	//protected double beta;

	public MySARSCollector(Domain domain) {
		super(domain);
	}

	public MySARSCollector(Domain domain, Policy teacher,Policy student) {
		super(domain);
		//this.teacher = teacher;
		//this.student=student;
		//this.beta=0;//initiate beta
		// TODO Auto-generated constructor stub
	}

	@Override
	public SARSData collectDataFrom(State s, RewardFunction rf, int maxSteps,
			TerminalFunction tf, SARSData intoDataset) {
		// TODO Auto-generated method stub
		return null;
	}
	
	
	/**
	 * 
	 * @param s current state
	 * @param teacher the expert policy to assist the student to learn.
	 * @param student the current student policy derived from REGAL algorithm.
	 * @param beta the probability interpolation to smooth the actions between the teacher and the student policy
	 * @return
	 */
	public GroundedAction getAction(State s, Policy teacher, Policy student, double beta){
		double p=Math.random();
		GroundedAction action;
		
		
		// With probability beta, the action is from the teacher. Otherwise, from the student own policy.
		if(p<=beta){
			//System.out.println("Teacher");
			action=(GroundedAction)teacher.getAction(s);
		}else{
			//System.out.println("Student");
			action=(GroundedAction)student.getAction(s);
		}
		
		return action;
	}
	
	public SARSData collectDataFrom(State s, RewardFunction rf, int maxSteps,
			TerminalFunction tf, SARSData intoDataset, Policy teacher,Policy student,Double beta) {
		
		
		if(intoDataset == null){
			intoDataset = new SARSData();
		}
		
		State curState = s;
		int nsteps = 0;
		while(!tf.isTerminal(curState) && nsteps < maxSteps){
			
			
			//List<GroundedAction> gas = Action.getAllApplicableGroundedActionsFromActionList(this.actions, curState);
			//GroundedAction ga = gas.get(RandomFactory.getMapped(0).nextInt(gas.size()));
			GroundedAction ga=(GroundedAction)getAction(curState,teacher,student,beta);
			State nextState = ga.executeIn(curState);
			double r = rf.reward(curState, ga, nextState);
			intoDataset.add(curState, ga, r, nextState);
			curState = nextState;
			
			nsteps++;
			
		}
		
		
		return intoDataset;
	}

	/**
	 * Collects nSamples of SARS tuples and returns it in a {@link MySARSData} object.
	 * @param sg a state geneator for finding initial state from which data can be collected.
	 * @param rf the reward function that defines the reward received.
	 * @param nSamples the number of SARS samples to collect.
	 * @param maxEpisodeSteps the maximum number of steps that can be taken when rolling out from a state generted by {@link StateGenerator} sg, before a new rollout is started.
	 * @param tf the terminal function that caused a rollout to stop and a new state to be generated from {@link StateGenerator} sg.
	 * @param intoDataset the dataset into which the results will be collected. If null, a new dataset is created.
	 * @param p the policy to navigate the sampling process.
	 * @return the intoDataset object, which is created if it is input as null.
	 */
	public SARSData collectNInstances(StateGenerator sg, RewardFunction rf,
			int nSamples, int maxEpisodeSteps, TerminalFunction tf,
			SARSData intoDataset,Policy teacher,Policy student,Double beta) {

		if (intoDataset == null) {
			intoDataset = new SARSData(nSamples);
		}

		while (nSamples > 0) {
			int maxSteps = Math.min(nSamples, maxEpisodeSteps);
			int oldSize = intoDataset.size();
			this.collectDataFrom(sg.generateState(), rf, maxSteps, tf,
					intoDataset,teacher,student,beta);
			int delta = intoDataset.size() - oldSize;
			nSamples -= delta;
		}

		return intoDataset;

	}
	
	
	public SARSData collectNInstances(State sg, RewardFunction rf,
			int nSamples, int maxEpisodeSteps, TerminalFunction tf,
			SARSData intoDataset,Policy teacher,Policy student,Double beta) {

		if (intoDataset == null) {
			intoDataset = new SARSData(nSamples);
		}

		while (nSamples > 0) {
			int maxSteps = Math.min(nSamples, maxEpisodeSteps);
			int oldSize = intoDataset.size();
			this.collectDataFrom(sg, rf, maxSteps, tf,
					intoDataset,teacher,student,beta);
			int delta = intoDataset.size() - oldSize;
			nSamples -= delta;
		}

		return intoDataset;

	}

}
