package linearchainmdp;

import java.util.ArrayList;
import java.util.List;

import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.core.Attribute;
import burlap.oomdp.core.Attribute.AttributeType;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.ObjectClass;
import burlap.oomdp.core.ObjectInstance;
import burlap.oomdp.core.PropositionalFunction;
import burlap.oomdp.core.State;
import burlap.oomdp.core.TransitionProbability;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.SADomain;

public class LinearChainMDPDomain implements DomainGenerator {

	public static final String ATTX = "x";

	public static final String CLASSAGENT = "agent";
	public static final String CLASSLOCATION = "location";
	
	public static final String ACTIONFORWARD = "forward";
	public static final String ACTIONBACKWARD= "backword";
	
	public static final String PFAT = "at";
	

	/*
	 * 
	 * The length of the chain
	 */
	protected int chainLength;

	/**
	 * the transition dynamic for the final state
	 * 
	 */
	protected double finalStateTransitionDy;

	public LinearChainMDPDomain(int chainLength, double finalStateTransitionDy) {

		this.chainLength = chainLength;
		this.finalStateTransitionDy=finalStateTransitionDy;
	}

	@Override
	public Domain generateDomain() {

		SADomain domain = new SADomain();

		Attribute xatt = new Attribute(domain, ATTX, AttributeType.INT);
		xatt.setLims(0, 10);

		ObjectClass agentClass = new ObjectClass(domain, CLASSAGENT);
		agentClass.addAttribute(xatt);

		ObjectClass locationClass = new ObjectClass(domain, CLASSLOCATION);
		locationClass.addAttribute(xatt);
		
		new Movement(ACTIONFORWARD, domain, 0,this.chainLength,this.finalStateTransitionDy);
		new Movement(ACTIONBACKWARD, domain, 1,this.chainLength,this.finalStateTransitionDy);
		
		new AtLocation(domain);

		return domain;
	}

	protected class Movement extends Action {

		// 0: move forward 1: move backward
		protected double[] directionProbs = new double[2];

		/*
		 * 
		 * The length of the chain
		 */
		protected int chainLength;

		/**
		 * the transition dynamic for the final state
		 * 
		 */
		protected double finalStateTransitionDy;

		public Movement(String actionName, Domain domain, int direction, int chainLength, double finalStateTransitionDy) {
			super(actionName, domain, "");
			for (int i = 0; i < 2; i++) {
				directionProbs[i] = 1.0;
			}
			this.chainLength = chainLength;

			this.finalStateTransitionDy = finalStateTransitionDy;
		}

		@Override
		protected State performActionHelper(State s, String[] params) {

			// get agent and current position
			ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
			int curX = agent.getDiscValForAttribute(ATTX);

			// sample direction with random roll
			double r = Math.random();
			double sumProb = 0.;
			int dir = 0;
			for (int i = 0; i < this.directionProbs.length; i++) {
				sumProb += this.directionProbs[i];
				if (r < sumProb) {
					dir = i;
					break; // found direction
				}
			}

			// get resulting position
			int newPos = this.moveResult(curX, dir);

			// set the new position
			agent.setValue(ATTX, newPos);

			// return the state we just modified
			return s;
		}

		@Override
		public List<TransitionProbability> getTransitions(State s, String[] params) {

			// get agent and current position
			ObjectInstance agent = s.getFirstObjectOfClass(CLASSAGENT);
			int curX = agent.getDiscValForAttribute(ATTX);

			List<TransitionProbability> tps = new ArrayList<TransitionProbability>(2);
			TransitionProbability noChangeTransition = null;

			for (int i = 0; i < this.directionProbs.length; i++) {
				int newPos = this.moveResult(curX, i);
				// the agent is in the goal state
				if (newPos == this.chainLength - 1) {
					// new possible outcome
					State ns = s.copy();
					ObjectInstance nagent = ns.getFirstObjectOfClass(CLASSAGENT);
					nagent.setValue(ATTX, newPos);

					// create transition probability object and add to our list
					// of outcomes
					if (i == 0) {
						tps.add(new TransitionProbability(ns, this.finalStateTransitionDy));
					} else {
						tps.add(new TransitionProbability(ns, 1-this.finalStateTransitionDy));
					}
				} else if (newPos != curX) {
					// new possible outcome
					State ns = s.copy();
					ObjectInstance nagent = ns.getFirstObjectOfClass(CLASSAGENT);
					nagent.setValue(ATTX, newPos);

					// create transition probability object and add to our list
					// of outcomes
					tps.add(new TransitionProbability(ns, this.directionProbs[i]));
				} else {
					// this direction didn't lead anywhere new
					// if there are existing possible directions
					// that wouldn't lead anywhere, aggregate with them
					if (noChangeTransition != null) {
						noChangeTransition.p += this.directionProbs[i];
					} else {
						// otherwise create this new state and transition
						noChangeTransition = new TransitionProbability(s.copy(), this.directionProbs[i]);
						tps.add(noChangeTransition);
					}
				}
			}

			return tps;
		}

		protected int moveResult(int curX, int direction) {
			// first get change in x from direction using 0: forward;
			// 1:backward
			int xdelta = 0;
			// if curX is in the final state
			if (curX == this.chainLength - 1) {
				if (direction == 1) {
					xdelta = -curX;
				}else if(Math.random()>this.finalStateTransitionDy){
					xdelta = -curX;	
				}else{
					xdelta = 0;
				}

			} else {
				
				if (direction == 0) {
					xdelta = 1;
				} else {// The agent will move back to the start point
					xdelta = -curX;
				}
			}

			int nx = curX + xdelta;

			// make sure new position is valid (not a wall or off bounds)
			if (nx < 0 || nx >= chainLength || chainLength == 1) {
				nx = curX;
			}

			return nx;

		}
	}
	
	protected class AtLocation extends PropositionalFunction{

		public AtLocation(Domain domain){
			super(PFAT, domain, new String []{CLASSAGENT,CLASSLOCATION});
		}
		
		@Override
		public boolean isTrue(State s, String[] params) {
			ObjectInstance agent = s.getObject(params[0]);
			ObjectInstance location = s.getObject(params[1]);
			
			int ax = agent.getDiscValForAttribute(ATTX);
			
			int lx = location.getDiscValForAttribute(ATTX);
			
			return ax == lx;
		}
				
		
	}

}
