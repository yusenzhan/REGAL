package REGAL;

import java.awt.Color;
import java.util.List;
import java.util.Random;

import domain.GWRandomStateGenerator;
import domain.NullTerminalFunction;
import burlap.behavior.singleagent.Policy;
import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.ArrowActionGlyph;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.LandmarkColorBlendInterpolation;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.StateValuePainter2D;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D.PolicyGlyphRenderStyle;
//import burlap.behavior.singleagent.learning.lspi.SARSCollector;
import burlap.behavior.singleagent.learning.lspi.SARSData;
import burlap.behavior.singleagent.planning.OOMDPPlanner;
import burlap.behavior.singleagent.planning.QComputablePlanner;
import burlap.behavior.singleagent.planning.StateConditionTest;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.behavior.singleagent.planning.deterministic.TFGoalCondition;
import burlap.behavior.statehashing.DiscreteStateHashFactory;
import burlap.behavior.statehashing.StateHashFactory;
import burlap.debugtools.RandomFactory;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.domain.singleagent.gridworld.GridWorldStateParser;
import burlap.domain.singleagent.gridworld.GridWorldVisualizer;
import burlap.oomdp.auxiliary.StateGenerator;
//import burlap.oomdp.auxiliary.StateGenerator;
import burlap.oomdp.auxiliary.StateParser;
import burlap.oomdp.auxiliary.common.RandomStartStateGenerator;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.State;
//import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.SADomain;
import burlap.oomdp.singleagent.common.SinglePFTF;
//import burlap.oomdp.singleagent.common.SinglePFTF;
import burlap.oomdp.singleagent.common.UniformCostRF;
import burlap.oomdp.singleagent.common.VisualActionObserver;

public class DAGGERLearning {

	protected Domain domain; // domain
	protected TerminalFunction tf;
	protected RewardFunction rf;
	protected State initialState;
	protected StateHashFactory hashingFactory;
	protected StateGenerator sg;

	protected REGAL regal; // REGAL algorithm
	protected Policy teacher; // the teacher policy
	protected Policy student; // the student policy
	protected int currentTimeStep; // current time step
	protected int maxInteration;// max iteration for the data aggregation
	protected int maxSteps;
	protected double H; // the upper bound of the span of all MDPs
	protected double p; // probability seed

	/**
	 * @param domain
	 * @param tf
	 * @param rf
	 * @param initialState
	 * @param regal
	 * @param teacher
	 * @param student
	 * @param maxInteration
	 * @param maxSteps
	 * @param h
	 * @param p
	 */
	public DAGGERLearning(Domain domain, TerminalFunction tf, RewardFunction rf, State initialState,
			StateHashFactory hashingFactory, Policy teacher, Policy student, int maxInteration, int maxSteps, double h,
			double p) {
		super();
		this.domain = domain;
		this.tf = tf;
		this.rf = rf;
		this.hashingFactory = hashingFactory;
		this.initialState = initialState;
		this.H = h;
		this.regal = new REGAL(domain, initialState, tf, rf, hashingFactory, H);
		this.teacher = teacher;
		this.student = student;
		this.maxInteration = maxInteration;
		this.maxSteps = maxSteps;
		this.p = p;
		this.currentTimeStep = 0;// current time is always 0
		this.sg = new RandomStartStateGenerator((SADomain)this.domain,this.initialState);
	}

	/**
	 * Training the REGAL algorithm and the policy the member student.
	 */

	public void train() {
		MySARSCollector collector = new MySARSCollector(domain);
		SARSData data = null;
		// train the student
		for (int i = 0; i < this.maxInteration; i++) {
			System.out.println("This is interation " + i + " DAGGER");
			double beta = getBetai(i, p);
			// data = collector.collectDataFrom(this.initialState, this.rf,
			// this.maxSteps, this.tf, null, this.teacher, this.student, beta);
			// data = collector.collectNInstances(this.initialState, this.rf,
			// maxSteps, maxSteps, this.tf, null, teacher,
			// student, beta);
			data = collector.collectNInstances(sg, this.rf, maxSteps, 50, this.tf, null, teacher, student, beta);
			System.out.println(data.dataset.size());
			student = regal.experiment(data);
		}

	}

	/**
	 * @param i
	 *            the current iteration
	 * @param p
	 *            the probability seed in [0,1]
	 * @return the p^i is the beta decaying exponentially.
	 */

	public double getBetai(int i, double p) {

		return Math.pow(p, i);

	}

	public void valueFunctionVisualize(QComputablePlanner planner, Policy p) {
		List<State> allStates = StateReachability.getReachableStates(initialState, (SADomain) domain, hashingFactory);
		LandmarkColorBlendInterpolation rb = new LandmarkColorBlendInterpolation();
		rb.addNextLandMark(0., Color.RED);
		rb.addNextLandMark(1., Color.BLUE);

		StateValuePainter2D svp = new StateValuePainter2D(rb);
		svp.setXYAttByObjectClass(GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTX, GridWorldDomain.CLASSAGENT,
				GridWorldDomain.ATTY);

		PolicyGlyphPainter2D spp = new PolicyGlyphPainter2D();
		spp.setXYAttByObjectClass(GridWorldDomain.CLASSAGENT, GridWorldDomain.ATTX, GridWorldDomain.CLASSAGENT,
				GridWorldDomain.ATTY);
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONNORTH, new ArrowActionGlyph(0));
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONSOUTH, new ArrowActionGlyph(1));
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONEAST, new ArrowActionGlyph(2));
		spp.setActionNameGlyphPainter(GridWorldDomain.ACTIONWEST, new ArrowActionGlyph(3));
		spp.setRenderStyle(PolicyGlyphRenderStyle.DISTSCALED);

		ValueFunctionVisualizerGUI gui = new ValueFunctionVisualizerGUI(allStates, svp, planner);
		gui.setSpp(spp);
		gui.setPolicy(p);
		gui.setBgColor(Color.GRAY);
		gui.initGUI();
	}

	/**
	 * @return the student
	 */
	public Policy getStudent() {
		return student;
	}

	/**
	 * @param student
	 *            the student to set
	 */
	public void setStudent(Policy student) {
		this.student = student;
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		// create the domain
		GridWorldDomain gwdg = new GridWorldDomain(15, 15);
		gwdg.setMapToFourRooms();
		gwdg.setDeterministicTransitionDynamics();
		/*int [][] map = new int[][]{
				{0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,1,0,0,1,1,1,0,0,0,0},
				{0,0,0,0,0,1,0,0,0,0,0,1,0,0,0},
				{0,0,0,0,0,1,0,0,0,0,0,0,1,0,0},
				{0,0,0,0,0,1,0,0,0,0,0,0,1,0,0},
				{1,0,1,1,1,1,1,1,0,1,1,1,1,1,1},
				{0,0,0,0,1,0,0,1,0,0,0,0,0,0,0},
				{0,0,0,0,1,0,0,1,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
				{0,0,0,0,1,0,0,1,0,0,0,0,0,0,0},
				{0,0,0,0,1,0,0,1,0,0,0,0,0,0,0},
				{0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
				{0,0,0,0,1,0,0,1,0,0,0,0,0,0,0},
				{0,0,0,0,1,0,0,1,0,0,0,0,0,0,0},
				{0,0,0,0,1,0,0,1,0,0,0,0,0,0,0},
		};
		gwdg.setMap(map);*/
		Domain domain = gwdg.generateDomain();

		// create the state parser
		StateParser sp = new GridWorldStateParser(domain);

		// StateGenerator rStateGen = new MCRandomStateGenerator(domain);

		// define the task
		RewardFunction rf = new UniformCostRF();
		// TerminalFunction tf = new NullTerminalFunction();
		TerminalFunction tf = new SinglePFTF(domain.getPropFunction(GridWorldDomain.PFATLOCATION));
		StateConditionTest goalCondition = new TFGoalCondition(tf);

		// set up the initial state of the task
		State initialState = GridWorldDomain.getOneAgentOneLocationState(domain);
		GridWorldDomain.setAgent(initialState, 0, 0);
		GridWorldDomain.setLocation(initialState, 0, 10, 10);

		// set up the state hashing system
		DiscreteStateHashFactory hashingFactory = new DiscreteStateHashFactory();
		hashingFactory.setAttributesForClass(GridWorldDomain.CLASSAGENT,
				domain.getObjectClass(GridWorldDomain.CLASSAGENT).attributeList);

		// add visual observer

		//VisualActionObserver observer = new VisualActionObserver(domain, GridWorldVisualizer.getVisualizer(domain,
				//gwdg.getMap()));

		//((SADomain) domain).setActionObserverForAllAction(observer);
		//observer.initGUI();

		// construct the teacher
		OOMDPPlanner planner = new MyVI(domain, rf, tf, 1, hashingFactory, 0.001, 100);
		planner.planFromState(initialState);

		// create a Q-greedy policy from the planner
		Policy teacher = new GreedyQPolicy((QComputablePlanner) planner);

		Policy student = null;
		Policy.RandomPolicy t=new Policy.RandomPolicy(domain);

		DAGGERLearning dagger = new DAGGERLearning(domain, tf, rf, initialState, hashingFactory, teacher, student, 10,
				1000, 1000, 0.5);
		// System.out.println(Math.pow(0.5, 0));
		dagger.train();

		Policy p = dagger.getStudent();

		// record the plan results to a file
		p.evaluateBehavior(initialState, rf, tf);

		// visualize the value function and policy
		//dagger.valueFunctionVisualize((QComputablePlanner) planner, p);

	}

}
