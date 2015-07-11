
import linearchainmdp.LinearChainMDPDomain;
import linearchainmdp.LinearChainMDPRewardFunction;
import linearchainmdp.LinearChainMDPTerminalFunction;
import policy.MinQPolicy;
import utility.DataFile;
import REGAL.DAGGERLearning;
import REGAL.MyVI;
import burlap.behavior.singleagent.Policy;
import burlap.behavior.singleagent.planning.OOMDPPlanner;
import burlap.behavior.singleagent.planning.QComputablePlanner;
import burlap.behavior.singleagent.planning.StateConditionTest;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.behavior.singleagent.planning.deterministic.TFGoalCondition;
import burlap.behavior.statehashing.DiscreteStateHashFactory;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.domain.singleagent.gridworld.GridWorldRewardFunction;
import burlap.domain.singleagent.gridworld.GridWorldStateParser;
import burlap.oomdp.auxiliary.StateParser;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.State;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.SADomain;
import burlap.oomdp.singleagent.common.SinglePFTF;
import burlap.oomdp.singleagent.common.UniformCostRF;
import burlap.oomdp.singleagent.common.VisualActionObserver;

public class ExperimentLinear {
	
	public static int maxTrial = 1;

	public double[][] records;
	public double[] prints;
	public int maxInteration;

	public ExperimentLinear(int maxInteration) {

		this.maxInteration = maxInteration;
		this.records = new double[maxTrial][maxInteration];
		this.prints = new double[this.maxInteration];
	}

	public void computeMean() {
	
		for (int i = 0; i < this.maxInteration; i++) {
			double reward = 0;
			for (int j = 0; j < maxTrial; j++) {
				reward += records[j][i];
			}

			this.prints[i] = reward / maxTrial;
		}
	}
	
	public void print() {
		
		System.out.println("---------------------------------Mean reward!----------------------------");

		for (int i = 0; i < this.maxInteration; i++) {

			System.out.println(i + " " + prints[i]);
		}

	}
	
	public void save(String filename){
		
		DataFile datafile=new DataFile(filename);
		System.out.println("---------------------------------Write to File:"+filename+"----------------------------");

		for (int i = 0; i < this.maxInteration; i++) {

			datafile.append(i+" "+prints[i]+"\n");
		}
		datafile.close();
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		// create the domain
		LinearChainMDPDomain gwdg = new LinearChainMDPDomain(50,0.5);


		Domain domain = gwdg.generateDomain();

		// define the task
		LinearChainMDPRewardFunction rf = new LinearChainMDPRewardFunction(50);
		
		
		// TerminalFunction tf = new NullTerminalFunction();
		TerminalFunction tf = new LinearChainMDPTerminalFunction(50,true);
		StateConditionTest goalCondition = new TFGoalCondition(tf);

		// set up the initial state of the task
		State initialState = LinearChainMDPDomain.getState(domain);

		// set up the state hashing system
		DiscreteStateHashFactory hashingFactory = new DiscreteStateHashFactory();
		hashingFactory.setAttributesForClass(GridWorldDomain.CLASSAGENT,
				domain.getObjectClass(GridWorldDomain.CLASSAGENT).attributeList);

		// add visual observer

		// VisualActionObserver observer = new VisualActionObserver(domain,GridWorldVisualizer.getVisualizer(domain, gwdg.getMap()));

		//((SADomain) domain).setActionObserverForAllAction(observer);
		//observer.initGUI();

		// construct the teacher
		OOMDPPlanner planner = new MyVI(domain, rf, tf, 1, hashingFactory, 0.001, 100);
		planner.planFromState(initialState);

		// create a Q-greedy policy from the planner
		Policy teacher = new GreedyQPolicy((QComputablePlanner) planner);
		Policy badteacher = new MinQPolicy((QComputablePlanner) planner);
		Policy.RandomPolicy randomteacher = new Policy.RandomPolicy(domain);

		Policy student = null;
		
		
		Experiment ex=new Experiment(20);

		for (int i = 0; i < maxTrial; i++) {
			System.out.println("--------------------------trial=" + i + "-----------------------------");

			DAGGERLearning dagger = new DAGGERLearning(domain, tf, rf, initialState, hashingFactory, randomteacher, student,
					20, 200, 1000, 0);
			// System.out.println(Math.pow(0.5, 0));
			double[] temparray = dagger.train();
			for(int j=0;j<temparray.length;j++){
				ex.records[i][j]=temparray[j];
			}
		}

		ex.computeMean();
		//ex.print();
		ex.save("linearnoteacher.txt");
		
		// Policy p = dagger.getStudent();

		// record the plan results to a file
		// p.evaluateBehavior(initialState, rf, tf);

		// visualize the value function and policy
		// dagger.valueFunctionVisualize((QComputablePlanner) planner, p);

	}
}
