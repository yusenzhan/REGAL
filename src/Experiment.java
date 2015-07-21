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

public class Experiment {

	public static int maxTrial = 10;

	public double[][] records;
	public double[] prints;
	public int maxInteration;

	public enum Extype {
		optiteacher, randteacher, badteacher, gridoptimalpolicy, gridnoadvice
	}

	public Experiment(int maxInteration) {

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

	public void save(String filename) {

		DataFile datafile = new DataFile(filename);
		System.out.println("---------------------------------Write to File:" + filename
				+ "----------------------------");

		for (int i = 0; i < this.maxInteration; i++) {

			datafile.append(i + " " + prints[i] + "\n");
		}
		datafile.close();
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		// create the domain
		GridWorldDomain gwdg = new GridWorldDomain(11, 11);
		gwdg.setMapToFourRooms();
		gwdg.setProbSucceedTransitionDynamics(0.8);
		//gwdg.setDeterministicTransitionDynamics();
		/*
		 * int [][] map = new int[][]{ {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
		 * {0,0,0,0,0,1,0,0,1,1,1,0,0,0,0}, {0,0,0,0,0,1,0,0,0,0,0,1,0,0,0},
		 * {0,0,0,0,0,1,0,0,0,0,0,0,1,0,0}, {0,0,0,0,0,1,0,0,0,0,0,0,1,0,0},
		 * {1,0,1,1,1,1,1,1,0,1,1,1,1,1,1}, {0,0,0,0,1,0,0,1,0,0,0,0,0,0,0},
		 * {0,0,0,0,1,0,0,1,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
		 * {0,0,0,0,1,0,0,1,0,0,0,0,0,0,0}, {0,0,0,0,1,0,0,1,0,0,0,0,0,0,0},
		 * {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0}, {0,0,0,0,1,0,0,1,0,0,0,0,0,0,0},
		 * {0,0,0,0,1,0,0,1,0,0,0,0,0,0,0}, {0,0,0,0,1,0,0,1,0,0,0,0,0,0,0}, };
		 * gwdg.setMap(map);
		 */
		Domain domain = gwdg.generateDomain();

		// create the state parser
		StateParser sp = new GridWorldStateParser(domain);

		// StateGenerator rStateGen = new MCRandomStateGenerator(domain);

		// define the task
		GridWorldRewardFunction rf = new GridWorldRewardFunction(domain, -1);
		rf.setReward(gwdg.getHeight() - 1, gwdg.getWidth() - 1, 1);
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

		// VisualActionObserver observer = new
		// VisualActionObserver(domain,GridWorldVisualizer.getVisualizer(domain,
		// gwdg.getMap()));

		// ((SADomain) domain).setActionObserverForAllAction(observer);
		// observer.initGUI();

		// construct the teacher
		OOMDPPlanner planner = new MyVI(domain, rf, tf, 1, hashingFactory, 0.001, 100);
		planner.planFromState(initialState);

		// create a Q-greedy policy from the planner
		Policy teacher = new GreedyQPolicy((QComputablePlanner) planner);
		Policy badteacher = new MinQPolicy((QComputablePlanner) planner);
		Policy.RandomPolicy randomteacher = new Policy.RandomPolicy(domain);

		Policy student = null;

		int maxinteration = 20;
		int[] maxsteps = { 10, 20, 50, 100, 200 };
		Experiment ex = null;

		/*for (int maxstep : maxsteps) {
			for (Extype exp : Extype.values()) {
				switch (exp) {
				case optiteacher:
					ex = new Experiment(maxinteration);
					for (int i = 0; i < maxTrial; i++) {
						System.out.println("--------------------------trial=" + i + "-----------------------------");

						DAGGERLearning dagger = new DAGGERLearning(domain, tf, rf, initialState, hashingFactory,
								teacher, student, maxinteration, maxstep, 1000, 0.5);
						// System.out.println(Math.pow(0.5, 0));
						double[] temparray = dagger.train();
						for (int j = 0; j < temparray.length; j++) {
							ex.records[i][j] = temparray[j];
						}
					}
					ex.computeMean();
					ex.save(exp.toString() + maxstep + ".txt");
					break;
				case randteacher:
					ex = new Experiment(maxinteration);
					for (int i = 0; i < maxTrial; i++) {
						System.out.println("--------------------------trial=" + i + "-----------------------------");

						DAGGERLearning dagger = new DAGGERLearning(domain, tf, rf, initialState, hashingFactory,
								randomteacher, student, maxinteration, maxstep, 1000, 0.5);
						// System.out.println(Math.pow(0.5, 0));
						double[] temparray = dagger.train();
						for (int j = 0; j < temparray.length; j++) {
							ex.records[i][j] = temparray[j];
						}
					}
					ex.computeMean();
					ex.save(exp.toString() + maxstep + ".txt");
					break;
				case badteacher:
					ex = new Experiment(maxinteration);
					for (int i = 0; i < maxTrial; i++) {
						System.out.println("--------------------------trial=" + i + "-----------------------------");

						DAGGERLearning dagger = new DAGGERLearning(domain, tf, rf, initialState, hashingFactory,
								badteacher, student, maxinteration, maxstep, 1000, 0.5);
						// System.out.println(Math.pow(0.5, 0));
						double[] temparray = dagger.train();
						for (int j = 0; j < temparray.length; j++) {
							ex.records[i][j] = temparray[j];
						}
					}
					ex.computeMean();
					ex.save(exp.toString() + maxstep + ".txt");
					break;
				case gridnoadvice:
					ex = new Experiment(maxinteration);
					for (int i = 0; i < maxTrial; i++) {
						System.out.println("--------------------------trial=" + i + "-----------------------------");

						DAGGERLearning dagger = new DAGGERLearning(domain, tf, rf, initialState, hashingFactory,
								randomteacher, student, maxinteration, maxstep, 1000, 0);
						// System.out.println(Math.pow(0.5, 0));
						double[] temparray = dagger.train();
						for (int j = 0; j < temparray.length; j++) {
							ex.records[i][j] = temparray[j];
						}
					}
					ex.computeMean();
					ex.save(exp.toString() + maxstep + ".txt");
					break;
				case gridoptimalpolicy:
					ex = new Experiment(maxinteration);
					for (int i = 0; i < maxTrial; i++) {
						System.out.println("--------------------------trial=" + i + "-----------------------------");

						DAGGERLearning dagger = new DAGGERLearning(domain, tf, rf, initialState, hashingFactory,
								teacher, student, maxinteration, maxstep, 1000, 1);
						// System.out.println(Math.pow(0.5, 0));
						double[] temparray = dagger.train();
						for (int j = 0; j < temparray.length; j++) {
							ex.records[i][j] = temparray[j];
						}
					}
					ex.computeMean();
					ex.save(exp.toString() + maxstep + ".txt");
					break;

				default:
					throw new RuntimeException("unreachable");

				}
			}
		}*/
		
		ex = new Experiment(maxinteration);
		for (int i = 0; i < maxTrial; i++) {
			System.out.println("--------------------------trial=" + i + "-----------------------------");

			DAGGERLearning dagger = new DAGGERLearning(domain, tf, rf, initialState, hashingFactory,
					teacher, student, maxinteration, 200, 1000, 0.5);
			// System.out.println(Math.pow(0.5, 0));
			double[] temparray = dagger.train();
			for (int j = 0; j < temparray.length; j++) {
				ex.records[i][j] = temparray[j];
			}
		}
		
		ex.computeMean();
		ex.save("test.txt");

		// Policy p = dagger.getStudent();

		// record the plan results to a file
		// p.evaluateBehavior(initialState, rf, tf);

		// visualize the value function and policy
		// dagger.valueFunctionVisualize((QComputablePlanner) planner, p);

	}
}
