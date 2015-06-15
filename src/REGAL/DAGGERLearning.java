package REGAL;
import burlap.behavior.singleagent.Policy;
//import burlap.behavior.singleagent.learning.lspi.SARSCollector;
import burlap.behavior.singleagent.learning.lspi.SARSData;
import burlap.oomdp.auxiliary.StateGenerator;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.State;
//import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;



public class DAGGERLearning{
	
	
	protected Domain domain; //domain
	protected TerminalFunction tf;
	protected RewardFunction rf;
	protected State initialState;
	
	protected REGAL regal; //REGAL algorithm
	protected Policy teacher; // the teacher policy
	protected Policy student; // the student policy
	protected int currentTimeStep; //current time step
	protected int maxInteration;//max iteration for the data aggregation
	protected int maxSteps;
	protected double H; // the upper bound of the span of all MDPs
	protected double p; // prbability seed
	
	
	

	
	



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
	public DAGGERLearning(Domain domain, TerminalFunction tf,
			RewardFunction rf, State initialState, Policy teacher,
			Policy student, int maxInteration, int maxSteps, double h, double p) {
		super();
		this.domain = domain;
		this.tf = tf;
		this.rf = rf;
		this.initialState = initialState;
		this.regal = new REGAL(domain,initialState,tf,rf);
		this.teacher = teacher;
		this.student = student;
		this.maxInteration = maxInteration;
		this.maxSteps = maxSteps;
		H = h;
		this.p = p;
		this.currentTimeStep=0;//current time is always 0
	}

	
	/**
	 * Training the REGAL algorithm and the policy the member student.
	 */

	public void train(){
		MySARSCollector collector=new MySARSCollector(domain);
		SARSData data=null;
		//train the student
		for(int i=0;i<this.maxInteration;i++){
			
			double beta=getBetai(i,p);
			data=collector.collectDataFrom( this.initialState,rf, maxSteps, tf, null, teacher, student, beta);
			student=regal.experiment(data);
		}
	}
	
	
	
	/** 
	 * @param i the current iteration
	 * @param p the probability seed in [0,1]
	 * @return the p^i is the beta decaying exponentially.
	 */
	
	public double getBetai(int i, double p){
		
		return Math.pow(i, p);
		
	}
	


	
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		Domain domain=null;
		

	}
	
}
