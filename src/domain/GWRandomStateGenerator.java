package domain;

import java.util.List;
import java.util.Random;

import burlap.behavior.statehashing.StateHashTuple;
import burlap.debugtools.RandomFactory;
import burlap.oomdp.auxiliary.StateGenerator;
import burlap.oomdp.core.State;

public class GWRandomStateGenerator implements StateGenerator {

	private List<StateHashTuple> states;
	private StateHashTuple initialState;

	private Random rand;

	public GWRandomStateGenerator(List<StateHashTuple> states, StateHashTuple initialState) {

		this.states = states;
		this.initialState = initialState;

		this.rand = RandomFactory.getMapped(0);

	}

	@Override
	public State generateState() {
		if (rand.nextDouble() < 0.5) {
			
			return this.initialState.s;

		} else {
			int randIndex = this.rand.nextInt(states.size());

			return states.get(randIndex).s;
		}
	}

}
