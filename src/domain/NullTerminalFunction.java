package domain;

import burlap.oomdp.core.State;
import burlap.oomdp.core.TerminalFunction;

public class NullTerminalFunction implements TerminalFunction {

	@Override
	public boolean isTerminal(State s) {
		// TODO Auto-generated method stub
		return false;
	}

}
