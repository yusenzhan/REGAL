package domain;

public abstract class GridWordMapGenerator {
	
	//map height and width
	protected int height;
	protected int width;
	
	//map start state
	protected int startX;
	protected int startY;
	
	//map goal state
	protected int goalX;
	protected int goalY;
	
	//method to generate map
	public abstract int[][] generateMap();

	/**
	 * @return the height
	 */
	public int getHeight() {
		return height;
	}

	/**
	 * @param height the height to set
	 */
	public void setHeight(int height) {
		this.height = height;
	}

	/**
	 * @return the width
	 */
	public int getWidth() {
		return width;
	}

	/**
	 * @param width the width to set
	 */
	public void setWidth(int width) {
		this.width = width;
	}

	/**
	 * @return the startX
	 */
	public int getStartX() {
		return startX;
	}

	/**
	 * @param startX the startX to set
	 */
	public void setStartX(int startX) {
		this.startX = startX;
	}

	/**
	 * @return the startY
	 */
	public int getStartY() {
		return startY;
	}

	/**
	 * @param startY the startY to set
	 */
	public void setStartY(int startY) {
		this.startY = startY;
	}

	/**
	 * @return the goalX
	 */
	public int getGoalX() {
		return goalX;
	}

	/**
	 * @param goalX the goalX to set
	 */
	public void setGoalX(int goalX) {
		this.goalX = goalX;
	}

	/**
	 * @return the goalY
	 */
	public int getGoalY() {
		return goalY;
	}

	/**
	 * @param goalY the goalY to set
	 */
	public void setGoalY(int goalY) {
		this.goalY = goalY;
	}
	
	
}
