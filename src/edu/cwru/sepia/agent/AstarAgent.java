package edu.cwru.sepia.agent;

import java.io.InputStream;
import java.io.OutputStream;
import java.util.Collection;
import java.util.EmptyStackException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Stack;

import edu.cwru.sepia.action.Action;
import edu.cwru.sepia.environment.model.history.History;
import edu.cwru.sepia.environment.model.state.ResourceNode;
import edu.cwru.sepia.environment.model.state.State;
import edu.cwru.sepia.environment.model.state.Unit;
import edu.cwru.sepia.util.Direction;

public class AstarAgent extends Agent {

    class MapLocation
    {

		public int x, y;
        public MapLocation parent;
        public float cost;

        public MapLocation(int x, int y, MapLocation cameFrom, float cost)
        {
            this.x = x;
            this.y = y;
            this.parent = cameFrom;
            this.cost = cost;
        }

		@Override
		public String toString() {
			return "(" + x + "," + y + ")";
		};

		private AstarAgent getOuterType() {
			return AstarAgent.this;
		}
        
		@Override
		public int hashCode() {
			final int prime = 31;
			int result = 1;
			result = prime * result + getOuterType().hashCode();
			result = prime * result + x;
			result = prime * result + y;
			return result;
		}

		@Override
		public boolean equals(Object obj) {
			if (this == obj)
				return true;
			if (obj == null)
				return false;
			if (getClass() != obj.getClass())
				return false;
			MapLocation other = (MapLocation) obj;
			if (!getOuterType().equals(other.getOuterType()))
				return false;
			if (x != other.x)
				return false;
			if (y != other.y)
				return false;
			return true;
		}
		
    }

    Stack<MapLocation> path;
    int footmanID, townhallID, enemyFootmanID;
    MapLocation nextLoc;

    private long totalPlanTime = 0; // nsecs
    private long totalExecutionTime = 0; //nsecs

    public AstarAgent(int playernum)
    {
        super(playernum);

        System.out.println("Constructed AstarAgent");
    }

    @Override
    public Map<Integer, Action> initialStep(State.StateView newstate, History.HistoryView statehistory) {
        // get the footman location
        List<Integer> unitIDs = newstate.getUnitIds(playernum);

        if(unitIDs.size() == 0)
        {
            System.err.println("No units found!");
            return null;
        }

        footmanID = unitIDs.get(0);

        // double check that this is a footman
        if(!newstate.getUnit(footmanID).getTemplateView().getName().equals("Footman"))
        {
            System.err.println("Footman unit not found");
            return null;
        }

        // find the enemy playernum
        Integer[] playerNums = newstate.getPlayerNumbers();
        int enemyPlayerNum = -1;
        for(Integer playerNum : playerNums)
        {
            if(playerNum != playernum) {
                enemyPlayerNum = playerNum;
                break;
            }
        }

        if(enemyPlayerNum == -1)
        {
            System.err.println("Failed to get enemy playernumber");
            return null;
        }

        // find the townhall ID
        List<Integer> enemyUnitIDs = newstate.getUnitIds(enemyPlayerNum);

        if(enemyUnitIDs.size() == 0)
        {
            System.err.println("Failed to find enemy units");
            return null;
        }

        townhallID = -1;
        enemyFootmanID = -1;
        for(Integer unitID : enemyUnitIDs)
        {
            Unit.UnitView tempUnit = newstate.getUnit(unitID);
            String unitType = tempUnit.getTemplateView().getName().toLowerCase();
            if(unitType.equals("townhall"))
            {
                townhallID = unitID;
            }
            else if(unitType.equals("footman"))
            {
                enemyFootmanID = unitID;
            }
            else
            {
                System.err.println("Unknown unit type");
            }
        }

        if(townhallID == -1) {
            System.err.println("Error: Couldn't find townhall");
            return null;
        }

        long startTime = System.nanoTime();
        path = findPath(newstate);
        totalPlanTime += System.nanoTime() - startTime;

        return middleStep(newstate, statehistory);
    }

    @Override
    public Map<Integer, Action> middleStep(State.StateView newstate, History.HistoryView statehistory) {
        long startTime = System.nanoTime();
        long planTime = 0;

        Map<Integer, Action> actions = new HashMap<Integer, Action>();

        if(shouldReplanPath(newstate, statehistory, path)) {
            long planStartTime = System.nanoTime();
            path = findPath(newstate);
            planTime = System.nanoTime() - planStartTime;
            totalPlanTime += planTime;
        }

        Unit.UnitView footmanUnit = newstate.getUnit(footmanID);

        int footmanX = footmanUnit.getXPosition();
        int footmanY = footmanUnit.getYPosition();

        if(!path.empty() && (nextLoc == null || (footmanX == nextLoc.x && footmanY == nextLoc.y))) {

            // stat moving to the next step in the path
            nextLoc = path.pop();

            System.out.println("Moving to (" + nextLoc.x + ", " + nextLoc.y + ")");
        }

        if(nextLoc != null && (footmanX != nextLoc.x || footmanY != nextLoc.y))
        {
            int xDiff = nextLoc.x - footmanX;
            int yDiff = nextLoc.y - footmanY;

            // figure out the direction the footman needs to move in
            Direction nextDirection = getNextDirection(xDiff, yDiff);

            actions.put(footmanID, Action.createPrimitiveMove(footmanID, nextDirection));
        } else {
            Unit.UnitView townhallUnit = newstate.getUnit(townhallID);

            // if townhall was destroyed on the last turn
            if(townhallUnit == null) {
                terminalStep(newstate, statehistory);
                return actions;
            }

            if(Math.abs(footmanX - townhallUnit.getXPosition()) > 1 ||
                    Math.abs(footmanY - townhallUnit.getYPosition()) > 1)
            {
                System.err.println("Invalid plan. Cannot attack townhall");
                totalExecutionTime += System.nanoTime() - startTime - planTime;
                return actions;
            }
            else {
                System.out.println("Attacking TownHall");
                // if no more movements in the planned path then attack
                actions.put(footmanID, Action.createPrimitiveAttack(footmanID, townhallID));
            }
        }

        totalExecutionTime += System.nanoTime() - startTime - planTime;
        return actions;
    }

    @Override
    public void terminalStep(State.StateView newstate, History.HistoryView statehistory) {
        System.out.println("Total turns: " + newstate.getTurnNumber());
        System.out.println("Total planning time: " + totalPlanTime/1e9);
        System.out.println("Total execution time: " + totalExecutionTime/1e9);
        System.out.println("Total time: " + (totalExecutionTime + totalPlanTime)/1e9);
    }

    @Override
    public void savePlayerData(OutputStream os) {

    }

    @Override
    public void loadPlayerData(InputStream is) {

    }

    /**
     * You will implement this method.
     *
     * This method should return true when the path needs to be replanned
     * and false otherwise. This will be necessary on the dynamic map where the
     * footman will move to block your unit.
     *
     * @param state
     * @param history
     * @param currentPath
     * @return
     */
    private boolean shouldReplanPath(State.StateView state, History.HistoryView history, Stack<MapLocation> currentPath)
    {
		if (enemyFootmanID == -1 || currentPath.empty()) {
        return false;
		}
		boolean pathBlocked = false;// check to see if an enemy occupies the
									// remaining steps to be followed
		// loop through each location in the stack, check if it is occupied by
		// the blocker
		Stack<MapLocation> pathCopy = (Stack<MapLocation>) currentPath.clone();// make
																				// a
																				// clone
																				// of
																				// the
																				// path
																				// that
																				// we
																				// can
																				// manipulate
		Unit.UnitView enemyFootmanUnit = state.getUnit(enemyFootmanID);
		MapLocation enemyFootmanLoc = new MapLocation(
				enemyFootmanUnit.getXPosition(),
				enemyFootmanUnit.getYPosition(), null, 0);// set current blocker
															// location to a
															// local variable
		MapLocation nextLoc = pathCopy.pop();
		while (nextLoc != null) {
			if (nextLoc.equals(enemyFootmanLoc)) {// replace with condition that
													// nextLoc is occupied by
													// the blocker
				pathBlocked = true;
				break;
			}
			try {
				nextLoc = pathCopy.pop();
			} catch (EmptyStackException ex) {
				nextLoc = null;
			}
		}
		return pathBlocked;
    }

    /**
     * This method is implemented for you. You should look at it to see examples of
     * how to find units and resources in Sepia.
     *
     * @param state
     * @return
     */
    private Stack<MapLocation> findPath(State.StateView state)
    {
        Unit.UnitView townhallUnit = state.getUnit(townhallID);
        Unit.UnitView footmanUnit = state.getUnit(footmanID);

        MapLocation startLoc = new MapLocation(footmanUnit.getXPosition(), footmanUnit.getYPosition(), null, 0);

        MapLocation goalLoc = new MapLocation(townhallUnit.getXPosition(), townhallUnit.getYPosition(), null, 0);

        MapLocation footmanLoc = null;
        if(enemyFootmanID != -1) {
            Unit.UnitView enemyFootmanUnit = state.getUnit(enemyFootmanID);
            footmanLoc = new MapLocation(enemyFootmanUnit.getXPosition(), enemyFootmanUnit.getYPosition(), null, 0);
        }

        // get resource locations
        List<Integer> resourceIDs = state.getAllResourceIds();
        Set<MapLocation> resourceLocations = new HashSet<MapLocation>();
        for(Integer resourceID : resourceIDs)
        {
            ResourceNode.ResourceView resource = state.getResourceNode(resourceID);

            resourceLocations.add(new MapLocation(resource.getXPosition(), resource.getYPosition(), null, 0));
        }

        return AstarSearch(startLoc, goalLoc, state.getXExtent(), state.getYExtent(), footmanLoc, resourceLocations);
    }
    /**
     * This is the method you will implement for the assignment. Your implementation
     * will use the A* algorithm to compute the optimum path from the start position to
     * a position adjacent to the goal position.
     *
     * You will return a Stack of positions with the top of the stack being the first space to move to
     * and the bottom of the stack being the last space to move to. If there is no path to the townhall
     * then return null from the method and the agent will print a message and do nothing.
     * The code to execute the plan is provided for you in the middleStep method.
     *
     * As an example consider the following simple map
     *
     * F - - - -
     * x x x - x
     * H - - - -
     *
     * F is the footman
     * H is the townhall
     * x's are occupied spaces
     *
     * xExtent would be 5 for this map with valid X coordinates in the range of [0, 4]
     * x=0 is the left most column and x=4 is the right most column
     *
     * yExtent would be 3 for this map with valid Y coordinates in the range of [0, 2]
     * y=0 is the top most row and y=2 is the bottom most row
     *
     * resourceLocations would be {(0,1), (1,1), (2,1), (4,1)}
     *
     * The path would be
     *
     * (1,0)
     * (2,0)
     * (3,1)
     * (2,2)
     * (1,2)
     *
     * Notice how the initial footman position and the townhall position are not included in the path stack
     *
     * @param start Starting position of the footman
     * @param goal MapLocation of the townhall
     * @param xExtent Width of the map
     * @param yExtent Height of the map
     * @param resourceLocations Set of positions occupied by resources
     * @return Stack of positions with top of stack being first move in plan
     */
    private Stack<MapLocation> AstarSearch(MapLocation start, MapLocation goal, int xExtent, int yExtent, MapLocation enemyFootmanLoc, Set<MapLocation> resourceLocations)
    {
		// Create the open and closed list
		AStarList openList = newAStarList(goal);
		AStarList closedList = newAStarList(goal);
		// Add the current location to the open list
		openList.add(start);
		MapLocation currentLoc = openList.pop();

		while (currentLoc != null) {
			// If the current location is the goal, build the path
			if (currentLoc.equals(goal)) {
				return buildPath(currentLoc, start);
			}
			// Get the neighbors that are on the map and not obstacles
			Collection<MapLocation> neighbors = getValidNeighbors(currentLoc,
					xExtent, yExtent, resourceLocations, enemyFootmanLoc);
			// Check each neighbor
			for (MapLocation loc : neighbors) {
				// If a better path to this node is in the open list, skip it
				if (openList.alreadyContainsWithLowerCost(loc)) {
					continue;
				}
				// If a better path to this node is in the closed list, skip it
				if (closedList.alreadyContainsWithLowerCost(loc)) {
					continue;
				}
				// Remove this node from any list that may contain it
				closedList.remove(loc);
				openList.remove(loc);
				// Add it to the open list
				openList.add(loc);
			}
			// Mark currentLoc as having been searched
			closedList.add(currentLoc);
			// Move to the next best node
			currentLoc = openList.pop();
		}
		// If the closed list is empty, then there is no path
		System.out.println("No available path.");
		System.exit(0);
		return new Stack<MapLocation>();
	}

	/**
	 * Gets the valid neighbors of the current location.
	 * 
	 * @param currentLoc
	 *            the current location to get neighbors of.
	 * @param xExtent
	 *            the xExtent of the map
	 * @param yExtent
	 *            the yExtent of the map
	 * @param resourceLocations
	 *            the location of any resources on the map
	 * @param enemyFootmanLoc
	 *            the location of a possible enemy
	 * @return a set of neighboring locations that are not resources, enemies
	 *         and on the map.
	 */
	private Collection<MapLocation> getValidNeighbors(MapLocation currentLoc,
			int xExtent, int yExtent, Set<MapLocation> resourceLocations,
			MapLocation enemyFootmanLoc) {
		Collection<MapLocation> neighbors = new HashSet<AstarAgent.MapLocation>();
		Collection<MapLocation> candidates = new HashSet<AstarAgent.MapLocation>();
		int x = currentLoc.x;
		int y = currentLoc.y;
		float cost = currentLoc.cost + 1;
		// NE
		candidates.add(new MapLocation(x - 1, y - 1, currentLoc, cost));
		// N
		candidates.add(new MapLocation(x, y - 1, currentLoc, cost));
		// NW
		candidates.add(new MapLocation(x + 1, y - 1, currentLoc, cost));
		// W
		candidates.add(new MapLocation(x + 1, y, currentLoc, cost));
		// SW
		candidates.add(new MapLocation(x + 1, y + 1, currentLoc, cost));
		// S
		candidates.add(new MapLocation(x, y + 1, currentLoc, cost));
		// SE
		candidates.add(new MapLocation(x - 1, y + 1, currentLoc, cost));
		// E
		candidates.add(new MapLocation(x - 1, y, currentLoc, cost));
		for (MapLocation loc : candidates) {
			if (!resourceLocations.contains(loc)
					&& (enemyFootmanLoc == null || !enemyFootmanLoc.equals(loc))
					&& 0 <= loc.x
					&& loc.x < xExtent && 0 <= loc.y && loc.y < yExtent) {
				neighbors.add(loc);
			}
		}
		return neighbors;
	}

	/**
	 * Builds a path from the start to the destination.
	 * 
	 * @param dest
	 *            the end of the path.
	 * @param start
	 *            the start of the path.
	 * @return A stack where the first element is the {@link MapLocation} to be
	 *         moved to from the start.
	 */
	private Stack<MapLocation> buildPath(MapLocation dest,
			MapLocation start) {
		Stack<MapLocation> path = new Stack<AstarAgent.MapLocation>();
		MapLocation end = dest.parent;
		while (!end.equals(start)) {
			path.push(end);
			end = end.parent;
		}
		return path;
	}

	// Convenience method for getting an AStarList object.
	private AStarList newAStarList(MapLocation goal) {
		return new AStarList(
				(o1, o2) -> {
					float c1 = totalCost(o1, goal);
					float c2 = totalCost(o2, goal);
					if (c1 < c2) {
						return -1;
					} else if (c1 > c2) {
						return 1;
					} else {
						return 0;
					}
				});
	}
    
	/**
	 * Computes the total cost of a given input based on f(x) = g(x) + h(x)
	 * 
	 * @param loc
	 *            the location to have its cost calculated
	 * @param goal
	 *            the desired end location
	 * @return a cost that is based on the nodes cost and the Chebyshev
	 *         distance.
	 */
    private float totalCost(MapLocation loc, MapLocation goal) {
		float g = loc.cost;
		float h = Math.max(Math.abs(goal.x - loc.x), Math.abs(goal.y - loc.y));
		return g + h;
    }

    /**
     * Primitive actions take a direction (e.g. NORTH, NORTHEAST, etc)
     * This converts the difference between the current position and the
     * desired position to a direction.
     *
     * @param xDiff Integer equal to 1, 0 or -1
     * @param yDiff Integer equal to 1, 0 or -1
     * @return A Direction instance (e.g. SOUTHWEST) or null in the case of error
     */
    private Direction getNextDirection(int xDiff, int yDiff) {

        // figure out the direction the footman needs to move in
        if(xDiff == 1 && yDiff == 1)
        {
            return Direction.SOUTHEAST;
        }
        else if(xDiff == 1 && yDiff == 0)
        {
            return Direction.EAST;
        }
        else if(xDiff == 1 && yDiff == -1)
        {
            return Direction.NORTHEAST;
        }
        else if(xDiff == 0 && yDiff == 1)
        {
            return Direction.SOUTH;
        }
        else if(xDiff == 0 && yDiff == -1)
        {
            return Direction.NORTH;
        }
        else if(xDiff == -1 && yDiff == 1)
        {
            return Direction.SOUTHWEST;
        }
        else if(xDiff == -1 && yDiff == 0)
        {
            return Direction.WEST;
        }
        else if(xDiff == -1 && yDiff == -1)
        {
            return Direction.NORTHWEST;
        }

        System.err.println("Invalid path. Could not determine direction");
        return null;
    }
}
