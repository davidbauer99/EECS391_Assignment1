package edu.cwru.sepia.agent;

import java.util.Comparator;
import java.util.PriorityQueue;

import edu.cwru.sepia.agent.AstarAgent.MapLocation;

public class AStarList {

	private final PriorityQueue<MapLocation> locs;

	/**
	 * Creates a new {@link AStarList} with the the given comparator.
	 * 
	 * @param cmpr
	 *            the {@link Comparator} that will be used to order elements in
	 *            this list.
	 */
	public AStarList(Comparator<MapLocation> cmpr) {
		this.locs = new PriorityQueue<AstarAgent.MapLocation>(cmpr);
	}

	/**
	 * Removes the {@link MapLocation} with the lowest TotalCost.
	 * 
	 * @return the {@link MapLocation} with the lowest total cost or null.
	 */
	public MapLocation pop() {
		return locs.poll();
	}

	/**
	 * Adds a new {@link MapLocation} to the list.
	 * 
	 * @param loc
	 *            the {@link MapLocation} to be added.
	 */
	public void add(MapLocation loc) {
		locs.add(loc);
	}

	/**
	 * Checks if there is already a {@link MapLocation} in the list with the
	 * same coordinates that has a lower score.
	 * 
	 * @param loc
	 *            the location that is being checked against
	 * @return true if there exits a {@link MapLocation} m such that
	 *         m.equals(loc) is true and m.cost < loc.cost
	 */
	public boolean alreadyContainsWithLowerCost(MapLocation loc) {
		MapLocation[] locArray = locs.toArray(new MapLocation[] {});
		for (MapLocation arrLoc : locArray) {
			if (arrLoc.equals(loc) && arrLoc.cost < loc.cost) {
				return true;
			}
		}
		return false;
	}

	/**
	 * Removes the {@link MapLocation} from the list.
	 * 
	 * @param loc
	 *            a {@link MapLocation} to be removed. An element e will be
	 *            removed if and only if e.equals(loc) is true
	 */
	public void remove(MapLocation loc) {
		locs.remove(loc);
	}

}
