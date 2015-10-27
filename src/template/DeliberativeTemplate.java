package template;

/* import table */
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.PriorityQueue;
import java.util.Set;

import logist.simulation.Vehicle;
import logist.agent.Agent;
import logist.behavior.DeliberativeBehavior;
import logist.plan.Action;
import logist.plan.Plan;
import logist.task.Task;
import logist.task.TaskDistribution;
import logist.task.TaskSet;
import logist.topology.Topology;
import logist.topology.Topology.City;

/**
 * An optimal planner for one vehicle.
 */
@SuppressWarnings("unused")
public class DeliberativeTemplate implements DeliberativeBehavior {

	enum Algorithm { BFS, ASTAR }
	
	/* Environment */
	Topology topology;
	TaskDistribution td;
	
	/* the properties of the agent */
	Agent agent;
	int capacity;

	/* the planning class */
	Algorithm algorithm;
	
	/* Starting city of the current plan */
	City start;
	
	@Override
	public void setup(Topology topology, TaskDistribution td, Agent agent) {
		this.topology = topology;
		this.td = td;
		this.agent = agent;
		
		// initialize the planner
		int capacity = agent.vehicles().get(0).capacity();
		String algorithmName = agent.readProperty("algorithm", String.class, "ASTAR");
		
		// Throws IllegalArgumentException if algorithm is unknown
		algorithm = Algorithm.valueOf(algorithmName.toUpperCase());
		
		// ...
		
	}
	
	@Override
	public Plan plan(Vehicle vehicle, TaskSet tasks) {
		Plan plan;

		// Compute the plan with the selected algorithm.
		switch (algorithm) {
		case ASTAR:
			// ...
			plan = optPlan(vehicle, tasks, new ASTARComparator());
			break;
		case BFS:
			// ...
			plan = optPlan(vehicle, tasks, new BFSComparator());
			break;
		default:
			throw new AssertionError("Should not happen.");
		}
		System.out.println(plan.totalDistance());
		return plan;
	}
	
	/**
	 * @param vehicle
	 * @param tasks
	 * @param comparator is different for BFS and ASTAR. 
	 * @return plan for the vehicle according to the algorithm used
	 */
	private Plan optPlan(Vehicle vehicle, TaskSet tasks, Comparator<ExtendedPlan> comparator) {
		
		start = vehicle.getCurrentCity();
		int tempWeight = 0;
		// Total weight of the tasks carried by the vehicle when the plan needs to be recomputed
		for(Task t: vehicle.getCurrentTasks()) {
			tempWeight += t.weight;
		}
		
		// Initialize the starting node and the queue with the first node
		ExtendedPlan plan = new ExtendedPlan(new Plan(start), 0, tempWeight, 0, computeHeuristic(tasks), tasks, vehicle.getCurrentTasks(), start);
		PriorityQueue<ExtendedPlan> queue = new PriorityQueue<ExtendedPlan>(100,comparator);
		queue.add(plan);
		
		while(queue.size() != 0) {
			// We get the first node. In case of BFS the queue is not sorted but in case of ASTAR it will 
			// return the node based on a heuristic
			ExtendedPlan first = queue.poll();
			
			// Termination condition: the number of tasks delivered in the plan 
			// is equal to the total number of tasks, we return the final node
			if(first.getCount() == (tasks.size() + vehicle.getCurrentTasks().size())) {
				return first.getPlan(); 
			}
			
			// We need to add to the queue the successors of the node.
			// There is a successor for each action that still need to be done: pick up an available task 
			// or deliver a task carried by the vehicle
			
			// Adds a pickup to a plan
			for(Integer pickup: first.getRemaining()) {
				Task next = setContains(tasks, pickup.intValue()) ?
						getTask(tasks, pickup.intValue())
						: getTask(vehicle.getCurrentTasks(), pickup.intValue());
				
				// A pickup action is added only if the vehicle has the capacity to take it
				if (canPickup(vehicle, next, first)) {
					ExtendedPlan newPlan = new ExtendedPlan(
						first.getPlan(), 
						first.count, 
						first.getWeight() + next.weight, // increasing the weight of the vehicle
						first.depth + 1, // used to do the BFS and getting the nodes corresponding to their depth
						computeHeuristic(remainingSet(tasks, first.getRemaining())), // used to order the queue in the ASTAR algorithm
						first.remaining, 
						first.carried, 
						first.getCity());
					
					// the vehicle needs to move to the pickup city
					for (City c: newPlan.getCity().pathTo(next.pickupCity)) {
						newPlan.getPlan().appendMove(c);
					}
					newPlan.setCity(next.pickupCity);
					newPlan.getPlan().appendPickup(next);
					newPlan.remaining.remove(pickup);
					newPlan.carried.add(pickup);
					queue.add(newPlan);
				}
			}
			// Adds a delivery to a plan
			for(Integer carried: first.getCarried()) { 
				Task next = setContains(tasks, carried.intValue()) ? 
					getTask(tasks, carried.intValue())
					: getTask(vehicle.getCurrentTasks(), carried.intValue());
				
				ExtendedPlan newPlan = new ExtendedPlan(
					first.getPlan(), 
					first.count + 1, // one more task is delivered
					first.depth + 1,
					first.getWeight() - next.weight, // updating the weight
					computeHeuristic(remainingSet(tasks, first.getRemaining())),
					first.remaining, 
					first.carried, 
					first.getCity());
				
				// move to the delivery city and then deliver
				for (City c: newPlan.getCity().pathTo(next.deliveryCity)) {
					newPlan.getPlan().appendMove(c);
				}
				newPlan.setCity(next.deliveryCity);
				newPlan.getPlan().appendDelivery(next);
				newPlan.carried.remove(carried);
				queue.add(newPlan);
			}
		}
		return null; //Some tasks could not be delivered
	}
	
	private Plan naivePlan(Vehicle vehicle, TaskSet tasks) {
		City current = vehicle.getCurrentCity();
		Plan plan = new Plan(current);

		for (Task task : tasks) {
			// move: current city => pickup location
			for (City city : current.pathTo(task.pickupCity))
				plan.appendMove(city);

			plan.appendPickup(task);

			// move: pickup location => delivery location
			for (City city : task.path())
				plan.appendMove(city);

			plan.appendDelivery(task);

			// set current city
			current = task.deliveryCity;
		}
		return plan;
	}

	@Override
	public void planCancelled(TaskSet carriedTasks) {
		
		if (!carriedTasks.isEmpty()) {
			// This cannot happen for this simple agent, but typically
			// you will need to consider the carriedTasks when the next
			// plan is computed.	
		}
	}
	
	/**
	 * @param vehicle
	 * @param task
	 * @param plan
	 * @return true if the vehicle has the capacity to take the additional task
	 */
	private boolean canPickup(Vehicle vehicle, Task task, ExtendedPlan plan) {
		return (plan.getWeight() + task.weight <= vehicle.capacity());
	}
	
	/**
	 * @param plan
	 * @return a copy of an ExtendedPlan
	 */
	private ExtendedPlan copyPlan(ExtendedPlan plan) {
		ExtendedPlan newPlan = new ExtendedPlan(
			new Plan(start), 
			plan.getCount(), 
			plan.getWeight(),
			plan.depth,
			plan.getHeuristic(),
			plan.getRemaining(), 
			plan.getCarried(), 
			plan.getCity());
		for (Action a: plan.getPlan()) {
			newPlan.getPlan().append(a);
		}
		return newPlan;
	}
	
	/**
	 * @param plan
	 * @return a copy of a plan
	 */
	private Plan copy(Plan plan) {
		Plan p = new Plan(start);
		for(Action a: plan) {
			p.append(a);
		}
		return p;
	}
	
	/**
	 * ExtendedPlan is used to describe a node. 
	 *
	 */
	public class ExtendedPlan {
		private Plan plan; 
		private int count; 
		private City city; 
		private int weight; 
		private HashSet<Integer> carried = new HashSet<Integer>(); // Set of the IDs of all carried tasks
		private HashSet<Integer> remaining = new HashSet<Integer>(); // Set of IDs of tasks not been picked up
		private int depth; 
		private double heuristic; 
		
		/**
		 * @param plan
		 * @param c Number of tasks delivered
		 * @param w Sum of all tasks carried by vehicle in plan
		 * @param d For BFS
		 * @param h For ASTAR
		 * @param tasks
		 * @param carrying
		 * @param city Last city in the plan
		 */
		public ExtendedPlan(Plan plan, int c, int w, int d, double h, TaskSet tasks, TaskSet carrying, City city) {
			this.plan = copy(plan);
			count = c;
			weight = w;
			this.depth = d;
			this.city = city;
			heuristic = h;
			for (Task t: tasks) {
				remaining.add(new Integer(t.id));
			}
			for (Task t: carrying) {
				carried.add(new Integer(t.id));
			}
		}
		
		/**
		 * @param plan
		 * @param c
		 * @param w
		 * @param d
		 * @param h
		 * @param tasks
		 * @param carrying
		 * @param city
		 */
		public ExtendedPlan(Plan plan, int c, int w, int d, double h, HashSet<Integer> tasks, HashSet<Integer> carrying, City city) {
			this.plan = copy(plan);
			count = c;
			weight = w;
			depth = d;
			heuristic = h;
			this.city = city;
			for (Integer t: tasks) {
				remaining.add(new Integer(t.intValue()));
			}
			for (Integer t: carrying) {
				carried.add(new Integer(t.intValue()));
			}
		}
		
		/**
		 * @return plan
		 */
		public Plan getPlan() {
			return plan;
		}
		
		/**
		 * @return count
		 */
		public int getCount() {
			return count;
		}
		
		/**
		 * @return carried tasks
		 */
		public HashSet<Integer> getCarried() {
			return carried;
		}
		
		/**
		 * @return remaining tasks
		 */
		public HashSet<Integer> getRemaining() {
			return remaining;
		}
		
		/**
		 * @param c new count
		 */
		public void setCount(int c) {
			count = c;
		}
		
		/**
		 * increment count
		 */
		public void increment() {
			count++;
		}
		
		/**
		 * @return last city of the plan
		 */
		public City getCity() {
			return city;
		}
		
		/**
		 * @param newCity update last city of the plan
		 */
		public void setCity(City newCity) {
			city = newCity;
		}
		
		/**
		 * @return total weight carried
		 */
		public int getWeight() {
			return weight;
		}
		
		/**
		 * @param newWeight update weight carried
		 */
		public void setWeight(int newWeight) {
			weight = newWeight;
		}
		
		/**
		 * @return heuristic of the plan
		 */
		public double getHeuristic() {
			return heuristic;
		}
		
	}
	
	/**
	 * @param tasks
	 * @param id
	 * @return task in tasks with a given id
	 */
	private Task getTask(TaskSet tasks, int id) {
		for(Task t: tasks) {
			if(t.id == id) {
				return t;
			}
		}
		return null;
	}
	
	/**
	 * @param tasks
	 * @param id
	 * @return true if the test tasks contain the task of a given id
	 */
	private boolean setContains(TaskSet tasks, int id) {
		for(Task t: tasks) {
			if(t.id == id) return true;
		}
		return false;
	}
	
	/**
	 * @param tasks
	 * @return heuristic for the remaining tasks
	 */
	private double computeHeuristic(TaskSet tasks) {
		double heuristic = Double.MAX_VALUE;
		// heuristic value = minimal distance between pickup city and delivery city of remaining tasks 
		// times the number of remaining tasks. The true distance can not be greater than this
		for(Task t: tasks) {
			double distance = t.pickupCity.distanceTo(t.deliveryCity);
			if(distance < heuristic) {
				heuristic = distance;
			}
		}
		heuristic *= tasks.size();
		return heuristic;
	}
	
	/**
	 * @param tasks
	 * @param remaining
	 * @return TaskSet of all tasks with the given IDs
	 */
	private TaskSet remainingSet(TaskSet tasks, HashSet<Integer> remaining) {
		TaskSet newSet = TaskSet.noneOf(tasks);
		for(Integer t: remaining) {
			newSet.add(getTask(tasks, t.intValue()));
		}
		return newSet;
	}
	
	/**
	 * Comparator used by the priority queue for the BFS algorithm. 
	 * The nodes are visited according to their depth
	 */
	public class BFSComparator implements Comparator<ExtendedPlan> {
		@Override
		public int compare(ExtendedPlan p1, ExtendedPlan p2) {
			if(p1.depth < p2.depth) return -1;
			else if(p1.depth > p2.depth) return 1;
			else return 0;
		}
	}
	
	/**
	 * Comparator used by the priority queue for the ASTAR algorithm.
	 * The nodes are visited according to the cost + heuristic.
	 */
	public class ASTARComparator implements Comparator<ExtendedPlan> {
		@Override
		public int compare(ExtendedPlan p1, ExtendedPlan p2) {
			// the cost is the total distance of the plan and the heuristic value 
			// heuristic value = minimal distance between pickup city and delivery city of remaining tasks 
			// times the number of remaining tasks 
			double cost1 = p1.getPlan().totalDistance() + p1.getHeuristic();
			double cost2 = p2.getPlan().totalDistance() + p2.getHeuristic();
			
			if(cost1 < cost2) {
				return -1;
			}
			else if(cost1 > cost2) {
				return 1;
			}
			else return 0;
		}
	}
}
