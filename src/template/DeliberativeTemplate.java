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
			plan = naivePlan(vehicle, tasks);
			break;
		case BFS:
			// ...
			plan = naivePlan(vehicle, tasks);
			break;
		default:
			throw new AssertionError("Should not happen.");
		}		
		return plan;
	}
	
	private Plan optPlan(Vehicle vehicle, TaskSet tasks) {
		start = vehicle.getCurrentCity(); 
				
		for(Task t: vehicle.getCurrentTasks()) {
			tasks.remove(t);
		}
		
		ExtendedPlan plan = new ExtendedPlan(new Plan(start), 0, tasks, vehicle.getCurrentTasks());
		
		PriorityQueue<ExtendedPlan> queue = new PriorityQueue<ExtendedPlan>(10, new PlanComparator());
		queue.add(plan);
		
		while(queue.size() != 0) {
			ExtendedPlan first = queue.poll();
			if(first.getCount() == tasks.size()) 
				return first.getPlan(); //Termination condition
			
			for(Integer carried: first.getCarried()) { // Adds a delivery to a plan
				ExtendedPlan newPlan = new ExtendedPlan(first.getPlan(), first.count+1,	first.remaining, first.carried);
				Task next = getTask(tasks, carried.intValue());
				newPlan.getPlan().appendMove(next.deliveryCity);
				newPlan.getPlan().appendDelivery(next);
				newPlan.carried.remove(carried);
				queue.add(newPlan);
			}
			
			for(Integer pickup: first.getRemaining()) { // Adds a pickup to a plan
				ExtendedPlan newPlan = new ExtendedPlan(first.getPlan(), first.count, first.remaining, first.carried);
				Task next = getTask(tasks, pickup.intValue());
				newPlan.getPlan().appendMove(next.pickupCity);
				newPlan.getPlan().appendPickup(next);
				newPlan.remaining.remove(pickup);
				newPlan.carried.add(pickup);
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
	
	private Task taskFrom(TaskSet tasks, City city) {
		Task res = null;
		for (Task task: tasks) {
			if (task.pickupCity.name == city.name) {
				res = task;
			}
		}
		return res;
	}
	
	private HashSet<Task> taskTo(HashSet<Task> tasks, City city) {
		HashSet<Task> res = new HashSet<Task>();
		for (Task task: tasks) {
			if (task.deliveryCity.name == city.name) {
				res.add(task);
			}
		}
		return res;
	}
	
	private boolean canPickup(Vehicle vehicle, Task task) {
		return (vehicle.getCurrentTasks().weightSum() + task.weight <= vehicle.capacity());
	}
	
	private ExtendedPlan copyPlan(ExtendedPlan plan) {
		ExtendedPlan newPlan = new ExtendedPlan(new Plan(start), plan.getCount(), plan.getRemaining(), plan.getCarried());
		for (Action a: plan.getPlan()) {
			newPlan.getPlan().append(a);
		}
		return newPlan;
	}
	
	public class ExtendedPlan {
		private Plan plan; 
		//private City city; //Final city in which vehicle arrived
		private int count; //Number of tasks delivered
		//private HashSet<Task> tasks = new HashSet<Task>();
		private HashSet<Integer> carried = new HashSet<Integer>();
		private HashSet<Integer> remaining = new HashSet<Integer>();
		
		public ExtendedPlan(Plan plan, int c, TaskSet tasks, TaskSet carrying) {
			this.plan = plan;
			count = c;
			for (Task t: tasks) {
				remaining.add(new Integer(t.id));
			}
			for (Task t: carrying) {
				carried.add(new Integer(t.id));
			}
		}
		
		public ExtendedPlan(Plan plan, int c, HashSet<Integer> tasks, HashSet<Integer> carrying) {
			this.plan = plan;
			count = c;
			for (Integer t: tasks) {
				remaining.add(new Integer(t.intValue()));
			}
			for (Integer t: carrying) {
				carried.add(new Integer(t.intValue()));
			}
		}
		
		public Plan getPlan() {
			return plan;
		}
		
		public int getCount() {
			return count;
		}
		
		public HashSet<Integer> getCarried() {
			return carried;
		}
		
		public HashSet<Integer> getRemaining() {
			return remaining;
		}
		
		public void setCount(int c) {
			count = c;
		}
		
		public void increment() {
			count++;
		}
		
		public void addCarried(Integer i) {
			carried.add(i);
		}
		
		public void remove(Integer i) {
			carried.add(i);
		}
	}
	
	private Task getTask(TaskSet tasks, int id) {
		for(Task t: tasks) {
			if(t.id == id) {
				return t;
			}
		}
		return null;
	}
	
	public class PlanComparator implements Comparator<ExtendedPlan> {
		@Override
		public int compare(ExtendedPlan p1, ExtendedPlan p2) {
			if (p1.getPlan().totalDistance() > p2.getPlan().totalDistance()) return -1;
			else if(p1.getPlan().totalDistance() < p2.getPlan().totalDistance()) return 1;	
			else return 0;
		}
	}
}