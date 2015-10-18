package template;

/* import table */
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
			plan = optPlan(vehicle, tasks);
			break;
		default:
			throw new AssertionError("Should not happen.");
		}		
		return plan;
	}
	
	private Plan optPlan(Vehicle vehicle, TaskSet tasks) {
		City current = vehicle.getCurrentCity();
		ExtendedPlan plan = new ExtendedPlan(new Plan(current), current);
		
		for(Task t: vehicle.getCurrentTasks()) {
			tasks.remove(t);
		}
		
		PriorityQueue<ExtendedPlan> queue = new PriorityQueue<ExtendedPlan>(10, new PlanComparator());
		queue.add(plan);
		
		while(queue.size() != 0) {
			ExtendedPlan first = queue.poll();
			if (first.getCount() == tasks.size()) {
				return first.getPlan();
			}
			
			//Delivery
			for (Task t: taskTo(first.getTasks(), current)) {
				first.getPlan().appendDelivery(t);
				first.increment();
			}
			
			//Pickup
			HashSet<Task> newTasks = tasksFrom(tasks, current);
			ExtendedPlan newPlan = copyPlan(first, current);
			boolean pickedUp = false;
			if (newTask != null && canPickup(vehicle, newTask) && !(newPlan.getTasks().contains(newTask))) {
				pickedUp = true;
				newPlan.addTask(newTask);
				newPlan.getPlan().appendPickup(newTask);
			}
			
			for(Task t: tasksFrom(newTasks, tasks, current, newPlan.getTasks(), vehicle)) {
				
			}
			//Move
			
			for (City neighbour: current.neighbors()) {
				ExtendedPlan nPlan = copyPlan(first, neighbour);
				nPlan.getPlan().appendMove(neighbour);
				queue.add(nPlan);
				
				if(pickedUp) {
					ExtendedPlan nPlan1 = copyPlan(newPlan, neighbour);
					nPlan1.getPlan().appendMove(neighbour);
					queue.add(nPlan1);
				}
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
	
	// add all tasks from the city (only available tasks and not exceeding a weight)
	private HashSet<Task> tasksFrom(HashSet<Task> newTasks, TaskSet tasks, City current, HashSet<Task> planTasks, Vehicle vehicle) {
		HashSet<Task> res = new HashSet<Task>();
		
		for (Task task: tasks) {
			if (task.pickupCity.name == current.name) {
				
				res.add(task);
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
	
	private ExtendedPlan copyPlan(ExtendedPlan plan, City city) {
		ExtendedPlan newPlan = new ExtendedPlan(new Plan(city), city);
		for (Action a: plan.getPlan()) {
			newPlan.getPlan().append(a);
		}
		newPlan.setCount(plan.getCount());
		return newPlan;
	}
	
	private boolean planFinished(PriorityQueue<Plan> plans) {
		boolean res = false;
		for (Plan p: plans) {
			 res = p.isSealed() ? true : false;
		}
		return res;
	}
	
	public class ExtendedPlan {
		private Plan plan; 
		private City city; //Final city in which vehicle arrived
		private int count; //Number of tasks delivered
		private HashSet<Task> tasks = new HashSet<Task>();
		
		public ExtendedPlan(Plan plan, City current, int c) {
			this.plan = plan;
			count = c;
			city = current;
		}
		
		public ExtendedPlan(Plan plan, City current) {
			this.plan = plan;
			city = current;
			count = 0;
		}
		
		public Plan getPlan() {
			return plan;
		}
		
		public int getCount() {
			return count;
		}
		
		public HashSet<Task> getTasks() {
			return tasks;
		}
		
		public void setCount(int c) {
			count = c;
		}
		
		public void increment() {
			count++;
		}
		
		public City getCity() {
			return city;
		}
		
		public void setCity(City c) {
			city = c;
		}
		
		public void addTask(Task t) {
			tasks.add(t);
		}
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


