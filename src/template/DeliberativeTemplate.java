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
			plan = astarPlan(vehicle, tasks);
			break;
		case BFS:
			// ...
			plan = bfsPlan(vehicle, tasks);
			break;
		default:
			throw new AssertionError("Should not happen.");
		}
		System.out.println(plan.totalDistance());
		return plan;
	}
	
	private Plan astarPlan(Vehicle vehicle, TaskSet tasks) {
		Plan plan = null;
		return plan;
	}
	
	private Plan bfsPlan(Vehicle vehicle, TaskSet tasks) {
		double min = Double.MAX_VALUE;
		start = vehicle.getCurrentCity();
		int tempWeight = 0;
		for(Task t: vehicle.getCurrentTasks()) {
			tempWeight += t.weight;
		}
		
		ExtendedPlan plan = new ExtendedPlan(new Plan(start), 0, tempWeight, 0, tasks, vehicle.getCurrentTasks(), start);
		
		PriorityQueue<ExtendedPlan> queue = new PriorityQueue<ExtendedPlan>(100, new PlanComparator());
		queue.add(plan);
		
		while(queue.size() != 0) {
			ExtendedPlan first = queue.poll();
				
			if(first.getCount() == tasks.size() && first.plan.totalDistance() <= min) {
//				System.out.println("-----------PLAN FOR " +vehicle.id());
//				for(Action a: first.getPlan()) System.out.println(a.toString());
				return first.getPlan(); //Termination condition
			}
			
			for(Integer pickup: first.getRemaining()) {// Adds a pickup to a plan
				Task next;
				if(setContains(tasks, pickup.intValue())){
					next = getTask(tasks, pickup.intValue());
				}
				else{
					next = getTask(vehicle.getCurrentTasks(), pickup.intValue());
				}
				
				if (canPickup(vehicle, next, first)) {
					ExtendedPlan newPlan = new ExtendedPlan(
						first.getPlan(), 
						first.count, 
						first.getWeight() + next.weight,
						first.depth + 1,
						first.remaining, 
						first.carried, 
						first.getCity());
					for (City c: newPlan.getCity().pathTo(next.pickupCity)) {
						newPlan.getPlan().appendMove(c);
						// System.out.println("Move to " + c.name + " for pickup at " + next.pickupCity);
					}
					newPlan.setCity(next.pickupCity);
					newPlan.getPlan().appendPickup(next);
					newPlan.remaining.remove(pickup);
					newPlan.carried.add(pickup);
					queue.add(newPlan);
					
					if(newPlan.plan.totalDistance() < min && newPlan.count == tasks.size()){
						min = newPlan.plan.totalDistance();
					}
				}
			}

			for(Integer carried: first.getCarried()) { // Adds a delivery to a plan
				Task next;
				if(setContains(tasks, carried.intValue())){
					next = getTask(tasks, carried.intValue());
				}
				else{
					next = getTask(vehicle.getCurrentTasks(), carried.intValue());
				}
				
				ExtendedPlan newPlan = new ExtendedPlan(
					first.getPlan(), 
					first.count + 1, 
					first.depth + 1,
					first.getWeight() - next.weight, 
					first.remaining, 
					first.carried, 
					first.getCity());
				for (City c: newPlan.getCity().pathTo(next.deliveryCity)) {
					newPlan.getPlan().appendMove(c);
					// System.out.println("Move to " + c.name + " for delivery at " + next.deliveryCity);
				}
				newPlan.setCity(next.deliveryCity);
				newPlan.getPlan().appendDelivery(next);
				newPlan.carried.remove(carried);
				queue.add(newPlan);
				
				if(newPlan.plan.totalDistance() < min && newPlan.count == tasks.size()){
					min = newPlan.plan.totalDistance();
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
	
	private boolean canPickup(Vehicle vehicle, Task task, ExtendedPlan plan) {
		return (plan.getWeight() + task.weight <= vehicle.capacity());
	}
	
	private ExtendedPlan copyPlan(ExtendedPlan plan) {
		ExtendedPlan newPlan = new ExtendedPlan(
			new Plan(start), 
			plan.getCount(), 
			plan.getWeight(),
			plan.depth,
			plan.getRemaining(), 
			plan.getCarried(), 
			plan.getCity());
		for (Action a: plan.getPlan()) {
			newPlan.getPlan().append(a);
		}
		return newPlan;
	}
	
	private Plan copy(Plan plan) {
		Plan p = new Plan(start);
		for(Action a: plan) {
			p.append(a);
		}
		return p;
	}
	
	public class ExtendedPlan {
		private Plan plan; 
		private int count; //Number of tasks delivered
		private City city;
		private int weight; //Sum of all tasks carried by vehicle in plan
		private HashSet<Integer> carried = new HashSet<Integer>(); 
		private HashSet<Integer> remaining = new HashSet<Integer>();
		private int depth;
		
		public ExtendedPlan(Plan plan, int c, int w, int d, TaskSet tasks, TaskSet carrying, City city) {
			this.plan = copy(plan);
			count = c;
			weight = w;
			this.depth = d;
			this.city = city;
			for (Task t: tasks) {
				remaining.add(new Integer(t.id));
			}
			for (Task t: carrying) {
				carried.add(new Integer(t.id));
			}
		}
		
		public ExtendedPlan(Plan plan, int c, int w, int d, HashSet<Integer> tasks, HashSet<Integer> carrying, City city) {
			this.plan = copy(plan);
			count = c;
			weight = w;
			depth = d;
			this.city = city;
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
		
		public City getCity() {
			return city;
		}
		
		public void setCity(City newCity) {
			city = newCity;
		}
		
		public int getWeight() {
			return weight;
		}
		
		public void setWeight(int newWeight) {
			weight = newWeight;
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
	private boolean setContains(TaskSet tasks, int id) {
		for(Task t: tasks) {
			if(t.id == id) return true;
		}
		return false;
	}
	public class PlanComparator implements Comparator<ExtendedPlan> {
		@Override
		public int compare(ExtendedPlan p1, ExtendedPlan p2) {
			if(p1.depth < p2.depth){
				return 1;
			}
			else if(p1.depth == p2.depth){
				if (p1.getPlan().totalDistance() > p2.getPlan().totalDistance()) return -1;
				else if(p1.getPlan().totalDistance() < p2.getPlan().totalDistance()) return 1;	
				else return 0;
			}
			else{
				return -1;
			}
		}
	}
}
