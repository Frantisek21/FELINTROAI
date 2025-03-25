from blockworld import BlockWorld
from queue import PriorityQueue

class BlockWorldHeuristic(BlockWorld):
    def __init__(self, num_blocks=5, state=None):
        BlockWorld.__init__(self, num_blocks, state)

    def heuristic(self, goal):
        """
        Heuristic: +1 if a block is above or below incorrectly, +2 if both are wrong
        """
        self_state = self.get_state()
        goal_state = goal.get_state()
        
        # Convert frozensets to lists for easier manipulation
        current_stacks = [list(stack) for stack in self_state]
        goal_stacks = [list(stack) for stack in goal_state]
        
        # Maps to track block relationships (what's above and below each block)
        current_relationships = {}
        goal_relationships = {}
        
        # Process current state relationships
        for stack in current_stacks:
            for i, block in enumerate(stack):
                # In BlockWorld, index 0 is the top of the stack
                above = None if i == 0 else stack[i-1]
                below = None if i == len(stack)-1 else stack[i+1]
                current_relationships[block] = {'above': above, 'below': below}
        
        # Process goal state relationships
        for stack in goal_stacks:
            for i, block in enumerate(stack):
                above = None if i == 0 else stack[i-1]
                below = None if i == len(stack)-1 else stack[i+1]
                goal_relationships[block] = {'above': above, 'below': below}
        
        # Calculate heuristic value
        heuristic_value = 0
        
        for block in current_relationships:
            if block in goal_relationships:
                current_rel = current_relationships[block]
                goal_rel = goal_relationships[block]
                
                # Check above relationship
                above_wrong = current_rel['above'] != goal_rel['above']
                
                # Check below relationship
                below_wrong = current_rel['below'] != goal_rel['below']
                
                # Apply penalties based on heuristic definition
                if above_wrong and below_wrong:
                    heuristic_value += 2  # Both wrong
                elif above_wrong or below_wrong:
                    heuristic_value += 1  # Only one is wrong
        
        return float(heuristic_value)


class AStar():
    def search(self, start, goal):
        # Initialize the priority queue for open nodes
        open_set = PriorityQueue()
        
        # Set to keep track of visited nodes
        closed_set = set()
        
        # Dictionary to track the path
        came_from = {}
        action_to = {}
        
        # Cost from start to node
        g_score = {start: 0}
        
        # Estimated total cost
        f_score = {start: start.heuristic(goal)}
        
        # Add start to open set
        # Format: (f_score, g_score, counter, node)
        open_set.put((f_score[start], 0, 0, start))
        counter = 1
        
        while not open_set.empty():
            # Get node with lowest f_score
            _, _, _, current = open_set.get()
            
            # Found the goal
            if current == goal:
                path = []
                while current in came_from:
                    path.append(action_to[current])
                    current = came_from[current]
                path.reverse()
                return path
            
            # Skip if already processed
            if hash(current) in closed_set:
                continue
                
            # Mark as processed
            closed_set.add(hash(current))
            
            # Process neighbors
            for action, neighbor in current.get_neighbors():
                # Skip if already processed
                if hash(neighbor) in closed_set:
                    continue
                
                # Calculate new cost
                tentative_g_score = g_score[current] + 1
                
                # Update if better path found
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    action_to[neighbor] = action
                    g_score[neighbor] = tentative_g_score
                    
                    # Calculate f_score
                    h_score = neighbor.heuristic(goal)
                    f_score[neighbor] = tentative_g_score + h_score
                    
                    # Add to open set
                    open_set.put((f_score[neighbor], tentative_g_score, counter, neighbor))
                    counter += 1
        
        # No path found
        return None
    
if __name__ == '__main__':
	# Here you can test your algorithm. You can try different N values, e.g. 6, 7.
	N = 5

	start = BlockWorldHeuristic(N)
	goal = BlockWorldHeuristic(N)

	print("Searching for a path:")
	print(f"{start} -> {goal}")
	print()

	astar = AStar()
	path = astar.search(start, goal)

	if path is not None:
		print("Found a path:")
		print(path)

		print("\nHere's how it goes:")

		s = start.clone()
		print(s)

		for a in path:
			s.apply(a)
			print(s)

	else:
		print("No path exists.")

	print("Total expanded nodes:", BlockWorld.expanded)