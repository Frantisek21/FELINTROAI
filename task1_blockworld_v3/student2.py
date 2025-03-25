from blockworld import BlockWorld
from queue import PriorityQueue

class BlockWorldHeuristic(BlockWorld):
    def __init__(self, num_blocks=5, state=None):
        BlockWorld.__init__(self, num_blocks, state)

    def heuristic(self, goal):
        self_state = self.get_state()
        goal_state = goal.get_state()
        
        # Convert to lists for easier manipulation
        self_stacks = [list(stack) for stack in self_state]
        goal_stacks = [list(stack) for stack in goal_state]
        
        # Flatten both configurations to identify blocks
        all_blocks = set()
        for stack in self_stacks:
            for block in stack:
                all_blocks.add(block)
        
        # Create a dictionary mapping each block to its position in the goal state
        # Position includes the stack and the position within the stack
        goal_positions = {}
        for stack_idx, stack in enumerate(goal_stacks):
            for pos_idx, block in enumerate(stack):
                goal_positions[block] = (stack_idx, pos_idx)
        
        # Count minimum moves needed
        moves_needed = 0
        
        # Check each stack in the current state
        for stack_idx, stack in enumerate(self_stacks):
            for pos_idx, block in enumerate(stack):
                # Get goal position of this block
                if block in goal_positions:
                    goal_stack_idx, goal_pos_idx = goal_positions[block]
                    
                    # If block is not in the correct position
                    if stack_idx != goal_stack_idx or pos_idx != goal_pos_idx:
                        # Count one move to relocate this block
                        moves_needed += 1
                        
                        # If this block has blocks on top of it, they must be moved first
                        moves_needed += len(stack) - pos_idx - 1
                        
                        # If in goal, blocks below this one must be in place
                        if goal_pos_idx > 0:
                            # Check if the stack below in current state matches goal
                            current_stack_below = []
                            if stack_idx < len(self_stacks) and pos_idx > 0:
                                current_stack_below = self_stacks[stack_idx][:pos_idx]
                            
                            goal_stack_below = goal_stacks[goal_stack_idx][:goal_pos_idx]
                            
                            # If they don't match, we need at least one more move
                            if current_stack_below != goal_stack_below:
                                moves_needed += 1
        
        return float(moves_needed)
class AStar():
    def search(self, start, goal):
        # Initialize open and closed sets
        open_set = PriorityQueue()
        closed_set = set()
        
        # For path reconstruction
        came_from = {}
        
        # Cost from start to current node
        g_score = {start: 0}
        
        # Estimated total cost from start to goal through current node
        f_score = {start: start.heuristic(goal)}
        
        # Add start node to open set with its f_score as priority
        open_set.put((f_score[start], 0, start))  # Added counter for tiebreaking
        counter = 1
        
        # For path reconstruction, keep track of actions
        action_to = {}
        
        while not open_set.empty():
            # Get the node with lowest f_score
            _, _, current = open_set.get()
            
            # If we found the goal, reconstruct and return the path
            if current == goal:
                path = []
                while current in came_from:
                    path.append(action_to[current])
                    current = came_from[current]
                path.reverse()
                return path
            
            # Add current to closed set
            if current in closed_set:
                continue
            
            closed_set.add(current)
            
            # Explore neighbors
            for action, neighbor in current.get_neighbors():
                # Skip if neighbor is already evaluated
                if neighbor in closed_set:
                    continue
                
                # Calculate tentative g_score
                tentative_g_score = g_score[current] + 1
                
                # If neighbor is not in open set or has a better g_score
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # Update path and scores
                    came_from[neighbor] = current
                    action_to[neighbor] = action
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + neighbor.heuristic(goal)
                    
                    # Add to open set
                    open_set.put((f_score[neighbor], counter, neighbor))
                    counter += 1
        
        # If we get here, no path was found
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