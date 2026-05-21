from collections import deque

def bfs(initial_state):
    """
    Breadth-First Search (BFS) algorithm to find the shortest path from initial_state to a goal state.
    """
    # 1. Use collections.deque for an efficient FIFO queue.
    # The queue stores tuples of (current_state, path_to_current_state).
    queue = deque([(initial_state, [])])
    
    # 2. Maintain a visited set to track explored GameState objects.
    # Relying on their __hash__ and __eq__ methods.
    visited = {initial_state}

    # 5. Check if the initial state is already a goal state.
    if initial_state.is_goal_state():
        return []

    while queue:
        current_state, path = queue.popleft()
        
        # 3. Use state.get_successors(toward_walls=True) to generate next states.
        for action, cost, next_state in current_state.get_successors(toward_walls=True):
            
            # 4. Skip successor states that result in a collision.
            if next_state.is_collision_state():
                continue
            
            if next_state not in visited:
                # 5. Check if a state is the goal as soon as it's generated for efficiency.
                if next_state.is_goal_state():
                    return path + [action]
                
                visited.add(next_state)
                queue.append((next_state, path + [action]))
                
    # Return an empty list if no path is found.
    return []
