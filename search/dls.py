def dls(initial_state, limit=200):
    """
    Depth-Limited Search (DLS) algorithm to find a path from initial_state to a goal state within a depth limit.
    """
    # Stack stores tuples of (current_state, path_to_current_state, current_depth).
    stack = [(initial_state, [], 0)]
    
    # visited stores the minimum depth at which each state was reached.
    # This allows re-exploring a state if it's reached at a shallower depth.
    visited = {initial_state: 0}

    while stack:
        current_state, path, depth = stack.pop()
        
        if current_state.is_goal_state():
            return path
            
        if depth < limit:
            # We explore successors in reverse order to match recursive DFS behavior if needed,
            # but here order doesn't strictly matter for DLS correctness.
            for action, cost, next_state in current_state.get_successors(toward_walls=True):
                if next_state.is_collision_state():
                    continue
                
                new_depth = depth + 1
                if next_state not in visited or new_depth < visited[next_state]:
                    visited[next_state] = new_depth
                    stack.append((next_state, path + [action], new_depth))
                
    return []
