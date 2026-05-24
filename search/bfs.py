from collections import deque

def reconstruct_path(parent, goal_features):
    actions = []
    current = goal_features


    while parent[current][0] is not None:
        current, action = parent[current]
        actions.append(action)

    actions.reverse()
    return actions


def bfs(initial_state):
   
    init_features = (
        initial_state.get_agent_position(), 
        tuple(sorted(initial_state.get_targets_positions()))
    )
    
   
    if initial_state.is_goal_state():
        return []

    frontier = deque([initial_state])
    visited = {init_features}

    parent = {
        init_features: (None, None)
    }

    while frontier:
        state = frontier.popleft()
        current_features = (
            state.get_agent_position(), 
            tuple(sorted(state.get_targets_positions()))
        )

      
        for action, cost, next_state in state.get_successors():
            
            next_features = (
                next_state.get_agent_position(), 
                tuple(sorted(next_state.get_targets_positions()))
            )

            if next_features in visited:
                continue

           
            visited.add(next_features)
            parent[next_features] = (current_features, action)
            
           
            if next_state.is_goal_state():
                return reconstruct_path(parent, next_features)
                
            frontier.append(next_state)

    return []