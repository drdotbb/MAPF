import pytest
from pathlib import Path
from single_agent_planner import get_sum_of_cost
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver

# Helpers to create simple test maps
def create_simple_map():
    """
    Creates a 3x3 map with no obstacles:
    . . .
    . . .
    . . .
    """
    return [[False, False, False],
            [False, False, False],
            [False, False, False]]

def create_obstacle_map():
    """
    Creates a 3x3 map with center obstacle:
    . . .
    . @ .
    . . .
    """
    return [[False, False, False],
            [False, True, False],
            [False, False, False]]

def test_independent_solver_simple():
    """
    Test Independent Solver on a simple conflict-free path.
    Agent 0: (0,0) -> (0,2)
    Agent 1: (2,0) -> (2,2)
    """
    my_map = create_simple_map()
    starts = [(0, 0), (2, 0)]
    goals = [(0, 2), (2, 2)]
    
    solver = IndependentSolver(my_map, starts, goals)
    paths = solver.find_solution()
    
    assert len(paths) == 2
    # Check path validity (start and end)
    assert paths[0][0] == (0,0)
    assert paths[0][-1] == (0,2)
    assert paths[1][0] == (2,0)
    assert paths[1][-1] == (2,2)
    
    # Expected cost: 2 steps each -> total 4
    assert get_sum_of_cost(paths) == 4

def test_cbs_solver_conflict():
    """
    Test CBS Solver where agents must cross paths.
    Agent 0: (0,0) -> (0,2)
    Agent 1: (0,2) -> (0,0)
    They are on the same row moving towards each other.
    """
    my_map = create_simple_map()
    starts = [(0, 0), (0, 2)]
    goals = [(0, 2), (0, 0)]
    
    # CBS should resolve the conflict
    solver = CBSSolver(my_map, starts, goals)
    paths = solver.find_solution()
    
    assert paths is not None
    assert len(paths) == 2
    
    # Verify no collision at any time step
    max_time = max(len(p) for p in paths)
    for t in range(max_time):
        positions = []
        for p in paths:
            if t < len(p):
                positions.append(p[t])
            else:
                positions.append(p[-1]) # Stay at goal
        
        # Check for vertex collisions (duplicates in positions)
        assert len(positions) == len(set(positions)), f"Collision detected at time {t}"

def test_prioritized_solver_obstacle():
    """
    Test Prioritized Solver navigating around an obstacle.
    Agent 0: (1,0) -> (1,2) through (1,1) which is blocked
    """
    my_map = create_obstacle_map()
    starts = [(1, 0)]
    goals = [(1, 2)]
    
    solver = PrioritizedPlanningSolver(my_map, starts, goals)
    paths = solver.find_solution()
    
    assert paths is not None
    path = paths[0]
    
    # Should not step on (1,1)
    assert (1,1) not in path
    
    # Should reach goal
    assert path[-1] == (1,2)

