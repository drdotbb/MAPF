# MAPF
Multi-Agent Path Finding

Multi-Agent Path Finding (MAPF) and implement a single-agent solver, namely space-time A*, and parts of three MAPF solvers, namely prioritized planning,
Conflict-Based Search (CBS), and CBS with disjoint splitting.

## Unit Testing Suite (New)
Added a comprehensive unit test suite to validate the pathfinding algorithms.

### Features Tested
- **Independent Solver:** Verifies non-conflicting paths are found correctly.
- **CBS Solver:** Verifies conflict resolution between agents.
- **Prioritized Planning:** Verifies obstacle avoidance.

### Running Tests
```bash
pip install -r requirements.txt
pytest tests/
```
