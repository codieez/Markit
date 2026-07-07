# Markit

Markit is a Python-based Mars rover simulator that generates random terrain, computes an optimal path with A* search, executes the mission, and exports a visual mission report.

## Features

- Grid-based terrain generation with configurable obstacle density.
- Deterministic maps via seed-based randomization.
- A* pathfinding with Manhattan heuristic.
- Mission execution with console map output.
- Automatic mission graph export to `mission_graphs.png`.

## Tech Stack

- Python 3
- Matplotlib (for graph export)

## Getting Started

### 1. Install Dependencies

```bash
pip install matplotlib
```

### 2. Run the Simulation

```bash
python3 mars_rover_sim.py
```

## Output

Running the simulator produces:

- A text-based final terrain map in the terminal.
- Mission result summary (success/failure and move count).
- A graph image file: `mission_graphs.png`.

## Project Structure

- `mars_rover_sim.py`: Main simulation, rover logic, path planner, and graph export.

## Configuration

Simulation defaults are set in `Simulation(...)` in `mars_rover_sim.py`:

- `rows=12`
- `cols=20`
- `obstacle_ratio=0.20`

You can change these values to generate different mission environments.
