# PREDICTING AND REDUCING TRAFFIC IN A MULTI-AGENT SYSTEM

## Welcome!
This is my 2022 Science Fair Project to optimize traffic circulation with multi-agent pathfinding.
I built an algorithm that can predict future congestion and another one to adjust the paths of cars according to future congestion.
A simulation is then created to to evaluate these two algorithms.

- Future congestion prediction is implemented through an action priority queue.
- I used A* pathfinding to make the car prioritize busier intersections.

A Youtube Video is coming soon to explain my project!

## How to View the Simulation
1. Git clone the repository using `git clone https://github.com/CosiNull/traffic_v1.git`
2. Once inside the project directory, (assuming that you are using a Python 3.8 environment that already has Matplotlib and numpy) run `pip install pygame`
3. Run the simulation with `python simulation.py`

```sh```
git clone https://github.com/CosiNull/traffic_v1.git
pip install pygame
python simulation.py
```sh```