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

```
git clone https://github.com/CosiNull/traffic_v1.git
cd traffic-v1
pip install pygame
python simulation.py
```

## Tweaking the Settings 
You can tweak the setting of the simulation by accesing the `settings.py` file.
The available variables to tweak are:
- The number of cars in the simulation `num_car`
- The path to the terrain stored in a pickle file `road_file`
- The seed of the simulation `seed`
- The simulation playspeed `play_speed`
- The screen width/height `width`/`height`

- The algorithm that the cars will use `func`:
Set `func = "dj"` for the cars to use Dijkstra's algorithm
Set `func = "as"` for the cars to use regular A*
Set `func = "mlt"` for the cars to use Hybrid Multi-agent A*




