# scarab_gazebo
Gazebo simulation files for Scarab robot in some random place in the Rocky Mountains.

## Setup
The generated heightmap seems to require that it's stored in your `~/.gazebo/models` folder. This is kinda inconvenient. I might try to change that later.

1) `cd <path to workspace>/src/scarab_gazebo/Media/models/`
2) `cp -r rocky_mtn/ ~/.gazebo/models/` (You can make the directory if it doesn't already exist)

## Run Simulation
To run the simulation for the Rocky Mountain environment:
```
roslaunch scarab_gazebo scarab.launch
```

You can specify which launch file to run. Launch file should be stored in `<path to your workspace>/src/scarab_gazebo/launch`. For example, to run `scarab.launch` (the default world that only contained the scarab robot):

```
roslaunch scarab_gazebo scarab.launch world:=scarab
```

TODO:
1) Check if querying series of Points is suitable for getSucessors()

