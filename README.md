# MiniProject1

Navigate to folder where you want to save the repo then clone it.

```
git clone https://github.com/KarolRafalCharylo/MiniProject1
```

## Structure

The main file is `main.py`, this is the script to be executed with `ros_run`. The idea is that every functional part of the simulation is separated into it's own self contained file such as `fetch.py`, `move_to.py`, etc. These files contain the logic needed for that specific part of the simulation. This also helps with collaboration as we will not be working in the same file, make potential issues with git less likely.


## The auto_copy watcher

Since the python files need to be in the `~/catkin_ws/src/$YOUR_WORKSPACE_HERE$/scripts` directory you would need to manually copy them over and make them executable.

The `auto_copy.sh` script solves this issue. This script should be kept running in a terminal window while you work on the `.py` files in the repo because it detects when you create a new file or save an existing one and saves you the hassle of copying the files over to your catkin workspace and making them executable.

**IF** your workspace is the `hello_ros` workspace then you can run the script without any arguments.

```
./auto_copy.sh
```

**However**, if you are using a different workspace for the project, specify the path to the `/scripts` directory of that workspace as a parameter

```
./auto_copy.sh ~/catkin_ws/src/$YOUR_WORKSPACE_HERE$/scripts
```
