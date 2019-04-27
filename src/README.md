# Read me

Copilot was built on the race car platform from MIT (https://github.com/mit-racecar/racecar/tree/master/racecar). The source files are in src/algorithms/wall_following folder. The rest of the files are left over from previous users of this car, and we did not change or delete those files. Even though the source files are in wall_following folder, nothing from the original wall_following are there.

# Launch instructions
cd into copilot, run catkin_make, source devel/setup.bash, roslaunch wall_following real_world_wall_following.

# Current capacities
Brake and stop in front of obstacles when speed is under 2 m/s.

Turn away from obstacles when speed is under 2 m/s, and angle into obstacle is less than 30 degrees. Otherwise, it will try to brake.
