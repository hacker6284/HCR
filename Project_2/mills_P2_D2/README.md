`stingray_sim` is the package, and my script is in the scripts folder.
My launch file is called `follow_wall.launch` and is located in the launch folder of the package. The package is designed to be placed in the catkin workspace of the `Stingray_Sim` folder from the gitlab.
`stingray_setup.bash` needs to be sourced in addition to the one in the devel folder in my package in order for my package to run.

Once the launchfile launches, the prompt may not show up, but type 1 and hit enter to train the model, and use ^C to kill it when training is complete. Otherwise, you can type 2 and hit enter after it launches to follow based on the learned Q-table.
