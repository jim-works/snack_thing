Dependencies:
- streamlit `pip install streamlit`
- moveit, grasping_msgs, etc. Assume all modules we used for the homework.
- Requires you have a web browser (ui). Run in a conatiner with a gui, and use `apt install firefox`
    - note: if you don't want to use the ui, don't run the streamlit commands. You can use `echo "x,y" | nc 127.0.0.1:65432` to send the expected packets without the ui.

To run

- `catkin_make`
- `. devel/setup.bash`
- `roslaunch snack_thing gazebo.launch`
- wait for gazebo to open completely
- Open a new terminal and source again. Run `roslaunch snack_thing move.launch`
- Open a new terminal and run `python -m streamlit src/streamlit_file.py`

Our code is in the `snack_thing` package (src/snack_thing).