# Reports the state of a joysticks axes and buttons.
Header header           # timestamp in the header is the time the data is received from the joystick
float32[] axes          # the axes measurements from a joystick
string[] button     	# button name and event (released/pressed) read from the 3d mouse
