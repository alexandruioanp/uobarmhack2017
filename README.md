# UoB ARM Hackathon 2017
Code developed during the 2017 ARM Hackathon at the University of Bristol (24 hours).

The project was a 2 DOF robotic arm built with cheap servos and shoddy mechanics. It had a pen attached to it, which it used to (try to) draw things.

Only a third of the project is hosted here. The rest will be linked via submodules.

This is a Python HTTP server that receives point coordinates via POST and does the inverse kinematics calculations needed to drive the servos.

See this [blog post](https://blog.alexandruioan.me/2017/01/31/the-2017-university-of-bristol-arm-hackathon/) for more details.

# Usage
This is Python 3 code.

python bruteforce.py [port]

| Default/hardcoded values |               |
| ------------------------ | ------------- |
| Port                     | 1180          |
| Serial port              | /dev/ttyACM0  |
| Baud rate                | 115200        |
