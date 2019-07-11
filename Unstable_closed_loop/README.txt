wheel_feedback

This project is a test of a closed-loop system involving motor-powered wheels.
An initial input is given in the form of turning the right wheel of the MiP.
This change in wheel position, in radians, is then taken as a negative duty
for the right wheel. The difference in wheel position, in radians, between the 
right and left wheels is then taken as the duty of the left wheel. Since the
right wheel will then constantly change position, this creates a feedback loop
for the left wheel.