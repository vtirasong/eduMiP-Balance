balance_mip

This project adds to balance_body by also stabilizing wheel position.
In addition to the implementation of controller D1, controller D2 is
used in difference equation calculations to produce an appropriate
theta reference that will stabilize the MiP body with respect to the
y-axis for the current wheel angular position.