% Given a the pendulum speed dx0 as well as a desired velocity
% dvel (reffring to x = 0) the next pendulum origin is calculated from the
% energy equation. Thus if you one sets dvel to 0 this is equivalent to the
% capture step.

function stepX = desiredVel(dx0, dvel)
   global g;
   global h;
   stepX = -sqrt(h/g * (dx0^2-dvel^2));
end