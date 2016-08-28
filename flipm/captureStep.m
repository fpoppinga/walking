function stepX = captureStep(dx)
    global g;
    global h;
    stepX = -dx * sqrt(h / g);
end
