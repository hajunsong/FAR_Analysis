function define_Y_vector

    global body
    global Y
    
    Y = [body.qi, body.qi_dot]';

end