function Y2qdq

    global Y num_body
    global body
    
    for indx = 2 : num_body + 1
        body(indx).qi = Y(indx - 1);
        body(indx).qi_dot = Y(indx + num_body - 1);
    end

end