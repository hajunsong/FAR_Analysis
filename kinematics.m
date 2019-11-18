function kinematics

    global body num_body
    
    for indx = 2 : num_body + 1
        % orientation
        body(indx).Aijpp = [cos(body(indx).qi) -sin(body(indx).qi) 0; sin(body(indx).qi) cos(body(indx).qi) 0; 0 0 1];
        body(indx).Ai_Cij = body(indx - 1).Ai*body(indx - 1).Cij;
        body(indx).Ai = body(indx).Ai_Cij*body(indx).Aijpp;
        
        % position
        body(indx - 1).sij = body(indx - 1).Ai*body(indx - 1).sijp;
        body(indx).ri = body(indx - 1).ri + body(indx - 1).sij;
    end
    
    % end point
    body(num_body + 1).sij = body(num_body + 1).Ai*body(num_body + 1).sijp;
    body(num_body + 1).re = body(num_body + 1).ri + body(num_body + 1).sij;
    body(num_body + 1).Ae = body(num_body + 1).Ai*body(num_body + 1).Cij;
    
    body(num_body + 1).ori = mat2rpy(body(num_body + 1).Ae);
%     body(num_body + 1).roll = atan2(body(num_body + 1).Ae(3,2), body(num_body + 1).Ae(3,3));
%     body(num_body + 1).pitch = atan2(-body(num_body + 1).Ae(3,1),sqrt(body(num_body + 1).Ae(3,2)^2 + body(num_body + 1).Ae(3,3)^2));
%     body(num_body + 1).yaw = atan2(body(num_body + 1).Ae(2,1), body(num_body + 1).Ae(1,1));

end