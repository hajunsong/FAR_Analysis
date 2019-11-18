function J = jacobian2

    global body
    
    Jv = zeros(3,6);
    Jw = zeros(3,6);

    for indx = 2 : 7
        body(indx).Aijpp_qi = [-sin(body(indx).qi) -cos(body(indx).qi) 0;cos(body(indx).qi) -sin(body(indx).qi) 0; 0 0 0];
        body(indx).Cij_Aijpp = body(indx - 1).Cij*body(indx).Aijpp;
        body(indx).Ai_Cij_Aijpp_qi = body(indx).Ai_Cij*body(indx).Aijpp_qi;
    end
    
    for indx = 2 : 7
        body(indx).A6_qi = zeros(3,3);
        body(indx).r6_qi = zeros(3,1);
        for indx2 = indx : 7
            if (indx2 == indx)
                body(indx).A6_qi = body(indx).A6_qi + body(indx2).Ai_Cij_Aijpp_qi;
            else
                temp = body(indx).A6_qi*body(indx2).Cij_Aijpp;
                body(indx).A6_qi = temp;
            end
            
            if indx2 < 7
                body(indx).r6_qi = body(indx).r6_qi + body(indx).A6_qi*body(indx2).sijp;
            end
        end
        body(indx).re_qi = body(indx).r6_qi + body(indx).A6_qi*body(end).sijp;
        body(indx).Ae_qi = body(indx).A6_qi*body(end).Cij;
        
        Jv(:, indx - 1) = body(indx).re_qi;
    end
    
    Ae_31 = body(end).Ae(3,1); 
    Ae_32 = body(end).Ae(3,2); 
    Ae_33 = body(end).Ae(3,3); 
    Ae_21 = body(end).Ae(2,1);	
    Ae_11 = body(end).Ae(1,1);
    
    roll_q_temp1 = Ae_32^2 + Ae_33^2;
    roll_q_temp2 = sqrt(roll_q_temp1);
    
    pitch_q_temp1 = sqrt(Ae_32^2 + Ae_33^2);
    pitch_q_temp2 = Ae_31^2 + pitch_q_temp1^2;
    pitch_q_temp3 = sqrt(pitch_q_temp2);
    
    yaw_q_temp1 = Ae_21^2 + Ae_11^2;
    yaw_q_temp2 = sqrt(yaw_q_temp1);
    
    for indx = 2 : 7
        body(indx).Ae_qi_31 = body(indx).Ae_qi(3,1);
        body(indx).Ae_qi_32 = body(indx).Ae_qi(3,2);
        body(indx).Ae_qi_33 = body(indx).Ae_qi(3,3);
        body(indx).Ae_qi_21 = body(indx).Ae_qi(2,1);
        body(indx).Ae_qi_11 = body(indx).Ae_qi(1,1);

        body(indx).roll_qi = ((roll_q_temp2 + Ae_33)*(body(indx).Ae_qi_32*Ae_33 - Ae_32*body(indx).Ae_qi_33))/(roll_q_temp2*(roll_q_temp1 + Ae_33*roll_q_temp2));

        body(indx).pitch_qi = -((pitch_q_temp3 + pitch_q_temp1)*(body(indx).Ae_qi_31*pitch_q_temp1 - Ae_31*(Ae_32*body(indx).Ae_qi_32 + Ae_33*body(indx).Ae_qi_33)/pitch_q_temp1))/(pitch_q_temp3*(pitch_q_temp2 + pitch_q_temp1*pitch_q_temp3));

        body(indx).yaw_qi = ((yaw_q_temp2 + Ae_11)*(body(indx).Ae_qi_21*Ae_11 - Ae_21*body(indx).Ae_qi_11))/(yaw_q_temp2*(yaw_q_temp1 + Ae_11*yaw_q_temp2));

        Jw(1, indx - 1) = body(indx).roll_qi;
        Jw(2, indx - 1) = body(indx).pitch_qi;
        Jw(3, indx - 1) = body(indx).yaw_qi;
    end
    
    J = [Jv;Jw];
    
end 