function save_data_motion(fp)

    global body t_current NRcount
    
    fprintf(fp, '%.7f\t', t_current);
    for i = 2 : 7
        fprintf(fp, '%.7f\t', body(i).qi);
    end

    kinematics();
    fprintf(fp, '%.7f\t%.7f\t%.7f\t', body(end).re');
    fprintf(fp, '%.7f\t%.7f\t%.7f\t', body(end).ori');
    
    for i = 2 : 7
        fprintf(fp, '%.7f\t', body(i).qi_dot);
    end
    
    fprintf(fp, '%.7f\t%.7f\t%.7f\t', body(end).re_dot');
    fprintf(fp, '%.7f\t%.7f\t%.7f\t', body(end).we');
%     
%     fprintf(fp, '%d\t', NRcount);
    fprintf(fp, '\n');

end