function save_data_dynamics(fp)

    global body t_current num_body
    
    fprintf(fp, '%.7f\t', t_current);
    for i = 2 : 7
        if i > num_body + 1
            fprintf(fp, '%.7f\t', 0);
        else
            fprintf(fp, '%.7f\t', body(i).qi);
        end
    end
    for i = 2 : 7
        if i > num_body + 1
            fprintf(fp, '%.7f\t', 0);
        else
            fprintf(fp, '%.7f\t', body(i).qi_dot);
        end
    end
    for i = 2 : 7
        if i > num_body + 1
            fprintf(fp, '%.7f\t', 0);
        else
            fprintf(fp, '%.7f\t', body(i).qi_ddot);
        end
    end
    
    fprintf(fp, '\n');

end