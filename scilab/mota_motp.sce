// clear all 
xdel(winsid());
clear;

//opens file
fd_gt=mopen('/home/andreu/Desktop/gt.txt','r');

//main loop: Parsing + GT-TRACKER OUTPUT mapping + 
end_flag = 0;
while (end_flag == 0) do
    
    //GROUND TRUTH PARSING
    gt = [];
    str_line = mgetl(fd_gt,1);
    if length(str_line) < 2 then
        end_flag = 1;
    else
        str_nums = strsplit(str_line,' ');        
        n_nums = size(str_nums);
        for i=1:n_nums(1)-1
            if str_nums ~= ' ' then
                gt = [gt strtod(str_nums(i))];
            end
        end
        disp(gt);
    end
    

    //TRACKER OPUTPUT PARSING
    
    //ESTABLISH BEST MAPPING
    
    //COMPUTE DISTANCES -> MOTP
    
    //CHECK MAPPING SWITCHES -> sw
    
    //COUNT FALSE POSITIVES -> fp
    
    //COUNT MISSINGS -> m
    
    //COMPUTE MOTA = (sw,fp,m)
    
end

//close file
mclose(fd_gt);

