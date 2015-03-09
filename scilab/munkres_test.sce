exec('/home/andreu/dev/ros_ws/src/pipol_tracker/scilab/munkres.sci');

AA = [10 19 8 15; 10 18 7 17; 13 16 9 14; 12 19 8 18; 14 17 10 19];
AA = [10 19 8 15; 10 18 10 17; 13 11 9 14; 12 13 8 18; 11 17 10 13];//crashes!! No 0 at last to column after step4 finished ...-> step 5 loops infintely
//AA = [10 19 8 15; 10 18 7 17; 11 16 9 14; 12 19 8 18; 14 17 10 19];
//AA = [10 1 8 15; 10 18 7 17; 11 16 9 14; 12 19 8 18; 14 17 10 19];
//AA = [9 1 8 15; 10 18 7 17; 11 16 9 14; 12 19 8 18; 14 17 10 19];
AA = [10 19 8 15; 10 18 10 17; 13 11 9 14; 12 13 8 18; 11 17 10 11]; //crashes!! ... idem
AA = [10 19 8 15; 10 18 10 17; 13 1 9 14; 12 13 8 18; 11 17 10 11]; //crashes!! ... idem

nn=6;
AA=zeros(nn,nn);
for ii=1:nn
    for jj=1:nn
        AA(ii,jj) = ii*jj;  //crashes!! 
    end
end

//call munkres
mapping = munkres(AA);

//remove mappings out of size
[nr,nc] = size(AA);
mapping_ok = []; 
for kk=1:size(mapping,1)
    if mapping(kk,1) <= nr then
        if mapping(kk,2) <= nc then
            mapping_ok = [mapping_ok; mapping(kk,:)];
        end
    end
end

//compute final cost
cost = 0;
for kk=1:size(mapping_ok,1)
    cost = cost + AA(mapping_ok(kk,1),mapping_ok(kk,2));
end

//display
disp(AA); 
disp(mapping_ok);
disp(cost);
