
//Main inspiration:
//    http://www.wikihow.com/Use-the-Hungarian-Algorithm 
//Others:
//    http://csclab.murraystate.edu/bob.pilgrim/445/munkres.html
//    http://www.hungarianalgorithm.com/examplehungarianalgorithm.php

function [mapping] = munkres(CC)

    //display input
    disp(CC);
    
    //step 1. Get sizes and ensure the matrix is square
    [nr nc] = size(CC);
    if nr > nc then //more rows than cols
        Csq = [CC ones(nr,nr-nc)*max(CC)];
    end
    if nc > nr then //more cols than rows
        Csq = [CC; ones(nc-nr,nc)*max(CC)];
    end
    nn = size(Csq,1);//number of rows = number of cols
    disp(Csq);

    //step 2. For each row in the matrix, substract the smallest of the row to all row values
    Cz = zeros(nn,nn);
    for ii=1:nn
        Cz(ii,:) = Csq(ii,:)-min(CC(ii,:));
    end
    disp(Cz);
    
    //step 3. If some column remains without 0, substract the minimum value of that column to all column elements
    for jj=1:nn
        nzc = 0;//non zero counter
        for ii=1:nn
            if Cz(ii,jj) == 0 then
                break;
            else
                nzc = nzc +1; 
            end
        end
        if (nzc == nr) then // column jj didn't have any 0
            Cz(:,jj) = Cz(:,jj)-min(Cz(:,jj));
        end
    end
    
    //step 4. Iteratively, Cover rows and columns with zeros , using minimum covering. 
    disp('Step4');
    while %T do
        [r_cover c_cover starred] = min_covering(Cz); //TO DO: EN LA SEGON ITERació està fallant la funció min_covering() !!!
          disp(Cz);
          disp(r_cover);
          disp(c_cover);        
//        disp(starred);
        if (size(r_cover,2)+size(c_cover,2)) == nn then
            disp('DONE !');
            break;
        else 
            //find the minimum uncovered value
            Czp = Cz;
            for (kk = 1:size(r_cover,2))
                Czp(r_cover(kk),:) = max(Cz);
            end
            for (kk = 1:size(c_cover,2))
                Czp(:,c_cover(kk)) = max(Cz);
            end
            min_uncovered = min(Czp); //disp(Czp);
            
            //add min_uncovered to every covered element
            for (kk = 1:size(r_cover,2))
                Cz(r_cover(kk),:) = Cz(r_cover(kk),:) + min_uncovered;
            end
            for (kk = 1:size(c_cover,2))
                Cz(:,c_cover(kk)) = Cz(:,c_cover(kk)) + min_uncovered
            end            
            
            //substract the minimum element from all the matrix
            min_Cz = min(Cz);
            Cz = Cz - min_Cz; 
        end
    end  
    
    //STEP 5 .
    //disp(Cz);  
    
    //return 
    mapping = [];
    
endfunction

function [row_cover, col_cover, starred] = min_covering(AA)
    row_cover = [];
    col_cover = [];
    zcr = zeros(1, nn); //counter of zeros per row
    zcc = zeros(1, nn); //counter of zeros per column
    for ii=1:nn //loop counting how many zeros at each row and column
        for jj=1:nn
            if AA(ii,jj) == 0 then
                zcr(ii) = zcr(ii)+1;
                zcc(jj) = zcc(jj)+1;
            end
        end
    end
    zc = [zcr zcc]; //concatenate zc rows and zc columns
    [zc_s,idx] = gsort(zc);
    starred = zeros(nn,nn); //1-> indicates starred zero
    for (kk = 1:2*nn)
        if idx(kk)>nn then //that's a column!
           jj = idx(kk)-nn;
           starred_count = 0; 
           for ii=1:nn
               if (AA(ii,jj) == 0) then
                   if starred(ii,jj) == 0 then
                       starred(ii,jj)=1;       
                       starred_count = starred_count +1; 
                   end                   
               end
           end
           if (starred_count ~=0) then
               col_cover = [col_cover jj];
           end
        else //that's a row
           ii = idx(kk) 
           starred_count = 0; 
           for jj=1:nn
               if (AA(ii,jj) == 0) then
                   if starred(ii,jj) == 0 then
                       starred(ii,jj)=1;       
                       starred_count = starred_count +1; 
                   end                   
               end
           end
           if (starred_count ~=0) then
               row_cover = [row_cover ii];
           end
        end
    end
//    disp(zc_s);
//    disp(idx);    
//    disp(starred);

endfunction

    

