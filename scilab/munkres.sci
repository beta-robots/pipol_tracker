//Main inspiration:
//    http://www.wikihow.com/Use-the-Hungarian-Algorithm 
//Others:
//    http://csclab.murraystate.edu/bob.pilgrim/445/munkres.html
//    http://www.hungarianalgorithm.com/examplehungarianalgorithm.php
//
//definitions: 
// - degree of a zero: in a matrix A, if a_ij = 0, we say that it has degree d when "zeros in row i + zeros in column j = d" 
// - degree matrix: the degree matrix  of A, D(A) is a matrix where each element dij is the degree of the zero. If ai_ij was non-zero, then d_ij = 0.

function [mapping] = munkres(CC)

    //display input
    //disp(CC);
    
    //step 1. Get sizes and ensure the matrix is square
    [nr nc] = size(CC);
    if nr == nc then
        Csq = CC;
    else
        if nr > nc then //more rows than cols
            Csq = [CC ones(nr,nr-nc)*max(CC)];
        end
        if nc > nr then //more cols than rows
            Csq = [CC; ones(nc-nr,nc)*max(CC)];
        end        
    end
    nn = size(Csq,1);//number of rows = number of cols
    //disp(Csq);

    //step 2. For each row in the matrix, substract the smallest of the row to all row values
    Cz = zeros(nn,nn);
    for ii=1:nn
        Cz(ii,:) = Csq(ii,:)-min(CC(ii,:));
    end
    //disp(Cz);
    
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
    //disp('Step4');
    while %T do
        [r_cover c_cover starred] = min_covering(Cz); //TO DO: EN LA SEGON ITERació està fallant la funció min_covering() !!!
        //disp(Cz);
        //disp(r_cover);
        //disp(c_cover);        
        //disp(starred);
        if (size(r_cover,2)+size(c_cover,2)) == nn then
            //disp('DONE !');
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
    
    //STEP 5 . Choose assignements
    //run over Cz rows.
    mapping = [];
    ii=1;
    while size(mapping,1) ~= nn do
        jj = find(~Cz(ii,:));
        nz =  size(jj,2);
        if nz == 1 then
            mapping = [mapping; [ii jj]];
            Cz(ii,:) = ones(1,nn); //mask row
            Cz(:,jj) = ones(nn,1); //mask column
        end
        pause
        ii = modulo(ii,nn)+1;
    end
    
endfunction


function [row_cover, col_cover, starred] = min_covering(AA)
    row_cover = [];
    col_cover = [];
    nn = size(AA,1)
    
    //count zeros per row and per column
    zcr = zeros(2, nn); //row1: counter of zeros per row, row2: per each 0 in ij, count how many other 0 are in row i and in column j
    zcc = zeros(2, nn); //row1: counter of zeros per column, row2: per each 0 in ij, count how many other 0 are in row i and in column j
    for ii=1:nn //loop counting how many zeros at each row and column
        for jj=1:nn
            if AA(ii,jj) == 0 then
                zcr(1,ii) = zcr(1,ii)+1;
                zcc(1,jj) = zcc(1,jj)+1;
                zcr(2,ii) = zcr(2,ii) + size(find(~AA(ii,:)),2) -1 + size(find(~AA(:,jj)),2) -1;
                zcc(2,jj) = zcc(2,jj) + size(find(~AA(ii,:)),2) -1 + size(find(~AA(:,jj)),2) -1;
            end
        end
    end
    zc = [zcr zcc]; //concatenate zc rows and zc columns
    [zc_s,idx] = gsort(zc(1,:));//idx keeps the index of the original elements in a sorted way
    zc_s_row2 = [];
    for (kk=1:2*nn)//rearrange the second row following the above sort of the first row
        zc_s_row2 = [zc_s_row2 zc(2,idx(kk))];
    end
    zc_s = [zc_s; zc_s_row2];
    
    //sort again zc_s by increasing order of row 2, but only in case of tie in row 1, 
    kk=1; 
    while kk<=2*nn
        idx_s = [];
        tie_idx = find(~(zc_s(1,:)-zc_s(1,kk)));//find all entries in row 1 equal to zc_s(1,kk)
//        pause
        if size(tie_idx,2) > 1 then
            sub_zc_s = zc_s(:,tie_idx(1):tie_idx($));
            [sub_zc_s_s,idx2] = gsort(sub_zc_s(2,:),'g','i');
            for ll=1:size(tie_idx,2)
                idx_s = [idx_s idx(kk+idx2(ll)-1)];//rearrange idx following the above sort of the second row in case of ties
            end
//            pause
            zc_s(2,tie_idx(1):tie_idx($)) = sub_zc_s_s;
            idx(tie_idx(1):tie_idx($)) = idx_s;
            kk = tie_idx($) + 1;
//            pause
        else
            kk = kk + 1;
        end
    end
//    disp('zc: '); disp(zc);
//    disp('idx2: '); disp(idx2);
//    disp('idx: '); disp(idx);    
//    disp('zc_s: '); disp(zc_s);
    
    starred = zeros(nn,nn); //1-> indicates starred zero
    for (kk = 1:2*nn)
        if idx(kk)>nn then //that's a column!
           jj = idx(kk)-nn;
           starred_count = 0; 
           for ii=1:nn
               if (AA(ii,jj) == 0) then //if it was 0 ...
                   if starred(ii,jj) == 0 then // ... and not yet starred
                       starred(ii,jj)=1; //star it      
                       starred_count = starred_count +1; //count it
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

endfunction

function [DD] = degreeMatrix(AA)
    [nr nc] = size(AA);
    DD = zeros(nr,nc);
    for ii=1:nr
        for jj=1:nc
            if AA(ii,jj) == 0 then     
                dr_ii = size(find(~AA(ii,:)),2);    
                dc_jj = size(find(~AA(:,jj)),2);
                DD(ii,jj) = dr_ii + dc_jj -1; //-1 beacuse a_ij zero is counted twice
            end
        end
    end
endfunction
