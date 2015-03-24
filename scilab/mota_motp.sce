// clear all 
xdel(winsid());
clear;

//includes
exec('/home/andreu/dev/ros_ws/src/pipol_tracker/scilab/munkres.sci');

//opens file
fd_gt=mopen('/home/andreu/dataSets/people_tracking/reem/ground_truth/20140925_twoPeople.txt','r');
fd_tracks=mopen('/home/andreu/dataSets/people_tracking/reem/tracker_results/20140925_twoPeople_1_tree_all.txt','r');

//plot y/n
plot_flag = 1;


//GROUND TRUTH PARSING
gt = list();
while (1) do
    gt_row = [];
    str_line = mgetl(fd_gt,1);
    if length(str_line) < 2 then
        break;
    else
        tkns = tokens(str_line,' ');
        for i=1:size(tkns,1)
            gt_row = [gt_row strtod(tkns(i))];
        end
        gt($+1) = gt_row;
    end
end

//TRACKER OPUTPUT PARSING
tracks = list();
while (1) do    
    tracks_row=[];
    str_line = mgetl(fd_tracks,1);
    if length(str_line) < 2 then
        break;
    else
        tkns = tokens(str_line,' ');
        for i=1:size(tkns,1)
            tracks_row = [tracks_row strtod(tkns(i))];
        end
        tracks($+1) = tracks_row;
    end
end

//delete tracks rows out of ground truth time interval
while ( tracks(1)(1) < gt(1)(1) )
    tracks(1) = null();
end
while ( tracks($)(1) > gt($)(1) )
    tracks($) = null();
end

//interpolate ground truth at result time stamp point
ii = 1;
gt_intp = list();
for jj=1:size(tracks)
    ts = tracks(jj)(1);//get time stamp
    gt_intp_row = ts;
    while ( ts > gt(ii)(1) ) do
        ii = ii + 1;
    end
    ts_gt_1 = gt(ii-1)(1);//ts just before
    ts_gt_2 = gt(ii)(1);//ts just after
    alpha = (ts - ts_gt_1) / (ts_gt_2 - ts_gt_1);
    Nt_gt = ( min( size(gt(ii),2),size(gt(ii-1),2) ) -1 ) / 3; //number of targets 
    if Nt_gt > 0 then
        for kk=1:Nt_gt
            //check "out of horizon"
            if ( gt(ii-1)(kk*3) < 0 ) | ( gt(ii-1)(kk*3) < 0 ) then
                gt_x = -1; //label as "out of horizon"
                gt_y = -1; //label as "out of horizon"
            else //compute linear interpolation
                gt_x = alpha*gt(ii)(kk*3) + (1-alpha)*gt(ii-1)(kk*3);
                gt_y = alpha*gt(ii)(kk*3+1) + (1-alpha)*gt(ii-1)(kk*3+1);
            end
            gt_intp_row = [gt_intp_row gt(ii-1)(kk*3-1) gt_x gt_y];
        end
    end
    gt_intp($+1) = gt_intp_row;
end

//set up plot
if plot_flag then
    fig1 = figure(0);
    fig1.background = 8;
    ah = gca();
    ah.isoview = "on";
    ah.x_label.text = "$y [m] (lateral)$";
    ah.x_label.font_size = 4;
    ah.y_label.text = "$x [m] (fwd)$";
    ah.y_label.font_size = 4;
    ah.grid = [1,1,1];
    ah.grid_position = "background";
    ah.auto_clear = "off";
    ah.auto_scale = "off";
    ah.data_bounds = [-3 0; 3 5];
    plot_colors = ["r";"g";"b";"k";"y"];
end

//MAIN LOOP TO COMPUTE MOT METRICS
for tt=1:size(tracks) //tt tracker iteration index (coincide with gorund truth interpolation index)
    
    //1. Filter out tracks with status lower than 15
    Nt_tracks = (size(tracks(tt),2)-1) / 4; //number of track targets (all)
    tk_sel = tracks(tt)(1); //track selection with status > 15. Init with timestamp
    for kk= 1:Nt_tracks
        if tracks(tt)(kk*4-1) >= 15 then
            tk_sel = [tk_sel tracks(tt)(kk*4-2:kk*4+1)]
        end
    end
    
    //2. Build D2 matrix. Sq Euclidean distance between all track-gt pairs 
    Nt_gt = (size(gt_intp(tt),2)-1) / 3; //number of gt targets 
    Nt_tracks = (size(tk_sel,2)-1) / 4; //number of track targets (selected according status)
    D2 = zeros(Nt_gt, Nt_tracks);
    for ii=1:Nt_gt //ii row/gt index
        gt_pos = [gt_intp(tt)(ii*3); gt_intp(tt)(ii*3+1)]; //gt ii position
        for jj= 1:Nt_tracks //jj column/track index
            tk_pos = [tk_sel(jj*4); tk_sel(jj*4+1)]; //track jj position
            D2(ii,jj) = (tk_pos(1)-gt_pos(1))^2 + (tk_pos(2)-gt_pos(2))^2; //sq euclidean distance
        end
    end
    
    //2. Check if any gt is further than d2_th to all tracks
    d2_th = 0.5*0.5; //0.5m threshold
    D2_chkd_rows = [];
    unassociated_gt = [];
    associated_gt = [];
    for ii=1:Nt_gt
        if ( min( D2(ii,:) ) > d2_th ) then
            unassociated_gt = [unassociated_gt gt_intp(tt)(ii*3-1)]; //save gt id as unassociated
        else
            D2_chkd_rows = [D2_chkd_rows; D2(ii,:)];
            associated_gt = [associated_gt gt_intp(tt)(ii*3-1)]; //save gt id as associated
        end
    end
    fn_count = size(unassociated_gt,2); //false negative counter: cases where a gt target is not tracked
    
    //3. In case D2_chkd_rows is non-empty, Check if any track is further than d2_th to all gt's
    unassociated_tk = [];
    associated_tk = [];
    D2_chkd = [];    
    if ( size(associated_gt) == 0 ) then
        fp_count = Nt_tracks; //in case of any potential gt association, all tracks are counted as false positive
    else
        for jj=1:Nt_tracks 
            if ( min( D2_chkd_rows(:,jj) ) > d2_th ) then
                unassociated_tk = [unassociated_tk tk_sel(jj*4-2)]; //save track id as unassociated
            else
                D2_chkd = [D2_chkd D2_chkd_rows(:,jj)];
                associated_tk = [associated_tk tk_sel(jj*4-2)]; //save track id as associated
            end
        end        
        fp_count = size(unassociated_tk,2); //false positiveS: a tracked target does not associate to any ground truth
    end
        
    //4. gt-tracks assignment. Munkres' Algorithm when more than 1 is involved  
    mapping_current = [];
    [Nt_gt Nt_tracks] = size(D2_chkd);
    if ( Nt_gt > 0 ) then //to avoid degenerate case where no potential matchings exist 
        if size(associated_gt) == 1 then //case with a single gt. Find the nearest track and assign to it
            mapping_current = [associated_gt(1)  associated_tk(find(~(D2_chkd(1,:)-min(D2_chkd(1,:))),1))];
            fp_count = fp_count + Nt_tracks-1; //all other tracks count as false positives
        else
            if size(associated_tk) == 1 then //case with a single tk. Find the nearest gt and assign to it
                mapping_current = [associated_gt(find(~(D2_chkd(:,1)-min(D2_chkd(:,1))),1)) associated_tk(1)];
                fn_count = fn_count + Nt_gt-1; //all other gt's count as false negatives
            else //general case. Munkres algorithm
                mapping_current = munkres(D2_chkd);    
            end             
        end
    end
    
    //5. Check if ID switches by comparing current and previous mappings
    if tt > 1 then
        // to do        
    end
    mapping_previous = mapping_current; 
   
    //6. DISPLAY   
    disp("New iteration--------");    
    mprintf('TS: %10.10f\n', tracks(tt)(1));
    disp('D2_chkd:'); disp(D2_chkd);    
    disp('fn_count:'); disp(fn_count);
    disp('fp_count:'); disp(fp_count);
    disp('mapping:');disp(mapping_current);
    
    //7. PLOT
    if plot_flag then
        if Nt_gt > 0 then
            for kk=1:Nt_gt
                plot(-gt_intp(tt)(kk*3+1),gt_intp(tt)(kk*3),plot_colors(pmodulo(gt_intp(tt)(kk*3-1)-1,5)+1)+'+');
            end
        end
        if Nt_tracks > 0 then
            for kk=1:Nt_tracks
                if tracks(tt)(kk*4-1) > 15 then //check status value: Visually confirmed or higher
                    plot(-tracks(tt)(kk*4+1),tracks(tt)(kk*4),plot_colors(pmodulo(tracks(tt)(kk*4-2)-1,5)+1)+'.');   
                end
            end    
        end
    end    
end





//COMPUTE DISTANCES -> MOTP
    
//CHECK MAPPING SWITCHES -> sw
    
//COUNT FALSE POSITIVES -> fp_count
    
//COUNT FALSE NEGATIVES -> fn_count
    
//COMPUTE MOTA = (sw,fp,m)

//close file
mclose(fd_gt);
mclose(fd_tracks);
