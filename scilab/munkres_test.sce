exec('/home/andreu/dev/ros_ws/src/pipol_tracker/scilab/munkres.sci');

AA = [10 19 8 15; 10 18 7 17; 13 16 9 14; 12 19 8 18; 14 17 10 19];
mapping = munkres(AA);
disp(mapping);
