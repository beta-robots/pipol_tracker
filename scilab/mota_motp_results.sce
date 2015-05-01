
//20140925_FollowMe_tree_all_20hz_*.txt
motpA1 = [0.14 0.14 0.14 0.14 0.14 0.14 0.14 0.14 0.14 0.14 0.14 0.14];
motaA1 = [0.55 0.79 0.67 0.78 0.64 0.77 0.69 0.60 0.74 0.61 0.73 0.75]; 

//20140925_FollowMe_tree_LegCam_20hz_*.txt
motpA2 = [0.11 0.11 0.11 0.10 0.11 0.11 0.11 0.11 0.11 0.11];
motaA2 = [0.62 0.45 0.75 0.73 0.58 0.70 0.65 0.69 0.55 0.61];

//20140925_FollowMe_tree_LegKin_20hz_*.txt
motpA3 = [0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15 0.15];
motaA3 = [0.71 0.71 0.70 0.72 0.71 0.71 0.72 0.72 0.72 0.72];

//20140925_FollowMe_nn_all_20hz_*.txt
motpA4 = [0.15 0.14 0.14 0.15 0.15 0.14  0.15 0.14 0.15 0.15];
motaA4 = [0.32 0.41 0.38 0.69 0.10 -0.13 0.32 0.37 0.59 0.39];

//20140925_robotMoving_tree_all_20hz_*.txt
motpB1 = [0.14 0.15 0.15 0.15 0.15 0.16 0.15 0.15 0.15 0.15];
motaB1 = [0.30 0.23 0.30 0.32 0.39 0.26 0.33 0.49 0.12 0.31];

//20140925_robotMoving_tree_LegCam_20hz_*.txt
motpB2 = [0.16 0.16 0.21 0.16 0.16  0.19 0.18 0.19 0.20 0.16];
motaB2 = [0.26 0.20 0.17 0.24 0.26 -0.03 0.09 0.07 0.09 0.18];

//20140925_robotMoving_tree_LegKin_20hz_*.txt
motpB3 = [0.15 0.15 0.15 0.15 0.14 0.14 0.14 0.14 0.14 0.14];
motaB3 = [0.33 0.36 0.23 0.30 0.29 0.39 0.36 0.30 0.31 0.33];

//20140925_robotMoving_nn_all_20hz_*.txt
motpB4 = [0.16  0.15 0.15 0.15 0.16 0.15 0.15 0.15 0.15 0.15];
motaB4 = [-0.13 0.12 0.31 0.13 0.11 0.37 0.13 0.26 0.36 0.40];

//20140926_twoPeople_tree_all_20hz_*.txt
motpC1 = [0.16 0.16 0.16 0.16 0.16 0.16 0.16 0.16 0.15 0.16];
motaC1 = [0.67 0.66 0.72 0.68 0.70 0.66 0.71 0.52 0.70 0.56];

//20140926_twoPeople_tree_LegCam_20hz_*.txt
motpC2 = [0.15 0.14 0.14 0.14 0.14 0.14 0.13 0.14 0.14 0.14];
motaC2 = [0.36 0.41 0.44 0.39 0.19 0.37 0.29 0.36 0.42 0.008];

//20140926_twoPeople_tree_LegKin_20hz_*.txt
motpC3 = [0.14 0.16 0.16 0.16 0.16 0.15 0.16 0.15 0.16 0.16];
motaC3 = [0.26 0.65 0.70 0.63 0.71 0.67 0.57 0.68 0.50 0.73];

//20140926_twoPeople_nn_all_20hz_*.txt
motpC4 = [0.15 0.16 0.15 0.15 0.16 0.15 0.15 0.15 0.15 0.15];
motaC4 = [0.47 0.74 0.48 0.47 0.44 0.49 0.78 0.49 0.50 0.72];



disp("************** A: FollowMe **************");
disp("motpA1: "); disp(mean(motpA1));
disp("motaA1: "); disp(mean(motaA1));
disp("motpA2: "); disp(mean(motpA2));
disp("motaA2: "); disp(mean(motaA2));
disp("motpA3: "); disp(mean(motpA3));
disp("motaA3: "); disp(mean(motaA3));
disp("motpA4: "); disp(mean(motpA4));
disp("motaA4: "); disp(mean(motaA4));

disp("************** B: RobotMoving **************");
disp("motpB1: "); disp(mean(motpB1));
disp("motaB1: "); disp(mean(motaB1));
disp("motpB2: "); disp(mean(motpB2));
disp("motaB2: "); disp(mean(motaB2));
disp("motpB3: "); disp(mean(motpB3));
disp("motaB3: "); disp(mean(motaB3));
disp("motpB4: "); disp(mean(motpB4));
disp("motaB4: "); disp(mean(motaB4));

disp("************** C: TwoPeople **************");
disp("motpC1: "); disp(mean(motpC1));
disp("motaC1: "); disp(mean(motaC1));
disp("motpC2: "); disp(mean(motpC2));
disp("motaC2: "); disp(mean(motaC2));
disp("motpC3: "); disp(mean(motpC3));
disp("motaC3: "); disp(mean(motaC3));
disp("motpC4: "); disp(mean(motpC4));
disp("motaC4: "); disp(mean(motaC4));
