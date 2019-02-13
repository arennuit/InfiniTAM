% Import trajectories.
% viveTraj            = dlmread('/home/arennuit/DevRoot/src/InfiniTAM/Files/Out_noMocap/viveTraj.dat');
% camTraj_fromVive_qt = dlmread('/home/arennuit/DevRoot/src/InfiniTAM/Files/00_Out_douaiNoMocap/camTraj_fromVive.dat');
% camTraj              = dlmread('/home/arennuit/DevRoot/src/InfiniTAM/Files/12_Out_straightNoGeometry/trackingState.dat');
camTraj_01           = dlmread('/home/arennuit/DevRoot/src/InfiniTAM/Files/12_Out_straightNoGeometry/trackingState_01.dat');
camTraj_02           = dlmread('/home/arennuit/DevRoot/src/InfiniTAM/Files/12_Out_straightNoGeometry/trackingState_02.dat');
% camTraj_03           = dlmread('/home/arennuit/DevRoot/src/InfiniTAM/Files/12_Out_straightNoGeometry/trackingState_03.dat');
camTraj_04           = dlmread('/home/arennuit/DevRoot/src/InfiniTAM/Files/12_Out_straightNoGeometry/trackingState_04.dat');
camTraj_05           = dlmread('/home/arennuit/DevRoot/src/InfiniTAM/Files/12_Out_straightNoGeometry/trackingState_05.dat');
camTraj_06           = dlmread('/home/arennuit/DevRoot/src/InfiniTAM/Files/12_Out_straightNoGeometry/trackingState_06.dat');
camTraj_07           = dlmread('/home/arennuit/DevRoot/src/InfiniTAM/Files/12_Out_straightNoGeometry/trackingState_07.dat');
camTraj_08           = dlmread('/home/arennuit/DevRoot/src/InfiniTAM/Files/12_Out_straightNoGeometry/trackingState_08.dat');
camTraj_09           = dlmread('/home/arennuit/DevRoot/src/InfiniTAM/Files/12_Out_straightNoGeometry/trackingState_09.dat');
camTraj_10           = dlmread('/home/arennuit/DevRoot/src/InfiniTAM/Files/12_Out_straightNoGeometry/trackingState_10.dat');
viveTraj            = dlmread('/home/arennuit/DevRoot/src/InfiniTAM/Files/12_Out_straightNoGeometry/viveTraj.dat');
% camTraj_icpMocap    = dlmread('/home/arennuit/DevRoot/src/InfiniTAM/Files/Out/trackingState_mocap.dat');

% Convert mocap traj from frame [mocap0] into frame [cam0].
H_cam_vive = [  0, 0, -1, 0.00873;...
               -1, 0,  0, 0.0465 ;...
                0, 1,  0, 0.0415 ;...
                0, 0,  0, 1      ];
f_cam_vive = frame_from_mat4( H_cam_vive );
f_vive_cam = frame_inv( f_cam_vive );

camTraj_fromVive = zeros( size(viveTraj) );
for i=1:size( viveTraj, 1 )
    % The frames are stored in the file using convention [idx, x, y, z, qx, qy, qz, qw],
    % convert to [qw, qx, qy, qz, x, y, z] used in our matlab math library.
    frame = [viveTraj(i, 8), viveTraj(i, 5), viveTraj(i, 6), viveTraj(i, 7), viveTraj(i, 2), viveTraj(i, 3), viveTraj(i, 4)];
    
    frame_converted = frame_mul_frame( frame_mul_frame( f_vive_cam, frame ), f_cam_vive );
    
    % Convert back between conventions.
    camTraj_fromVive(i, :) = [ i, frame_converted(5:7), frame_converted(2:4), frame_converted(1) ];
end

% Display LINEAR trajectories.
minIdx = 1;
maxIdx = size(camTraj_fromVive, 1);
legendLabels = {};
plot3( camTraj_fromVive(minIdx:maxIdx, 2), camTraj_fromVive(minIdx:maxIdx, 3), camTraj_fromVive(minIdx:maxIdx, 4), 'k+-' ); legendLabels{end + 1} = 'cam from vive';
hold on;
% plot3( camTraj(minIdx:maxIdx, 2),    camTraj(minIdx:maxIdx, 3),    camTraj(minIdx:maxIdx, 4), 'go-' );    legendLabels{end + 1} = '------';
% plot3( camTraj_01(minIdx:maxIdx, 2), camTraj_01(minIdx:maxIdx, 3), camTraj_01(minIdx:maxIdx, 4), 'b+-' ); legendLabels{end + 1} = 'no precond, raw ICP, no SVD';
% plot3( camTraj_02(minIdx:maxIdx, 2), camTraj_02(minIdx:maxIdx, 3), camTraj_02(minIdx:maxIdx, 4), 'r+-' ); legendLabels{end + 1} = 'relative precond, ICP, no SVD';
% plot3( camTraj_03(minIdx:maxIdx, 2), camTraj_03(minIdx:maxIdx, 3), camTraj_03(minIdx:maxIdx, 4), 'r+-' ); legendLabels{end + 1} = '-----';
% plot3( camTraj_04(minIdx:maxIdx, 2), camTraj_04(minIdx:maxIdx, 3), camTraj_04(minIdx:maxIdx, 4), 'mo-' ); legendLabels{end + 1} = 'absolute precond, ICP, no SVD';
% plot3( camTraj_05(minIdx:maxIdx, 2), camTraj_05(minIdx:maxIdx, 3), camTraj_05(minIdx:maxIdx, 4), 'co-' ); legendLabels{end + 1} = 'absolute precond, ICP, relative SVD';
% plot3( camTraj_06(minIdx:maxIdx, 2), camTraj_06(minIdx:maxIdx, 3), camTraj_06(minIdx:maxIdx, 4), 'go-' ); legendLabels{end + 1} = 'absolute precond, ICP damped, no SVD';
% plot3( camTraj_07(minIdx:maxIdx, 2), camTraj_07(minIdx:maxIdx, 3), camTraj_07(minIdx:maxIdx, 4), 'g+-' ); legendLabels{end + 1} = 'absolute precond, ICP 1 level, relative SVD';
% plot3( camTraj_08(minIdx:maxIdx, 2), camTraj_08(minIdx:maxIdx, 3), camTraj_08(minIdx:maxIdx, 4), 'r+-' ); legendLabels{end + 1} = 'absolute precond, ICP 1 level, absolute SVD';
% plot3( camTraj_09(minIdx:maxIdx, 2), camTraj_09(minIdx:maxIdx, 3), camTraj_09(minIdx:maxIdx, 4), 'b+-' ); legendLabels{end + 1} = 'no precond, ICP 5 levels, absolute SVD';
plot3( camTraj_10(minIdx:maxIdx, 2), camTraj_10(minIdx:maxIdx, 3), camTraj_10(minIdx:maxIdx, 4), 'go-' ); legendLabels{end + 1} = 'no precond, ICP 5 levels, relative SVD';
quiver3( [0, 0.05], [0, 0], [0, 0], [0, 0.1], [0, 0], [0, 0], 'r' );
quiver3( [0, 0.05], [0, 0], [0, 0], [0, 0], [0, 0.1], [0, 0], 'g' );
quiver3( [0, 0.05], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0.1], 'b' );
text(-0.1, 0, 0, 'p_{cam (cam_0)}')
legend( legendLabels );
axis equal
ax = gca;

% minIdx_2 = 695;
% maxIdx_2 = 710; %size(camTraj_icpNoMocap, 1);
% plot3( camTraj_fromVive(minIdx_2:maxIdx_2, 2), camTraj_fromVive(minIdx_2:maxIdx_2, 3), camTraj_fromVive(minIdx_2:maxIdx_2, 4), 'k+-' );
% plot3( camTraj_4(minIdx_2:maxIdx_2, 2), camTraj_4(minIdx_2:maxIdx_2, 3), camTraj_4(minIdx_2:maxIdx_2, 4), 'm+-' );
% linkIdx_1 = 146;
% plot3([camTraj_4(linkIdx_1, 2), camTraj_fromVive(linkIdx_1, 2)], [camTraj_4(linkIdx_1, 3), camTraj_fromVive(linkIdx_1, 3)], [camTraj_4(linkIdx_1, 4), camTraj_fromVive(linkIdx_1, 4)],'r');
% plot3([camTraj_4(linkIdx_1, 2), camTraj_fromVive(linkIdx_1, 2)], [camTraj_4(linkIdx_1, 3), camTraj_fromVive(linkIdx_1, 3)], [camTraj_4(linkIdx_1, 4), camTraj_fromVive(linkIdx_1, 4)],'ro','MarkerSize',12, 'LineWidth', 3);
% linkIdx_2 = 702;
% plot3([camTraj_4(linkIdx_2, 2), camTraj_fromVive(linkIdx_2, 2)], [camTraj_4(linkIdx_2, 3), camTraj_fromVive(linkIdx_2, 3)], [camTraj_4(linkIdx_2, 4), camTraj_fromVive(linkIdx_2, 4)],'r');
% plot3([camTraj_4(linkIdx_2, 2), camTraj_fromVive(linkIdx_2, 2)], [camTraj_4(linkIdx_2, 3), camTraj_fromVive(linkIdx_2, 3)], [camTraj_4(linkIdx_2, 4), camTraj_fromVive(linkIdx_2, 4)],'ro','MarkerSize',12, 'LineWidth', 3);


% dataLink( camTraj_fromVive(:,2:4), camTraj_5(:,2:4), ax);