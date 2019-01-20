


% Based on transformation between /iiwa_base_link and /target_frame from tf
% as on error IK problem is solved and corresponding joint angles
% changes is sent to arm_state topic

function [control_js_msg, tf_out_msg,x_err, y_err, z_err, a_err, b_err, g_err, state] = fcn(js_cur_msg, blank_js_msg, T, tf_blank_msg)

persistent seq;
if isempty(seq)
    seq = 0;
end

persistent w;
if isempty(w)
    w = 0;
end

% matlab shitty way
persistent th_sfm;
if isempty(th_sfm)
    th_sfm = [0;0;0;0;0;0;0];
end

state = 0;

% Message template
% ----------------------------------------------
str_array_of_jnames = {'iiwa_j1', 'iiwa_j2', 'iiwa_j3', 'iiwa_j4', 'iiwa_j5', 'iiwa_j6', 'iiwa_j7'};
control_js_msg = blank_js_msg;

th_num = length(str_array_of_jnames);
control_js_msg.Name_SL_Info.CurrentLength = uint32(th_num);
control_js_msg.Position_SL_Info.CurrentLength = uint32(th_num);

for idx=1:th_num
    str = str_array_of_jnames{idx};
    str_length = length(str);
    control_js_msg.Name(idx).Data(1:str_length) = uint8(str);
    control_js_msg.Name(idx).Data_SL_Info.CurrentLength = uint32(str_length);
end

% Number of links
% ----------------------------------------------
n = 7;

% Current configuration
% ----------------------------------------------
th_cur = zeros(n,1);
for i = 1:n
    th_cur(i) = js_cur_msg.Position(i);
end
HHH = eye(4,4);
H0RR = eye(4,4);
H0iR_fr = zeros(4,4,7);
H0i_fr = zeros(4,4,7);
for k=1:7
    H0iR_fr(:,:,k) = HHH;
    H0i_fr(:,:,k) = HHH;
end;

% checking for correctness of the assigned target
% ----------------------------------------------
if T(8) == true

    % DH parameters
    % -------------------------------------------
    th_DH  = [0;         0;     0;      0;     0;      0;      0];
    d_DH   = [0.36;      0;  0.42;      0;   0.4;      0;  0.126];
    a_DH   = [0;         0;     0;      0;     0;      0;      0];
    al_DH  = [pi/2;  -pi/2;  pi/2;  -pi/2;  pi/2;  -pi/2;      0];
    DH = [th_DH, d_DH, a_DH, al_DH];

    % Target position and orientation
    % -------------------------------------------
    P_tar = [T(1); T(2); T(3);];
    x_err=T(1);
    y_err=T(2);
    z_err=T(3);
    R_tar = quat2rotm([T(4), T(5), T(6), T(7)]);
    H_tar = [[R_tar(1,:), P_tar(1)];...
             [R_tar(2,:), P_tar(2)];...
             [R_tar(3,:), P_tar(3)];...
             [0,  0,   0,        1]];

    % mode choosing (true => trajectory mode)
    % -------------------------------------------
    if T(9) == true
        state = 1;

        % Trajectory generation
        % ---------------------------------------
        for i = 1:n
            control_js_msg.Position(i) = double(th_cur(i));
        end
    else
        if w == 2
        % IK solver call
        % ---------------------------------------
        state = 2;
        [th_tar, H0iR_fr, H0i_fr] = FABRIK_CS(H_tar, th_cur, DH); %%% !!!!!!!
        th_sfm = th_tar;
        %th_cur = [0;0;0;0;0;0;0];
        %th_tar = FABRIK_CS(H_tar, th_cur, DH);

        for i = 1:n
            control_js_msg.Position(i) = double(th_tar(i));
        end
        w = 0;
        else
            state = 4;
            th_tar = th_sfm;
            for i = 1:n
                control_js_msg.Position(i) = double(th_tar(i));
            end
            w = w + 1;
        end;
    end
else
    state = 3;
    for i = 1:n
        control_js_msg.Position(i) = double(th_cur(i));
    end
    x_err=0;
    y_err=0;
    z_err=0;
end

a_err=0;
b_err=0;
g_err=0;

tf_out_msg = tf_blank_msg;
tf_out_msg.Transforms_SL_Info.CurrentLength = uint32(14);

pr_frame_id = 'base_link';
pr_n_l = length(pr_frame_id);

ch_frame_id = 'fr6';
ch_n_l = length(ch_frame_id);

    H6 = H0iR_fr(:,:,6);% eye(4,4);
        %H6(1:3,1:3) = H0iR_fr(1:3,1:3,6).';
        %H6(1:3,4) = -H6(1:3,1:3)*H0iR_fr(1:3,4,6);
{
        Q = tform2quat(H6);
        tf_out_msg.Transforms(1).ChildFrameId(1:ch_n_l) = uint8(ch_frame_id);
        tf_out_msg.Transforms(1).ChildFrameId_SL_Info.CurrentLength = uint32(ch_n_l);
        tf_out_msg.Transforms(1).Header.FrameId(1:pr_n_l) = uint8(pr_frame_id);
        tf_out_msg.Transforms(1).Header.FrameId_SL_Info.CurrentLength = uint32(pr_n_l);
        tf_out_msg.Transforms(1).Transform.Translation.X = H6(1,4);
        tf_out_msg.Transforms(1).Transform.Translation.Y = H6(2,4);
        tf_out_msg.Transforms(1).Transform.Translation.Z = H6(3,4);
        tf_out_msg.Transforms(1).Transform.Rotation.W = Q(1);
        tf_out_msg.Transforms(1).Transform.Rotation.X = Q(2);
        tf_out_msg.Transforms(1).Transform.Rotation.Y = Q(3);
        tf_out_msg.Transforms(1).Transform.Rotation.Z = Q(4);



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ch_frame_id = 'fr5';
    ch_n_l = length(ch_frame_id);

        H5 = H0iR_fr(:,:,5);%eye(4,4);
        %H5(1:3,1:3) = H0iR_fr(1:3,1:3,5).';
        %H5(1:3,4) = -H5(1:3,1:3)*H0iR_fr(1:3,4,5);

        Q = tform2quat(H5);
        tf_out_msg.Transforms(2).ChildFrameId(1:ch_n_l) = uint8(ch_frame_id);
        tf_out_msg.Transforms(2).ChildFrameId_SL_Info.CurrentLength = uint32(ch_n_l);
        tf_out_msg.Transforms(2).Header.FrameId(1:pr_n_l) = uint8(pr_frame_id);
        tf_out_msg.Transforms(2).Header.FrameId_SL_Info.CurrentLength = uint32(pr_n_l);
        tf_out_msg.Transforms(2).Transform.Translation.X = H5(1,4);
        tf_out_msg.Transforms(2).Transform.Translation.Y = H5(2,4);
        tf_out_msg.Transforms(2).Transform.Translation.Z = H5(3,4);
        tf_out_msg.Transforms(2).Transform.Rotation.W = Q(1);
        tf_out_msg.Transforms(2).Transform.Rotation.X = Q(2);
        tf_out_msg.Transforms(2).Transform.Rotation.Y = Q(3);
        tf_out_msg.Transforms(2).Transform.Rotation.Z = Q(4);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ch_frame_id = 'fr4';
    ch_n_l = length(ch_frame_id);

        H4 = H0iR_fr(:,:,4);%eye(4,4);
        %H5(1:3,1:3) = H0iR_fr(1:3,1:3,5).';
        %H5(1:3,4) = -H5(1:3,1:3)*H0iR_fr(1:3,4,5);

        Q = tform2quat(H4);
        tf_out_msg.Transforms(3).ChildFrameId(1:ch_n_l) = uint8(ch_frame_id);
        tf_out_msg.Transforms(3).ChildFrameId_SL_Info.CurrentLength = uint32(ch_n_l);
        tf_out_msg.Transforms(3).Header.FrameId(1:pr_n_l) = uint8(pr_frame_id);
        tf_out_msg.Transforms(3).Header.FrameId_SL_Info.CurrentLength = uint32(pr_n_l);
        tf_out_msg.Transforms(3).Transform.Translation.X = H4(1,4);
        tf_out_msg.Transforms(3).Transform.Translation.Y = H4(2,4);
        tf_out_msg.Transforms(3).Transform.Translation.Z = H4(3,4);
        tf_out_msg.Transforms(3).Transform.Rotation.W = Q(1);
        tf_out_msg.Transforms(3).Transform.Rotation.X = Q(2);
        tf_out_msg.Transforms(3).Transform.Rotation.Y = Q(3);
        tf_out_msg.Transforms(3).Transform.Rotation.Z = Q(4);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ch_frame_id = 'fr3';
    ch_n_l = length(ch_frame_id);

        H3 = H0iR_fr(:,:,3);%eye(4,4);
        %H5(1:3,1:3) = H0iR_fr(1:3,1:3,5).';
        %H5(1:3,4) = -H5(1:3,1:3)*H0iR_fr(1:3,4,5);

        Q = tform2quat(H3);
        tf_out_msg.Transforms(4).ChildFrameId(1:ch_n_l) = uint8(ch_frame_id);
        tf_out_msg.Transforms(4).ChildFrameId_SL_Info.CurrentLength = uint32(ch_n_l);
        tf_out_msg.Transforms(4).Header.FrameId(1:pr_n_l) = uint8(pr_frame_id);
        tf_out_msg.Transforms(4).Header.FrameId_SL_Info.CurrentLength = uint32(pr_n_l);
        tf_out_msg.Transforms(4).Transform.Translation.X = H3(1,4);
        tf_out_msg.Transforms(4).Transform.Translation.Y = H3(2,4);
        tf_out_msg.Transforms(4).Transform.Translation.Z = H3(3,4);
        tf_out_msg.Transforms(4).Transform.Rotation.W = Q(1);
        tf_out_msg.Transforms(4).Transform.Rotation.X = Q(2);
        tf_out_msg.Transforms(4).Transform.Rotation.Y = Q(3);
        tf_out_msg.Transforms(4).Transform.Rotation.Z = Q(4);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ch_frame_id = 'fr2';
    ch_n_l = length(ch_frame_id);

        H2 = H0iR_fr(:,:,2);%eye(4,4);
        %H5(1:3,1:3) = H0iR_fr(1:3,1:3,5).';
        %H5(1:3,4) = -H5(1:3,1:3)*H0iR_fr(1:3,4,5);

        Q = tform2quat(H2);
        tf_out_msg.Transforms(5).ChildFrameId(1:ch_n_l) = uint8(ch_frame_id);
        tf_out_msg.Transforms(5).ChildFrameId_SL_Info.CurrentLength = uint32(ch_n_l);
        tf_out_msg.Transforms(5).Header.FrameId(1:pr_n_l) = uint8(pr_frame_id);
        tf_out_msg.Transforms(5).Header.FrameId_SL_Info.CurrentLength = uint32(pr_n_l);
        tf_out_msg.Transforms(5).Transform.Translation.X = H2(1,4);
        tf_out_msg.Transforms(5).Transform.Translation.Y = H2(2,4);
        tf_out_msg.Transforms(5).Transform.Translation.Z = H2(3,4);
        tf_out_msg.Transforms(5).Transform.Rotation.W = Q(1);
        tf_out_msg.Transforms(5).Transform.Rotation.X = Q(2);
        tf_out_msg.Transforms(5).Transform.Rotation.Y = Q(3);
        tf_out_msg.Transforms(5).Transform.Rotation.Z = Q(4);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ch_frame_id = 'fr1';
    ch_n_l = length(ch_frame_id);

        H1 = H0iR_fr(:,:,1);%eye(4,4);
        %H5(1:3,1:3) = H0iR_fr(1:3,1:3,5).';
        %H5(1:3,4) = -H5(1:3,1:3)*H0iR_fr(1:3,4,5);

        Q = tform2quat(H1);
        tf_out_msg.Transforms(6).ChildFrameId(1:ch_n_l) = uint8(ch_frame_id);
        tf_out_msg.Transforms(6).ChildFrameId_SL_Info.CurrentLength = uint32(ch_n_l);
        tf_out_msg.Transforms(6).Header.FrameId(1:pr_n_l) = uint8(pr_frame_id);
        tf_out_msg.Transforms(6).Header.FrameId_SL_Info.CurrentLength = uint32(pr_n_l);
        tf_out_msg.Transforms(6).Transform.Translation.X = H1(1,4);
        tf_out_msg.Transforms(6).Transform.Translation.Y = H1(2,4);
        tf_out_msg.Transforms(6).Transform.Translation.Z = H1(3,4);
        tf_out_msg.Transforms(6).Transform.Rotation.W = Q(1);
        tf_out_msg.Transforms(6).Transform.Rotation.X = Q(2);
        tf_out_msg.Transforms(6).Transform.Rotation.Y = Q(3);
        tf_out_msg.Transforms(6).Transform.Rotation.Z = Q(4);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ch_frame_id = 'fr0';
    ch_n_l = length(ch_frame_id);

        H0 = H0RR;%eye(4,4);
        %H5(1:3,1:3) = H0iR_fr(1:3,1:3,5).';
        %H5(1:3,4) = -H5(1:3,1:3)*H0iR_fr(1:3,4,5);

        Q = tform2quat(H0);
        tf_out_msg.Transforms(7).ChildFrameId(1:ch_n_l) = uint8(ch_frame_id);
        tf_out_msg.Transforms(7).ChildFrameId_SL_Info.CurrentLength = uint32(ch_n_l);
        tf_out_msg.Transforms(7).Header.FrameId(1:pr_n_l) = uint8(pr_frame_id);
        tf_out_msg.Transforms(7).Header.FrameId_SL_Info.CurrentLength = uint32(pr_n_l);
        tf_out_msg.Transforms(7).Transform.Translation.X = H0(1,4);
        tf_out_msg.Transforms(7).Transform.Translation.Y = H0(2,4);
        tf_out_msg.Transforms(7).Transform.Translation.Z = H0(3,4);
        tf_out_msg.Transforms(7).Transform.Rotation.W = Q(1);
        tf_out_msg.Transforms(7).Transform.Rotation.X = Q(2);
        tf_out_msg.Transforms(7).Transform.Rotation.Y = Q(3);
        tf_out_msg.Transforms(7).Transform.Rotation.Z = Q(4);


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ch_frame_id = 'fi6';
    ch_n_l = length(ch_frame_id);

        H6 = H0i_fr(:,:,6);% eye(4,4);
        %H6(1:3,1:3) = H0iR_fr(1:3,1:3,6).';
        %H6(1:3,4) = -H6(1:3,1:3)*H0iR_fr(1:3,4,6);

        Q = tform2quat(H6);
        tf_out_msg.Transforms(8).ChildFrameId(1:ch_n_l) = uint8(ch_frame_id);
        tf_out_msg.Transforms(8).ChildFrameId_SL_Info.CurrentLength = uint32(ch_n_l);
        tf_out_msg.Transforms(8).Header.FrameId(1:pr_n_l) = uint8(pr_frame_id);
        tf_out_msg.Transforms(8).Header.FrameId_SL_Info.CurrentLength = uint32(pr_n_l);
        tf_out_msg.Transforms(8).Transform.Translation.X = H6(1,4);
        tf_out_msg.Transforms(8).Transform.Translation.Y = H6(2,4);
        tf_out_msg.Transforms(8).Transform.Translation.Z = H6(3,4);
        tf_out_msg.Transforms(8).Transform.Rotation.W = Q(1);
        tf_out_msg.Transforms(8).Transform.Rotation.X = Q(2);
        tf_out_msg.Transforms(8).Transform.Rotation.Y = Q(3);
        tf_out_msg.Transforms(8).Transform.Rotation.Z = Q(4);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ch_frame_id = 'fi5';
    ch_n_l = length(ch_frame_id);

        H5 = H0i_fr(:,:,5);%eye(4,4);
        %H5(1:3,1:3) = H0iR_fr(1:3,1:3,5).';
        %H5(1:3,4) = -H5(1:3,1:3)*H0iR_fr(1:3,4,5);

        Q = tform2quat(H5);
        tf_out_msg.Transforms(9).ChildFrameId(1:ch_n_l) = uint8(ch_frame_id);
        tf_out_msg.Transforms(9).ChildFrameId_SL_Info.CurrentLength = uint32(ch_n_l);
        tf_out_msg.Transforms(9).Header.FrameId(1:pr_n_l) = uint8(pr_frame_id);
        tf_out_msg.Transforms(9).Header.FrameId_SL_Info.CurrentLength = uint32(pr_n_l);
        tf_out_msg.Transforms(9).Transform.Translation.X = H5(1,4);
        tf_out_msg.Transforms(9).Transform.Translation.Y = H5(2,4);
        tf_out_msg.Transforms(9).Transform.Translation.Z = H5(3,4);
        tf_out_msg.Transforms(9).Transform.Rotation.W = Q(1);
        tf_out_msg.Transforms(9).Transform.Rotation.X = Q(2);
        tf_out_msg.Transforms(9).Transform.Rotation.Y = Q(3);
        tf_out_msg.Transforms(9).Transform.Rotation.Z = Q(4);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ch_frame_id = 'fi4';
    ch_n_l = length(ch_frame_id);

        H4 = H0i_fr(:,:,4);%eye(4,4);
        %H5(1:3,1:3) = H0iR_fr(1:3,1:3,5).';
        %H5(1:3,4) = -H5(1:3,1:3)*H0iR_fr(1:3,4,5);

        Q = tform2quat(H4);
        tf_out_msg.Transforms(10).ChildFrameId(1:ch_n_l) = uint8(ch_frame_id);
        tf_out_msg.Transforms(10).ChildFrameId_SL_Info.CurrentLength = uint32(ch_n_l);
        tf_out_msg.Transforms(10).Header.FrameId(1:pr_n_l) = uint8(pr_frame_id);
        tf_out_msg.Transforms(10).Header.FrameId_SL_Info.CurrentLength = uint32(pr_n_l);
        tf_out_msg.Transforms(10).Transform.Translation.X = H4(1,4);
        tf_out_msg.Transforms(10).Transform.Translation.Y = H4(2,4);
        tf_out_msg.Transforms(10).Transform.Translation.Z = H4(3,4);
        tf_out_msg.Transforms(10).Transform.Rotation.W = Q(1);
        tf_out_msg.Transforms(10).Transform.Rotation.X = Q(2);
        tf_out_msg.Transforms(10).Transform.Rotation.Y = Q(3);
        tf_out_msg.Transforms(10).Transform.Rotation.Z = Q(4);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ch_frame_id = 'fi3';
    ch_n_l = length(ch_frame_id);

        H3 = H0i_fr(:,:,3);%eye(4,4);
        %H5(1:3,1:3) = H0iR_fr(1:3,1:3,5).';
        %H5(1:3,4) = -H5(1:3,1:3)*H0iR_fr(1:3,4,5);

        Q = tform2quat(H3);
        tf_out_msg.Transforms(11).ChildFrameId(1:ch_n_l) = uint8(ch_frame_id);
        tf_out_msg.Transforms(11).ChildFrameId_SL_Info.CurrentLength = uint32(ch_n_l);
        tf_out_msg.Transforms(11).Header.FrameId(1:pr_n_l) = uint8(pr_frame_id);
        tf_out_msg.Transforms(11).Header.FrameId_SL_Info.CurrentLength = uint32(pr_n_l);
        tf_out_msg.Transforms(11).Transform.Translation.X = H3(1,4);
        tf_out_msg.Transforms(11).Transform.Translation.Y = H3(2,4);
        tf_out_msg.Transforms(11).Transform.Translation.Z = H3(3,4);
        tf_out_msg.Transforms(11).Transform.Rotation.W = Q(1);
        tf_out_msg.Transforms(11).Transform.Rotation.X = Q(2);
        tf_out_msg.Transforms(11).Transform.Rotation.Y = Q(3);
        tf_out_msg.Transforms(11).Transform.Rotation.Z = Q(4);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ch_frame_id = 'fi2';
    ch_n_l = length(ch_frame_id);

        H2 = H0i_fr(:,:,2);%eye(4,4);
        %H5(1:3,1:3) = H0iR_fr(1:3,1:3,5).';
        %H5(1:3,4) = -H5(1:3,1:3)*H0iR_fr(1:3,4,5);

        Q = tform2quat(H2);
        tf_out_msg.Transforms(12).ChildFrameId(1:ch_n_l) = uint8(ch_frame_id);
        tf_out_msg.Transforms(12).ChildFrameId_SL_Info.CurrentLength = uint32(ch_n_l);
        tf_out_msg.Transforms(12).Header.FrameId(1:pr_n_l) = uint8(pr_frame_id);
        tf_out_msg.Transforms(12).Header.FrameId_SL_Info.CurrentLength = uint32(pr_n_l);
        tf_out_msg.Transforms(12).Transform.Translation.X = H2(1,4);
        tf_out_msg.Transforms(12).Transform.Translation.Y = H2(2,4);
        tf_out_msg.Transforms(12).Transform.Translation.Z = H2(3,4);
        tf_out_msg.Transforms(12).Transform.Rotation.W = Q(1);
        tf_out_msg.Transforms(12).Transform.Rotation.X = Q(2);
        tf_out_msg.Transforms(12).Transform.Rotation.Y = Q(3);
        tf_out_msg.Transforms(12).Transform.Rotation.Z = Q(4);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ch_frame_id = 'fi1';
    ch_n_l = length(ch_frame_id);

        H1 = H0i_fr(:,:,1);%eye(4,4);
        %H5(1:3,1:3) = H0iR_fr(1:3,1:3,5).';
        %H5(1:3,4) = -H5(1:3,1:3)*H0iR_fr(1:3,4,5);

        Q = tform2quat(H1);
        tf_out_msg.Transforms(13).ChildFrameId(1:ch_n_l) = uint8(ch_frame_id);
        tf_out_msg.Transforms(13).ChildFrameId_SL_Info.CurrentLength = uint32(ch_n_l);
        tf_out_msg.Transforms(13).Header.FrameId(1:pr_n_l) = uint8(pr_frame_id);
        tf_out_msg.Transforms(13).Header.FrameId_SL_Info.CurrentLength = uint32(pr_n_l);
        tf_out_msg.Transforms(13).Transform.Translation.X = H1(1,4);
        tf_out_msg.Transforms(13).Transform.Translation.Y = H1(2,4);
        tf_out_msg.Transforms(13).Transform.Translation.Z = H1(3,4);
        tf_out_msg.Transforms(13).Transform.Rotation.W = Q(1);
        tf_out_msg.Transforms(13).Transform.Rotation.X = Q(2);
        tf_out_msg.Transforms(13).Transform.Rotation.Y = Q(3);
        tf_out_msg.Transforms(13).Transform.Rotation.Z = Q(4);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ch_frame_id = 'fi7';
    ch_n_l = length(ch_frame_id);

        H0 = H0i_fr(:,:,7);%eye(4,4);
        %H5(1:3,1:3) = H0iR_fr(1:3,1:3,5).';
        %H5(1:3,4) = -H5(1:3,1:3)*H0iR_fr(1:3,4,5);

        Q = tform2quat(H0);
        tf_out_msg.Transforms(14).ChildFrameId(1:ch_n_l) = uint8(ch_frame_id);
        tf_out_msg.Transforms(14).ChildFrameId_SL_Info.CurrentLength = uint32(ch_n_l);
        tf_out_msg.Transforms(14).Header.FrameId(1:pr_n_l) = uint8(pr_frame_id);
        tf_out_msg.Transforms(14).Header.FrameId_SL_Info.CurrentLength = uint32(pr_n_l);
        tf_out_msg.Transforms(14).Transform.Translation.X = H0(1,4);
        tf_out_msg.Transforms(14).Transform.Translation.Y = H0(2,4);
        tf_out_msg.Transforms(14).Transform.Translation.Z = H0(3,4);
        tf_out_msg.Transforms(14).Transform.Rotation.W = Q(1);
        tf_out_msg.Transforms(14).Transform.Rotation.X = Q(2);
        tf_out_msg.Transforms(14).Transform.Rotation.Y = Q(3);
        tf_out_msg.Transforms(14).Transform.Rotation.Z = Q(4);
    }
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

}

%tf_out_msg.Transforms(1).Header.Seq = uint32(seq);
control_js_msg.Header.Seq = uint32(seq);
seq = seq + 1;
end


function [th_tar, H0iF, H0i] = FABRIK_CS(H0t, th_cur, DH, th_lim, orientation)

% default values
% ----------------------------------------------
if nargin < 5
  orientation = true;
end;

% accuracy of IK solver
% ----------------------------------------------
pos_acc = 0.001;
ang_acc = 0.01/180*pi;

% Number of links
% ----------------------------------------------
n = size(DH, 1);
th_tar = zeros(n,1);

% RDH parameters (Reverse Denavit Hartenberg)
% -------------------------------------------
th_RDH  = flipud(-DH(:,1));
d_RDH   = flipud(-DH(:,2));
a_RDH   = flipud(-DH(:,3));
al_RDH  = flipud(-DH(:,4));
RDH = [th_RDH, d_RDH, a_RDH, al_RDH];

% temprorary tables for the angle corrections and the target frames for
% the corresponding joints

F_fr = [4;4;2;2;0;0;0];
B_fr = [3;3;5;5;7;7;7];

F_jc = [0;  pi/2; 0;  pi/2; 0;  pi/2; 0];
B_jc = [0; -pi/2; 0; -pi/2; 0; -pi/2; 0];

% transformations and corresponding poses of frames
% -------------------------------------------
H   = zeros(4, 4, n);
H0i = zeros(4, 4, n);

for i = 1:n
    H(:,:,i) = [cos(th_cur(i)+DH(i,1)),  -sin(th_cur(i)+DH(i,1))*cos(DH(i,4)),  sin(th_cur(i)+DH(i,1))*sin(DH(i,4)),  DH(i,3)*cos(th_cur(i)+DH(i,1)); ...
                sin(th_cur(i)+DH(i,1)),   cos(th_cur(i)+DH(i,1))*cos(DH(i,4)), -cos(th_cur(i)+DH(i,1))*sin(DH(i,4)),  DH(i,3)*sin(th_cur(i)+DH(i,1)); ...
                                     0,                          sin(DH(i,4)),                         cos(DH(i,4)),                         DH(i,2); ...
                                     0,                                     0,                                    0,                               1  ];
    H0i(:,:,i) = eye(4);
    for j = 1:i
        H0i(:,:,i) = H0i(:,:,i)*H(:,:,j);
    end;
end;

% reverse poses
H0iF = H0i;

H0iF(:,:,n) = H0t;
%while 1
    % Forward reaching
    % -------------------------------------------
    for i=n:-1:1
        k = n-i+1;
        H_alad = [1,               0,                0,                 RDH(k,3); ...
                  0,   cos(RDH(k,4)),   -sin(RDH(k,4)),  -RDH(k,2)*sin(RDH(k,4)); ...
                  0,   sin(RDH(k,4)),    cos(RDH(k,4)),   RDH(k,2)*cos(RDH(k,4)); ...
                  0,               0,                0,                        1  ];
        H0iFd = H0iF(:,:,i)*H_alad;
        RiFd0 = H0iFd(1:3,1:3).';
        PiFd0 = -RiFd0*H0iFd(1:3,4);
        HiFd0 = eye(4,4);
        HiFd0(1:3,1:3) = RiFd0;
        HiFd0(1:3,4) = PiFd0;
        if F_fr(k) == 0
            HiFim1 = HiFd0;
        else
            HiFim1 = HiFd0 * H0i(:,:,F_fr(k));
        end;
        if (HiFim1(1,4) == 0) && (HiFim1(2,4) == 0)
            th_tar(i) = th_cur(i);
        else
            th_tar(i) = atan2(HiFim1(2,4),HiFim1(1,4)) + F_jc(k);
        end;

        if i-1~= 0
            H_th = [cos(th_tar(i)),   -sin(th_tar(i)),   0,  0; ...
                    sin(th_tar(i)),    cos(th_tar(i)),   0,  0; ...
                                 0,                 0,   1,  0; ...
                                 0,                 0,   0,  1  ];
            H0iF(:,:,i-1) = H0iFd*H_th;
        end;
    end;

    % Backward reaching
    % -------------------------------------------
    H0it = H0i;
    for i=1:n
        if i == 1
            Hitip1R = H0iF(:,:,B_fr(i));
        else
            Rit0 = H0it(1:3,1:3,i-1).';
            Pit0 = -Rit0*H0it(1:3,4,i-1);
            Hit0 = eye(4,4);
            Hit0(1:3,1:3) = Rit0;
            Hit0(1:3,4) = Pit0;
            Hitip1R = Hit0*H0iF(:,:,B_fr(i));
        end;

        if (Hitip1R(1,4) == 0) && (Hitip1R(2,4) == 0)
            th_tar(i) = th_cur(i);
        elseif (i == n) && (abs(Hitip1R(1,4)) <= 0.001) && (abs(Hitip1R(2,4)) <= 0.001)
            eul = rotm2eul(Hitip1R(1:3,1:3));
            th_tar(i) = eul(1);
        else
            th_tar(i) = atan2(Hitip1R(2,4),Hitip1R(1,4))+B_jc(i);
        end;

        Him1tit = [cos(th_tar(i)+DH(i,1)),  -sin(th_tar(i)+DH(i,1))*cos(DH(i,4)),  sin(th_tar(i)+DH(i,1))*sin(DH(i,4)),  DH(i,3)*cos(th_tar(i)+DH(i,1)); ...
                   sin(th_tar(i)+DH(i,1)),   cos(th_tar(i)+DH(i,1))*cos(DH(i,4)), -cos(th_tar(i)+DH(i,1))*sin(DH(i,4)),  DH(i,3)*sin(th_tar(i)+DH(i,1)); ...
                                      0,                            sin(DH(i,4)),                         cos(DH(i,4)),                        DH(i,2) ; ...
                                      0,                                     0,                                    0,                               1    ];
        if i==1
            H0it(:,:,i) = Him1tit;
        else
            H0it(:,:,i) = H0it(:,:,i-1)*Him1tit;
        end;
    end;
%end;

end





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% traj_mode = tr_mode;
% out_of_ = out_of_trj;
%
% if (out_of_ == true) % another wired shit
%     on_traj = false;
% end
%
%
% H = [cos(th7)*(sin(th6)*(sin(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) - cos(th1)*cos(th4)*sin(th2)) - cos(th6)*(cos(th5)*(cos(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + cos(th1)*sin(th2)*sin(th4)) + sin(th5)*(cos(th3)*sin(th1) + cos(th1)*cos(th2)*sin(th3)))) + sin(th7)*(sin(th5)*(cos(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + cos(th1)*sin(th2)*sin(th4)) - cos(th5)*(cos(th3)*sin(th1) + cos(th1)*cos(th2)*sin(th3))), cos(th7)*(sin(th5)*(cos(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + cos(th1)*sin(th2)*sin(th4)) - cos(th5)*(cos(th3)*sin(th1) + cos(th1)*cos(th2)*sin(th3))) - sin(th7)*(sin(th6)*(sin(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) - cos(th1)*cos(th4)*sin(th2)) - cos(th6)*(cos(th5)*(cos(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + cos(th1)*sin(th2)*sin(th4)) + sin(th5)*(cos(th3)*sin(th1) + cos(th1)*cos(th2)*sin(th3)))),   cos(th6)*(sin(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) - cos(th1)*cos(th4)*sin(th2)) + sin(th6)*(cos(th5)*(cos(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + cos(th1)*sin(th2)*sin(th4)) + sin(th5)*(cos(th3)*sin(th1) + cos(th1)*cos(th2)*sin(th3))),   d67*(cos(th6)*(sin(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) - cos(th1)*cos(th4)*sin(th2)) + sin(th6)*(cos(th5)*(cos(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + cos(th1)*sin(th2)*sin(th4)) + sin(th5)*(cos(th3)*sin(th1) + cos(th1)*cos(th2)*sin(th3)))) + d45*(sin(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) - cos(th1)*cos(th4)*sin(th2)) - d23*cos(th1)*sin(th2);...
%      -cos(th7)*(sin(th6)*(sin(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) + cos(th4)*sin(th1)*sin(th2)) - cos(th6)*(cos(th5)*(cos(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) - sin(th1)*sin(th2)*sin(th4)) + sin(th5)*(cos(th1)*cos(th3) - cos(th2)*sin(th1)*sin(th3)))) - sin(th7)*(sin(th5)*(cos(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) - sin(th1)*sin(th2)*sin(th4)) - cos(th5)*(cos(th1)*cos(th3) - cos(th2)*sin(th1)*sin(th3))), sin(th7)*(sin(th6)*(sin(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) + cos(th4)*sin(th1)*sin(th2)) - cos(th6)*(cos(th5)*(cos(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) - sin(th1)*sin(th2)*sin(th4)) + sin(th5)*(cos(th1)*cos(th3) - cos(th2)*sin(th1)*sin(th3)))) - cos(th7)*(sin(th5)*(cos(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) - sin(th1)*sin(th2)*sin(th4)) - cos(th5)*(cos(th1)*cos(th3) - cos(th2)*sin(th1)*sin(th3))), - cos(th6)*(sin(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) + cos(th4)*sin(th1)*sin(th2)) - sin(th6)*(cos(th5)*(cos(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) - sin(th1)*sin(th2)*sin(th4)) + sin(th5)*(cos(th1)*cos(th3) - cos(th2)*sin(th1)*sin(th3))), - d67*(cos(th6)*(sin(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) + cos(th4)*sin(th1)*sin(th2)) + sin(th6)*(cos(th5)*(cos(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) - sin(th1)*sin(th2)*sin(th4)) + sin(th5)*(cos(th1)*cos(th3) - cos(th2)*sin(th1)*sin(th3)))) - d45*(sin(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) + cos(th4)*sin(th1)*sin(th2)) - d23*sin(th1)*sin(th2);...
%      cos(th7)*(cos(th6)*(cos(th5)*(cos(th2)*sin(th4) + cos(th3)*cos(th4)*sin(th2)) - sin(th2)*sin(th3)*sin(th5)) + sin(th6)*(cos(th2)*cos(th4) - cos(th3)*sin(th2)*sin(th4))) - sin(th7)*(sin(th5)*(cos(th2)*sin(th4) + cos(th3)*cos(th4)*sin(th2)) + cos(th5)*sin(th2)*sin(th3)),                                                                                                                                                                                     - cos(th7)*(sin(th5)*(cos(th2)*sin(th4) + cos(th3)*cos(th4)*sin(th2)) + cos(th5)*sin(th2)*sin(th3)) - sin(th7)*(cos(th6)*(cos(th5)*(cos(th2)*sin(th4) + cos(th3)*cos(th4)*sin(th2)) - sin(th2)*sin(th3)*sin(th5)) + sin(th6)*(cos(th2)*cos(th4) - cos(th3)*sin(th2)*sin(th4))),                                                                                                                  cos(th6)*(cos(th2)*cos(th4) - cos(th3)*sin(th2)*sin(th4)) - sin(th6)*(cos(th5)*(cos(th2)*sin(th4) + cos(th3)*cos(th4)*sin(th2)) - sin(th2)*sin(th3)*sin(th5)),                                                                                                                                                             d01 + d45*(cos(th2)*cos(th4) - cos(th3)*sin(th2)*sin(th4)) - d67*(sin(th6)*(cos(th5)*(cos(th2)*sin(th4) + cos(th3)*cos(th4)*sin(th2)) - sin(th2)*sin(th3)*sin(th5)) - cos(th6)*(cos(th2)*cos(th4) - cos(th3)*sin(th2)*sin(th4))) + d23*cos(th2);...
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                  0,                                                                                                                                                                                                                                                                              0,                                                                                                                                                                                                                                                                                                                                                                                                           1];
% x_cur = H(1,4);
% y_cur = H(2,4);
% z_cur = H(3,4);
% eul_cur = rotm2eul(H(1:3,1:3));
% eul = quat2eul(q);
%
% x_err = x - x_cur;
% y_err = y - y_cur;
% z_err = z - z_cur;
% eul_err = eul - eul_cur;
% a_err = eul_err(3);
% b_err = eul_err(2);
% g_err = eul_err(1);
%
% if (on_traj == true)
%
% else
%     Qf = IK_calc2(x, y, z, q, th_cur); %final pose
%
%     if (tr_mode == true)
%
%     else
%
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% th1 = js_cur_msg.Position(1);
% th2 = js_cur_msg.Position(2);
% th3 = js_cur_msg.Position(3);
% th4 = js_cur_msg.Position(4);
% th5 = js_cur_msg.Position(5);
% th6 = js_cur_msg.Position(6);
% th7 = js_cur_msg.Position(7);
% th_cur = [th1, th2, th3, th4, th5, th6, th7];

% function [control_js_msg, tr, qs, qf, t, out_of_trj, TR_, enblr]   = fcn(blank_js_msg, Qs, Qf, Q2, Q3, traj_mode)
%
%
% persistent Qs_;
% if isempty(Qs_)
%     Qs_ = [0, 0, 0, 0, 0, 0];
% end
%
% persistent TRth;
% if isempty(TRth)
%     TRth = zeros(300,6);
%
% enblr = 0;
%
% out_of_trj = true;
%
% str_array_of_jnames = {'iiwa_j1', 'iiwa_j2', 'iiwa_j3', 'iiwa_j4', 'iiwa_j5', 'iiwa_j6', 'iiwa_j7'};
% control_js_msg = blank_js_msg;
%
% th_num = length(str_array_of_jnames);
% control_js_msg.Name_SL_Info.CurrentLength = uint32(th_num);
%
% for idx=1:th_num
%     str = str_array_of_jnames{idx};
%     str_length = length(str);
%     control_js_msg.Name(idx).Data(1:str_length) = uint8(str);
%     control_js_msg.Name(idx).Data_SL_Info.CurrentLength = uint32(str_length);
% end
%
% if (traj_mode == false)
%     control_js_msg.Position_SL_Info.CurrentLength = uint32(th_num);
%     control_js_msg.Position(1) = double(Qf(1));
%     control_js_msg.Position(2) = double(Qf(2));
%     control_js_msg.Position(3) = double(Qf(3));
%     control_js_msg.Position(4) = double(Qf(4));
%     control_js_msg.Position(5) = double(Qf(5));
%     control_js_msg.Position(6) = double(Qf(6));
%     control_js_msg.Position(7) = double(Qf(7));
%
%     control_js_msg.Header.Seq = uint32(seq);
%     seq = seq + 1;
%     tr = zeros(1,6);
%     tr(1) = Qf(1); tr(2) = Qf(2); tr(3) = Qf(4);
%     tr(4) = Qf(5); tr(5) = Qf(6); tr(6) = Qf(7);
%     t = 0;
%     TR_ = zeros(300,3);
% else
%     if (on_traj == false)
%         Qs_(1:2) = Qs(1:2); Qs_(3:6) = Qs(4:7);
%         Q2_(1:2) = Q2(1:2); Q2_(3:6) = Q2(4:7);
%         Q3_(1:2) = Q3(1:2); Q3_(3:6) = Q3(4:7);
%         Qf_(1:2) = Qf(1:2); Qf_(3:6) = Qf(4:7);
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         enblr = 1;
%         %TR_ = zeros(300,3);
%         for j = 1:1:300
%             tj = j/100;
%             TRth(j,:) = traj434(2, Qs_', Q2_', Q3_', Qf_', tj);
%             TR(j,:) = FK(TRth(j,1), TRth(j,2), 0.0, TRth(j,3), TRth(j,4), TRth(j,5), TRth(j,6));
%             %plot3(TR(1:301,1), TR(1:301,2), TR(1:301,3));
%         end
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%         on_traj = true;
%     end
%     TR_ = TR;
%     out_of_trj = false;
%
%     t = i/100;
%     tr = traj434(2, Qs_', Q2_', Q3_', Qf_', t);
%     i = i + 1;
%     if (i== 300)
%         i = 0;
%         out_of_trj = true;
%         on_traj = false;
%     end
%
%     control_js_msg.Position_SL_Info.CurrentLength = uint32(th_num);
%     control_js_msg.Position(1) = double(tr(1));
%     control_js_msg.Position(2) = double(tr(2));
%     control_js_msg.Position(3) = double(0.0);
%     control_js_msg.Position(4) = double(tr(3));
%     control_js_msg.Position(5) = double(tr(4));
%     control_js_msg.Position(6) = double(tr(5));
%     control_js_msg.Position(7) = double(tr(6));
%
%     control_js_msg.Header.Seq = uint32(seq);
%     seq = seq + 1;
%
% end
%
%
% end
%
%
%
% function tr = traj434(in_t, in_qs, in_q2, in_q3, in_qf, t)%, in_dqs, in_dqf, in_ddqs, in_ddqf)
%
%     n = 6;%numrows(in_qs);
%
%     % Обнуление не заданных граничных условий
%     %if nargin == 6
%         in_dqs  = zeros(1,n)'; in_dqf  = zeros(1,n)';
%         in_ddqs = zeros(1,n)'; in_ddqf = zeros(1,n)';
%     %end
%
%     % интерполяция с интервалом в 10мс
%     %itr_n = in_t*100;
%     %t_step = 1/itr_n;
%
%     % Соответствующая матрица из выражения
%     Am = [0  0  0  0  1  0  0   0   0   0   0   0   0   0;...
%           0  0  0  1  0  0  0   0   0   0   0   0   0   0;...
%           0  0  2  0  0  0  0   0   0   0   0   0   0   0;...
%           1  1  1  1  1  0  0   0   0   0   0   0   0   0;...
%           0  0  0  0  0  0  0   0   1   0   0   0   0   0;...
%           4  3  2  1  0  0  0   -1  0   0   0   0   0   0;...
%           12 6  2  0  0  0  -2  0   0   0   0   0   0   0;...
%           0  0  0  0  0  1  1   1   1   0   0   0   0   0;...
%           0  0  0  0  0  0  0   0   0   0   0   0   0   1;...
%           0  0  0  0  0  3  2   1   0   0   0   0   -1  0;...
%           0  0  0  0  0  6  2   0   0   0   0   -2  0   0;...
%           0  0  0  0  0  0  0   0   0   1   1   1   1   1;...
%           0  0  0  0  0  0  0   0   0   4   3   2   1   0;...
%           0  0  0  0  0  0  0   0   0   12  6   2   0   0];
%     Ami = inv(Am);
%
%     % Определение полиномиальных коэффициентов для граничных условий
%     % всех обобщенных координат A, скоростей dA и ускорений ddA
%     A = ones(6,14);
%     for i=1:n
%         A(i,:)   = Ami*[in_qs(i); in_dqs(i); in_ddqs(i); in_q2(i); in_q2(i);0; 0; ...
%                         in_q3(i); in_q3(i); 0; 0; in_qf(i); in_dqf(i); in_ddqf(i)];
%     end
%
%     % Вычисление значений интерполирующих полиномов по всей траектории
%     tr = zeros(1,n);
%     if t <= 1
%         for i=1:n
%             tr(1,i) = polyval(A(i,1:5),t);
%         end
%     end
%
%     if (t > 1) && (t <= 2)
%         for i=1:n
%             tr(1,i) = polyval(A(i,6:9),(t-1));
%         end
%     end
%
%     if (t > 2)
%         for i=1:n
%             tr(1,i) = polyval(A(i,10:14),(t-2));
%         end
%     end
% end
%
%
% function TRxyz = FK(th1, th2, th3, th4, th5, th6, th7)
% d01 = 0.36;
% d23 = 0.42;
% d45 = 0.4;
% d67 = 0.126;
% H = [cos(th7)*(sin(th6)*(sin(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) - cos(th1)*cos(th4)*sin(th2)) - cos(th6)*(cos(th5)*(cos(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + cos(th1)*sin(th2)*sin(th4)) + sin(th5)*(cos(th3)*sin(th1) + cos(th1)*cos(th2)*sin(th3)))) + sin(th7)*(sin(th5)*(cos(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + cos(th1)*sin(th2)*sin(th4)) - cos(th5)*(cos(th3)*sin(th1) + cos(th1)*cos(th2)*sin(th3))), cos(th7)*(sin(th5)*(cos(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + cos(th1)*sin(th2)*sin(th4)) - cos(th5)*(cos(th3)*sin(th1) + cos(th1)*cos(th2)*sin(th3))) - sin(th7)*(sin(th6)*(sin(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) - cos(th1)*cos(th4)*sin(th2)) - cos(th6)*(cos(th5)*(cos(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + cos(th1)*sin(th2)*sin(th4)) + sin(th5)*(cos(th3)*sin(th1) + cos(th1)*cos(th2)*sin(th3)))),   cos(th6)*(sin(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) - cos(th1)*cos(th4)*sin(th2)) + sin(th6)*(cos(th5)*(cos(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + cos(th1)*sin(th2)*sin(th4)) + sin(th5)*(cos(th3)*sin(th1) + cos(th1)*cos(th2)*sin(th3))),   d67*(cos(th6)*(sin(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) - cos(th1)*cos(th4)*sin(th2)) + sin(th6)*(cos(th5)*(cos(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + cos(th1)*sin(th2)*sin(th4)) + sin(th5)*(cos(th3)*sin(th1) + cos(th1)*cos(th2)*sin(th3)))) + d45*(sin(th4)*(sin(th1)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) - cos(th1)*cos(th4)*sin(th2)) - d23*cos(th1)*sin(th2);...
%      -cos(th7)*(sin(th6)*(sin(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) + cos(th4)*sin(th1)*sin(th2)) - cos(th6)*(cos(th5)*(cos(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) - sin(th1)*sin(th2)*sin(th4)) + sin(th5)*(cos(th1)*cos(th3) - cos(th2)*sin(th1)*sin(th3)))) - sin(th7)*(sin(th5)*(cos(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) - sin(th1)*sin(th2)*sin(th4)) - cos(th5)*(cos(th1)*cos(th3) - cos(th2)*sin(th1)*sin(th3))), sin(th7)*(sin(th6)*(sin(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) + cos(th4)*sin(th1)*sin(th2)) - cos(th6)*(cos(th5)*(cos(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) - sin(th1)*sin(th2)*sin(th4)) + sin(th5)*(cos(th1)*cos(th3) - cos(th2)*sin(th1)*sin(th3)))) - cos(th7)*(sin(th5)*(cos(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) - sin(th1)*sin(th2)*sin(th4)) - cos(th5)*(cos(th1)*cos(th3) - cos(th2)*sin(th1)*sin(th3))), - cos(th6)*(sin(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) + cos(th4)*sin(th1)*sin(th2)) - sin(th6)*(cos(th5)*(cos(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) - sin(th1)*sin(th2)*sin(th4)) + sin(th5)*(cos(th1)*cos(th3) - cos(th2)*sin(th1)*sin(th3))), - d67*(cos(th6)*(sin(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) + cos(th4)*sin(th1)*sin(th2)) + sin(th6)*(cos(th5)*(cos(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) - sin(th1)*sin(th2)*sin(th4)) + sin(th5)*(cos(th1)*cos(th3) - cos(th2)*sin(th1)*sin(th3)))) - d45*(sin(th4)*(cos(th1)*sin(th3) + cos(th2)*cos(th3)*sin(th1)) + cos(th4)*sin(th1)*sin(th2)) - d23*sin(th1)*sin(th2);...
%      cos(th7)*(cos(th6)*(cos(th5)*(cos(th2)*sin(th4) + cos(th3)*cos(th4)*sin(th2)) - sin(th2)*sin(th3)*sin(th5)) + sin(th6)*(cos(th2)*cos(th4) - cos(th3)*sin(th2)*sin(th4))) - sin(th7)*(sin(th5)*(cos(th2)*sin(th4) + cos(th3)*cos(th4)*sin(th2)) + cos(th5)*sin(th2)*sin(th3)),                                                                                                                                                                                     - cos(th7)*(sin(th5)*(cos(th2)*sin(th4) + cos(th3)*cos(th4)*sin(th2)) + cos(th5)*sin(th2)*sin(th3)) - sin(th7)*(cos(th6)*(cos(th5)*(cos(th2)*sin(th4) + cos(th3)*cos(th4)*sin(th2)) - sin(th2)*sin(th3)*sin(th5)) + sin(th6)*(cos(th2)*cos(th4) - cos(th3)*sin(th2)*sin(th4))),                                                                                                                  cos(th6)*(cos(th2)*cos(th4) - cos(th3)*sin(th2)*sin(th4)) - sin(th6)*(cos(th5)*(cos(th2)*sin(th4) + cos(th3)*cos(th4)*sin(th2)) - sin(th2)*sin(th3)*sin(th5)),                                                                                                                                                             d01 + d45*(cos(th2)*cos(th4) - cos(th3)*sin(th2)*sin(th4)) - d67*(sin(th6)*(cos(th5)*(cos(th2)*sin(th4) + cos(th3)*cos(th4)*sin(th2)) - sin(th2)*sin(th3)*sin(th5)) - cos(th6)*(cos(th2)*cos(th4) - cos(th3)*sin(th2)*sin(th4))) + d23*cos(th2);...
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                  0,                                                                                                                                                                                                                                                                              0,                                                                                                                                                                                                                                                                                                                                                                                                           1];
% TRxyz = H*[0;0;0;1];
% TRxyz = TRxyz(1:3,1)';
% end
