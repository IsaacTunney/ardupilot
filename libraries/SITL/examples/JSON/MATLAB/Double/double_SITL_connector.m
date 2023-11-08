% this function handles all the UDP connection to SITL using the TCP/UDP/IP
% Toolbox 2.0.6 by Peter Rydesäter
% https://uk.mathworks.com/matlabcentral/fileexchange/345-tcp-udp-ip-toolbox-2-0-6

function double_SITL_connector(state1, state2, init_yaw1, init_yaw2, init_function1, init_function2, physics_function1, physics_function2, UDPport1, UDPport2, max_timestep)
try
    pnet('closeall') % close any connections left open from past runs
catch
    warning('Could not execute pnet mex, trying to compile')
    if ispc
        % running on windows
        mex -O -outdir ../tcp_udp_ip_2.0.6 ../tcp_udp_ip_2.0.6/pnet.c ws2_32.lib -DWIN32
    else
        % running on unix or mac
        mex -O -outdir ../tcp_udp_ip_2.0.6 ../tcp_udp_ip_2.0.6/pnet.c
    end
    try
        pnet('closeall')
    catch
        error('Failed to compile pnet mex file, see tcp_udp_ip_2.0.6/pnet.c for instructions on manual complication')
    end
end

% init physics
state1 = init_function1(state1,init_yaw1);
state2 = init_function2(state2,init_yaw2);

% Init the UDP port
u1 = pnet('udpsocket',UDPport1);
pnet(u1,'setwritetimeout',1);
pnet(u1,'setreadtimeout',0);

u2 = pnet('udpsocket',UDPport2);
pnet(u2,'setwritetimeout',1);
pnet(u2,'setreadtimeout',0);

sched_timer = tic;
frame_time = tic;
frame_count1 = 0;
physics_time_s1 = 0;
last_SITL_frame1 = -1;

frame_count2 = 0;
physics_time_s2 = 0;
last_SITL_frame2 = -1;

timeout_time = 1;
print_frame_count = 1000; % print the fps every x frames
connected1 = false;
connected2 = false;
bytes_read =  4 + 4 + 16*2; % the number of bytes received in each packet
while true
%     in_bytes1 = 0;
%     in_bytes2 = 0;
    if connected1 && connected2
        readTimeout1 = tic;
        while true
            in_bytes1 = pnet(u1,'readpacket',bytes_read);
            if in_bytes1 > 0
                break;
            end
            if toc(readTimeout1) > timeout_time
                fprintf('Connection 1 disconnected\n')
                connected1 = false;
                break;
            end
        end
        readTimeout2 = tic;
        while true
            in_bytes2 = pnet(u2,'readpacket',bytes_read);
            if in_bytes2 > 0
                break;
            end
            if toc(readTimeout2) > timeout_time
                fprintf('Connection 2 disconnected\n')
                connected2 = false;
                break;
            end
        end
    else
        in_bytes1 = pnet(u1,'readpacket',bytes_read);
        in_bytes2 = pnet(u2,'readpacket',bytes_read);
    end
    
    if in_bytes1 > 0
        connectionTimeout1 = tic;
    end
    if in_bytes2 > 0
        connectionTimeout2 = tic;
    end

    if connected1
        if toc(connectionTimeout1) > timeout_time
            fprintf('Connection 1 disconnected\n')
            connected1 = false;
        end
    end

    if connected2
        if toc(connectionTimeout2) > timeout_time
            fprintf('Connection 2 disconnected\n')
            connected2 = false;
        end
    end
    
    % if there is another frame waiting, read it straight away
    if in_bytes1 > bytes_read
        if in_bytes1 == u1.InputBufferSize
            % buffer got full, reset
            % should only happen if we have been paused in Matlab for some time
            fprintf('Buffer reset\n')
            continue;
        end
        continue;
    end

    if in_bytes2 > bytes_read
        if in_bytes2 == u2.InputBufferSize
            % buffer got full, reset
            % should only happen if we have been paused in Matlab for some time
            fprintf('Buffer reset\n')
            continue;
        end
        continue;
    end

    % read in data from AP and check the magic value is what expect
    if in_bytes1 > 0
        magic1 = pnet(u1,'read',1,'UINT16','intel');
        frame_rate1 = double(pnet(u1,'read',1,'UINT16','intel'));
        SITL_frame1 = pnet(u1,'read',1,'UINT32','intel');
        pwm_in1 = double(pnet(u1,'read',16,'UINT16','intel'))';

        if magic1 ~= 18458
            warning('incorrect magic value')
            continue;
        end
    end

    if in_bytes2 > 0
        magic2 = pnet(u2,'read',1,'UINT16','intel');
        frame_rate2 = double(pnet(u2,'read',1,'UINT16','intel'));
        SITL_frame2 = pnet(u2,'read',1,'UINT32','intel');
        pwm_in2 = double(pnet(u2,'read',16,'UINT16','intel'))';

        if magic2 ~= 18458
            warning('incorrect magic value')
            continue;
        end

    end
    dup1 = false;
    % Check if the fame is in expected order
    if in_bytes1 > 0
        if SITL_frame1 < last_SITL_frame1
            % Controller has reset, reset physics also
            state1 = init_function1(state1, init_yaw1);
            physics_time_s1 = 0;
            connected1 = false;
            fprintf('Controller reset\n')
        elseif SITL_frame1 == last_SITL_frame1
            % duplicate frame, skip
            fprintf('Duplicate input frame for 1\n')
            dup1 = true;
        elseif SITL_frame1 ~= last_SITL_frame1 + 1 && connected1
            fprintf('Missed %i input frames\n',SITL_frame1 - last_SITL_frame1 - 1)
        end

    end
    
    dup2 = false;
    if in_bytes2 > 0
        if SITL_frame2 < last_SITL_frame2
            % Controller has reset, reset physics also
            state2 = init_function2(state2, init_yaw2);
            physics_time_s2 = 0;
            connected2 = false;
            fprintf('Controller reset\n')
        elseif SITL_frame2 == last_SITL_frame2
            % duplicate frame, skip
            fprintf('Duplicate input frame for 2\n')
            dup2 = true;
        elseif SITL_frame2 ~= last_SITL_frame2 + 1 && connected2
            fprintf('Missed %i input frames\n',SITL_frame2 - last_SITL_frame2 - 1)
        end
        
    end

    if dup1 || dup2
        continue;
    else
        if in_bytes1
            last_SITL_frame1 = SITL_frame1;
            state1.delta_t = min(1/frame_rate1,max_timestep);
            physics_time_s1 = physics_time_s1 + state1.delta_t;
        end
        if in_bytes2
            last_SITL_frame2 = SITL_frame2;
            state2.delta_t = min(1/frame_rate2,max_timestep);
            physics_time_s2 = physics_time_s2 + state2.delta_t;
        end
    end

    if in_bytes1 > 0
        if ~connected1
            % use port -1 to indicate connection to address of last recv pkt
            connected1 = true;
            [ip, port] = pnet(u1,'gethost');
            fprintf('Connected to %i.%i.%i.%i:%i\n',ip,port)
        end
        frame_count1 = frame_count1 + 1;
    end
    if in_bytes2 > 0
        if ~connected2
            % use port -1 to indicate connection to address of last recv pkt
            connected2 = true;
            [ip, port] = pnet(u2,'gethost');
            fprintf('Connected to %i.%i.%i.%i:%i\n',ip,port)
        end
        frame_count2 = frame_count2 + 1;
    end

    % do a physics time step
    if in_bytes1 > 0 && ~dup1
        state1 = physics_function1(pwm_in1,state1);
        state1.time_s = physics_time_s1;
    end
    if in_bytes2 > 0 && ~dup2
        state2 = physics_function2(pwm_in2,state2);
        state2.time_s = physics_time_s2;
    end

    % build structure representing the JSON string to be sent
    if in_bytes1 > 0 || dup1
        JSON1.timestamp = physics_time_s1;
        JSON1.imu.gyro = state1.gyro;
        JSON1.imu.accel_body = state1.accel;
        JSON1.position = state1.position;
        JSON1.attitude = state1.attitude;
        JSON1.velocity = state1.velocity;
%         fprintf('IMU GYRO\n')
%         disp(JSON1.imu.gyro)
%         fprintf('IMU ACCEL\n')
%         disp(JSON1.imu.accel_body)
%         fprintf('POS\n')
%         disp(JSON1.position)
%         fprintf('ATT\n')
%         disp(JSON1.attitude)
%         fprintf('VEL\n')
%         disp(JSON1.velocity)
    end
    if in_bytes2 > 0 || dup2
        JSON2.timestamp = physics_time_s2;
        JSON2.imu.gyro = state2.gyro;
        JSON2.imu.accel_body = state2.accel;
        JSON2.position = state2.position;
        JSON2.attitude = state2.attitude;
        JSON2.velocity = state2.velocity;
    end

    % Report to AP
    if in_bytes1 > 0
        pnet(u1,'printf',sprintf('\n%s\n',jsonencode(JSON1)));
        pnet(u1,'writepacket');
    end
    if in_bytes2 > 0
        pnet(u2,'printf',sprintf('\n%s\n',jsonencode(JSON2)));
        pnet(u2,'writepacket');
    end

    % print a fps and runtime update
    if in_bytes1 > 0
        if rem(frame_count1,print_frame_count) == 0
            total_time = toc(frame_time);
            frame_time = tic;
            time_ratio = (print_frame_count*state1.delta_t)/total_time;
            fprintf("%0.2f fps, %0.2f%% of realtime\n",print_frame_count/total_time,time_ratio*100)
        end
    end
end

