rosshutdown;
rosinit('http://ivory.local:11311')
alpha = 1.5;
delpha = -1.5;

format long;

cmd_velPub = rospublisher('/catvehicle/cmd_vel', 'geometry_msgs/Twist');
pause(3); % Wait to ensure publisher is registered

velmsg = rosmessage(cmd_velPub);
velmsg.Angular.Z = 0;

velmsg.Linear.X = 0;

%% State 1: Accelerate to 55mph~24m/s
accelerate(cmd_velPub, velmsg, velmsg.Linear.X , 24.0, alpha);
fprintf('\nState 1 finished. Current velocity is %16.8f',velmsg.Linear.X);

%% State 2: Hold to 55mps~24m/s for 60.0 seconds
hold_velocity(cmd_velPub, velmsg, 60.0);
fprintf('\nState 2 finished. Current velocity is %16.8f',velmsg.Linear.X);

%% State 3: Decelerate to 35mph~15m/s
decelerate(cmd_velPub, velmsg, velmsg.Linear.X , 15.0, delpha);
fprintf('\nState 3 finished. Current velocity is %16.8f',velmsg.Linear.X);

%% State 4: Hold to 35mps~15m/s for 60.0 seconds
hold_velocity(cmd_velPub, velmsg, 60.0);
fprintf('\nState 4 finished. Current velocity is %16.8f',velmsg.Linear.X);

%% State 5: Accelerate to 55mph~24m/s
accelerate(cmd_velPub, velmsg, velmsg.Linear.X , 24.0, alpha);
fprintf('\nState 5 finished. Current velocity is %16.8f',velmsg.Linear.X);

%% State 6: Hold to 55mps~24m/s for 60.0 seconds
hold_velocity(cmd_velPub, velmsg, 60.0);
fprintf('\nState 6 finished. Current velocity is %16.8f',velmsg.Linear.X);

%% State 7: Decelerate to 45mph~20m/s
decelerate(cmd_velPub, velmsg, velmsg.Linear.X , 20.0, delpha);
fprintf('\nState 7 finished. Current velocity is %16.8f',velmsg.Linear.X);

%% State 8: Hold to 45mps~20m/s for 60.0 seconds
hold_velocity(cmd_velPub, velmsg, 60.0)
fprintf('\nState 8 finished. Current velocity is %16.8f',velmsg.Linear.X);

%% State 9: Accelerate to 55mph~24m/s
accelerate(cmd_velPub, velmsg, velmsg.Linear.X , 24.0, alpha);
fprintf('\nState 9 finished. Current velocity is %16.8f',velmsg.Linear.X);

%% State 10: Hold to 55mps~24m/s for 60.0 seconds
hold_velocity(cmd_velPub, velmsg, 60.0);
fprintf('\nState 10 finished. Current velocity is %16.8f',velmsg.Linear.X);

%% State 11: Decelerate to 0m/s
decelerate(cmd_velPub, velmsg, velmsg.Linear.X , 0.0, delpha);
fprintf('\nState 11 finished. Current velocity is %16.8f',velmsg.Linear.X);

function accelerate(publisher, message, vel_init, vel_end, acceleration)
    t0 = rostime('now');
    sec_t0 = double(t0.Sec)+double(t0.Nsec)*10^-9;
    while(message.Linear.X <= vel_end)
        current = rostime('now');
        sec_current = double(current.Sec)+double(current.Nsec)*10^-9;
        message.Linear.X = vel_init + acceleration*(sec_current - sec_t0);
        send(publisher,message);
    end
end

function decelerate(publisher, message, vel_init, vel_end, deceleration)
    t0 = rostime('now');
    sec_t0 = double(t0.Sec)+double(t0.Nsec)*10^-9;
    while(message.Linear.X >= vel_end)
        current = rostime('now');
        sec_current = double(current.Sec)+double(current.Nsec)*10^-9;
        message.Linear.X = vel_init + deceleration*(sec_current - sec_t0);
        send(publisher,message);
    end
end

function hold_velocity(publisher, message, seconds)
    t0 = rostime('now');
    sec_t0 = double(t0.Sec)+double(t0.Nsec)*10^-9;

    while(1)
        send(publisher, message);
        current = rostime('now');
        sec_current = double(current.Sec)+double(current.Nsec)*10^-9;
        fprintf('\nDiff time: %16.8f\n',sec_current - sec_t0);
        if(sec_current - sec_t0 >= seconds)
            break;
        end
    end
end
