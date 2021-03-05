clc, clear, close all
format long

%% init master class
vehicle = vehicle;
vehicle.plotter = vehicle.plotter.update;
vehicle.plotter = vehicle.plotter.plot_pos;
x=1;
while true
    x = x + 1;
    vehicle = vehicle.sim_step;
    vehicle.plotter = vehicle.plotter.update;
    [crashed, ~]= vehicle.crashDetn.vehicle_crashed;
    if x == 10
        vehicle.plotter = vehicle.plotter.plot_pos;
        x = 1;
    end
    if crashed
        vehicle.plotter = vehicle.plotter.plot_pos;
        disp('vehicle crashed')
        break
    end
end



