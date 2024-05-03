% Prompt user to input parameters
H = input('Enter the height of the target (m): ');
n_missiles = input('Enter the number of missiles: ');

missile_data = cell(n_missiles, 3);
for i = 1:n_missiles
    missile_name = input(['Enter the name of missile ' num2str(i) ': '], 's');
    velocity = input(['Enter the velocity of ' missile_name ' (m/s): ']);
    delay = input(['Enter the operation delay of ' missile_name ' (s): ']);
    missile_data{i, 1} = missile_name;
    missile_data{i, 2} = velocity;
    missile_data{i, 3} = delay;
end

eta = input('Enter the latitude of Kaduna (degrees): ');
zta = eta * pi / 180;
g = 9.7803267714 * ((1 + 0.00193185138639 * (sin(zta))^2) / sqrt(1 - 0.00669437999013 * (sin(zta))^2 )); % Acceleration due to gravity

phi = input('Enter the angle at which the radar sighted the target (degrees): ');
w = phi * pi / 180; % conversion of angle to radians

% Prompt user to input initial target position
x_0 = input('Enter the initial horizontal position of the target (m): ');
y_0 = input('Enter the initial vertical position of the target (m): ');

% Constants
rho = 1.29; % air density (Kg/m^3)
C_d = 0.75; % Drag coefficient of circular cross section
Dr = 0.3; % diameter of missile(m)
A = pi * Dr^2 / 4; % cross sectional area of missile(m^2)

% Find the missile with the perfect angle of attack
perfect_angle_missile = missile_data{1, 1};

for i = 1:n_missiles
    figure;
    hold on;

    missile_name = missile_data{i, 1};
    v = missile_data{i, 2}; % velocity of missile (m/s)
    T = missile_data{i, 3}; % Thrust of missile (N)

    % Solve the simultaneous equation numerically using vpasolve
    syms D t % positive
    [Sx, Sv] = vpasolve(v * cos(D) * t + ((T - rho * C_d * A / 2 * v.^2 * (cos(D)).^2) * cos(D) * t.^2 / (2 * missile_data{i, 2})) == (x_0 - (missile_data{i, 2} * sin(w) * t)),...
                        v * sin(D) * t + (((T - rho * C_d * A / 2 * v.^2 * (sin(D)).^2) * sin(D) / (2 * missile_data{i, 2})) - (g / 2)) * t.^2 == y_0 - (missile_data{i, 2} * cos(w) * t + (g / 2) * t^2), [D, t]);
    B = double(Sx); % Angle of launch
    Time_of_intercept = double(Sv); % Time of interception

    % Check if the time of interception is valid
    if ~isempty(Time_of_intercept) && isreal(Time_of_intercept) && all(Time_of_intercept >= 0)
        % Position of missile
        f = 0:Time_of_intercept/100:Time_of_intercept;
        [X_missile, Y_missile] = deal(zeros(size(f)));
        for j = 1:numel(f)
            X_missile(j) = v * cos(B) * f(j) + (((T - rho * C_d * A / 2 * v.^2 * (cos(B)).^2) / (2 * missile_data{i, 2})) * cos(B) * f(j)^2);
            Y_missile(j) = v * sin(B) * f(j) + (((T - rho * C_d * A / 2 * v.^2 * (sin(B)).^2) * sin(B) / (2 * missile_data{i, 2})) - (g / 2)) * f(j)^2;
        end

        % Plot the trajectory
        plot(X_missile, Y_missile, 'DisplayName', missile_name);
    else
        % Adjust the angle of attack to ensure the missile does not intercept
        B = atan2(y_0, x_0);
        % Position of missile
        f = linspace(0, 10000, 100);
        [X_missile, Y_missile] = deal(zeros(size(f)));
        for j = 1:numel(f)
            X_missile(j) = v * cos(B) * f(j);
            Y_missile(j) = v * sin(B) * f(j) - (g / 2) * f(j)^2;
        end

        % Plot the trajectory
        plot(X_missile, Y_missile, 'DisplayName', [missile_name ' (Adjusted)']);
    end

    % Position of target
    f = linspace(0, Time_of_intercept, 100);
    X_target = x_0 - (missile_data{i, 2} * sin(w) * f);
    Y_target = y_0 - (missile_data{i, 2} * cos(w) * f + ((g / 2) * f.^2));
    plot(X_target, Y_target, 'g', 'DisplayName', 'Target');

    xlabel('Horizontal Distance (m)');
    ylabel('Vertical Distance (m)');
    title(['Missile Interception - ' missile_name]);
    legend('Location', 'Best');
    grid on;
    hold off;
end
