% Parameters
H = 10000; % height of target (m)
missile_data = {
    'Jaguar', 1320, 5;
    'Tiger', 980, 1.0;
    'Leopard', 750, 0.5
};
n_missiles = size(missile_data, 1);

eta = 10.5104642; % latitude of Kaduna (degrees)
zta = eta * pi / 180;
g = 9.7803267714 * ((1 + 0.00193185138639 * (sin(zta))^2) / sqrt(1 - 0.00669437999013 * (sin(zta))^2 )); % Acceleration due to gravity

phi = 20; % Angle at which the radar sighted the target (degrees)
w = phi * pi / 180; % conversion of angle to radians
y_0 = H; % Initial height of target (m)
x_0 = y_0 * cot(w); % Initial horizontal position (m)
X_0 = 0; % initial horizontal position of launch or observation base (m)
Y_0 = 0; % initial vertical position of launch or observation base (m)

for i = 1:n_missiles
    figure; % Create a new figure for each missile
    hold on;

    missile_name = missile_data{i, 1};
    v = missile_data{i, 2}; % velocity of missile (m/s)
    T = missile_data{i, 3}; % Thrust of missile (N)

    % Solving the simultaneous equation numerically using vpasolve
    syms D t % positive
    [Sx, Sv] = vpasolve(X_0 + v * cos(D) * t + ((T - K * v.^2 * (cos(D)).^2) * cos(D) * t.^2 / (2 * m)) == (x_0 - (v_t * sin(w) * t)),...
                        Y_0 + v * sin(D) * t + (((T - K * v.^2 * (sin(D)).^2) * sin(D) / (2 * m)) - (g / 2)) * t.^2 == y_0 - (v_t * cos(w) * t + (g / 2) * t^2), [D, t]);
    B = double(Sx); % Angle of launch
    Time_of_intercept = double(Sv); % Time of interception

    % Position of missile
    f = 0:Time_of_intercept/100:Time_of_intercept;
    [X_missile, Y_missile] = deal(zeros(size(f)));
    for j = 1:numel(f)
        X_missile(j) = X_0 + v * cos(B) * f(j) + (((T - K * v.^2 * (cos(B)).^2) / (2 * m)) * cos(B) * f(j)^2);
        Y_missile(j) = Y_0 + v * sin(B) * f(j) + (((T - K * v.^2 * (sin(B)).^2) * sin(B) / (2 * m)) - (g / 2)) * f(j)^2;
    end

    plot(X_missile, Y_missile, 'DisplayName', missile_name);
    
    % Position of target
    X_target = x_0 - (v_t * sin(w) * f);
    Y_target = y_0 - (v_t * cos(w) * f + ((g / 2) * f.^2));
    plot(X_target, Y_target, 'g', 'DisplayName', 'Target');

    xlabel('Horizontal Distance (m)');
    ylabel('Vertical Distance (m)');
    title(['Missile Interception - ' missile_name]);
    legend('Location', 'Best');
    grid on;
    hold off;
end

% Combined graph
figure;
hold on;

for i = 1:n_missiles
    missile_name = missile_data{i, 1};
    v = missile_data{i, 2}; % velocity of missile (m/s)
    T = missile_data{i, 3}; % Thrust of missile (N)

    % Solving the simultaneous equation numerically using vpasolve
    syms D t % positive
    [Sx, Sv] = vpasolve(X_0 + v * cos(D) * t + ((T - K * v.^2 * (cos(D)).^2) * cos(D) * t.^2 / (2 * m)) == (x_0 - (v_t * sin(w) * t)),...
                        Y_0 + v * sin(D) * t + (((T - K * v.^2 * (sin(D)).^2) * sin(D) / (2 * m)) - (g / 2)) * t.^2 == y_0 - (v_t * cos(w) * t + (g / 2) * t^2), [D, t]);
    B = double(Sx); % Angle of launch
    Time_of_intercept = double(Sv); % Time of interception

    % Position of missile
    f = 0:Time_of_intercept/100:Time_of_intercept;
    [X_missile, Y_missile] = deal(zeros(size(f)));
    for j = 1:numel(f)
        X_missile(j) = X_0 + v * cos(B) * f(j) + (((T - K * v.^2 * (cos(B)).^2) / (2 * m)) * cos(B) * f(j)^2);
        Y_missile(j) = Y_0 + v * sin(B) * f(j) + (((T - K * v.^2 * (sin(B)).^2) * sin(B) / (2 * m)) - (g / 2)) * f(j)^2;
    end

    plot(X_missile, Y_missile, 'DisplayName', missile_name);
end

% Position of target
f = linspace(0, Time_of_intercept, 100);
X_target = x_0 - (v_t * sin(w) * f);
Y_target = y_0 - (v_t * cos(w) * f + ((g / 2) * f.^2));
plot(X_target, Y_target, 'g', 'DisplayName', 'Target');

xlabel('Horizontal Distance (m)');
ylabel('Vertical Distance (m)');
title('Missile Interception - Combined');
legend('Location', 'Best');
grid on;
hold off;
