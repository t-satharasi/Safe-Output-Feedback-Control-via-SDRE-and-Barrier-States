classdef SDREBasedWithBarrier < handle
    properties
        params
        t_span
        init_state
        root_folder  % will depend on systemType
        systemType
        cases
    end

    methods
        function obj = SDREBasedWithBarrier(x0, x0_hat, obs_center, ...
                obs_radius, fun_G, fun_A, systemType)

            % Store system type
            obj.systemType = systemType;

            % Problem setup
            obj.params.n = 2;
            obj.params.m = 1;
            obj.params.q = 1;

            obj.params.Q = diag([1, 1, 1]);
            obj.params.R = eye(obj.params.m);
            obj.params.gamma = 1;
            obj.params.K = 1;

            obj.params.M = 0.001 * eye(obj.params.n);
            obj.params.alpha = 1;
            obj.params.SIG = eye(obj.params.q);
            obj.params.C = [1 0];

            % Robust parameters
            obj.params.theta_under = 1;
            obj.params.theta_over = 0.3;
            obj.params.x0_bar = 1;
            
            % Circular obstacle parameters
            obj.params.obs_center = obs_center;
            obj.params.obs_radius = obs_radius;

            % Dynamics
            obj.params.fun_A = fun_A;
            obj.params.fun_G = fun_G;

            % initial states
            obj.params.init_states = [x0;x0_hat];

            obj.cases = {
                struct('L', 0,   'name', "BarrierOnly")
                struct('L', 0.3, 'name', "BarrierAndRobust")
            };

            % Update root folder to include systemType
            obj.root_folder = fullfile("data", systemType);

            % Ensure folder exists
            if ~exist(obj.root_folder, "dir")
                mkdir(obj.root_folder);
            end
        end

        function run(obj)
            for i = 1:numel(obj.cases)
                case_params = obj.cases{i};
                obj.params.L = case_params.L;

                % Initial conditions
                x0 = obj.params.init_states(1:obj.params.n);
               
                z0 = obj.beta(x0) - obj.beta([0; 0]);
                x0_hat = obj.params.init_states(obj.params.n+1:end);
                z0_hat = obj.betaHat(0, x0_hat) - obj.betaHat(0, [0; 0]) ;
                theta0 = reshape(0.01 * eye(obj.params.n), obj.params.n^2, 1);
                s0 = [x0; z0];
                s0_hat = [x0_hat; z0_hat];
                obj.init_state = [s0; s0_hat; theta0];

                obj.t_span = [0 25];
                

                ode = @(t, x) obj.closedLoopDynamics(t, x);
                [t, sol] = ode45(ode, obj.t_span, obj.init_state);

                x = sol(:, 1:2);
                x_hat = sol(:, 4:5);
                e = x - x_hat;
                theta = sol(:, 7:end);

                outdir = fullfile(obj.root_folder, case_params.name);
                if ~exist(outdir, "dir"), mkdir(outdir); end
                
                save(fullfile(outdir, "t_data.dat"), "t", "-ascii");
                save(fullfile(outdir, "x_data.dat"), "x", "-ascii");
                save(fullfile(outdir, "x_hat_data.dat"), "x_hat", "-ascii");
                save(fullfile(outdir, "e_data.dat"), "e", "-ascii");
                save(fullfile(outdir, "theta_hat_data.dat"), "theta", "-ascii");

                % Saving data with time series for tikz
                e_with_time = [t,e];
                save(fullfile(outdir, "e_data_with_time.dat"), "e_with_time", "-ascii");

                fprintf("Saved all data for case: %s\n", case_params.name);
            end
        end
    end

    methods (Access = private)
        function states_dot = closedLoopDynamics(obj, t, states)
            p = obj.params;
            n = p.n;

            s = states(1:n+1);
            s_hat = states(n+2:2*(n+1));
            theta = reshape(states(2*(n+1)+1:end,1), n, n);

            x = s(1:n); z = s(n+1);
            x_hat = s_hat(1:n); z_hat = s_hat(n+1);

            u = obj.controlLaw(t, s_hat);

            x_dot = obj.f(x) + obj.g(x) * u;
            z_dot = obj.barrierDynamics(x, z, u);

            y = p.C * x;
            y_hat = p.C * x_hat;

            [s_hat_dot, theta_dot] = obj.observerUpdate(t, x_hat, z_hat, theta, u, y, y_hat);

            states_dot = [x_dot; z_dot; s_hat_dot; reshape(theta_dot, [], 1)];
        end

        function F = f(obj, x)
            F = obj.SDCMatrix(x) * x;
        end

        function G = g(obj, x)
            G = obj.params.fun_G(x);
        end

        function A = SDCMatrix(obj, x)
            A = obj.params.fun_A(x);
        end

        function h = constraint(obj, x)
            c = obj.params.obs_center;
            r = obj.params.obs_radius;
            h = (x(1) - c(1))^2 + (x(2) - c(2))^2 - r^2;
        end

        function dh = gradh(obj, x)
            c = obj.params.obs_center;
            dh = 2 * [x(1) - c(1), x(2) - c(2)];
        end

       function db = gradb(obj, x)
            K = obj.params.K;
            db  = -K/x^2;
        end

        function out = Phi(obj, x)
            K = obj.params.K;
            if K == 0, out = 0; else, out = -x^2 / K; end
        end

        function z_dot = barrierDynamics(obj, x, z, u)
            beta0 = obj.beta([0; 0]);
            beta_x = obj.beta(x);
            phi = obj.Phi(z + beta0);
            grad = obj.gradh(x);
            z_dot = phi * grad * (obj.f(x) + obj.g(x) * u) ...
                    - obj.params.gamma * (z + beta0 - beta_x);
        end

        function B = beta(obj, x)
            K = obj.params.K;
            h = obj.constraint(x);
            B = K / h;
        end

        function B_hat = betaHat(obj, t, x_hat)
            K = obj.params.K;
            eps = obj.errorBound(t);
            h_hat = obj.constraint(x_hat);
            B_hat = K / (h_hat - eps);
        end

        function eps = errorBound(obj, t)
            p = obj.params;
            eps = p.L * sqrt(p.theta_over / p.theta_under) * p.x0_bar * exp(-max(eig(p.M)*t));
        end

        function A_hat = A(obj, t, s_hat)
            n = obj.params.n;
            x_hat = s_hat(1:n); z_hat = s_hat(n+1);
            beta0t = obj.betaHat(t, [0; 0]);
            gamma = obj.params.gamma;
            A_x = obj.SDCMatrix(x_hat);
            A_z = obj.Phi(z_hat + beta0t) * obj.gradh(x_hat) * A_x ...
                  + gamma * obj.Psi(t, x_hat);
            A_hat = [A_x, zeros(n, 1); A_z, -gamma];
        end

        function u = controlLaw(obj, t, s_hat)
            Q = obj.params.Q;
            R = obj.params.R;
            A1 = obj.A(t, s_hat);
            G1 = obj.AugmentedControlEffect(t, s_hat);
            try
                P = care(A1, G1, Q, R);
            catch
                P = eye(size(A1));
            end
            u = -inv(R) * G1' * P * s_hat;
        end

        function G_hat = AugmentedControlEffect(obj, t, s_hat)
            n = obj.params.n;
            x_hat = s_hat(1:n);
            z_hat = s_hat(n + 1);
            beta0t = obj.betaHat(t, [0; 0]);
            G_hat = [obj.g(x_hat);
                     obj.Phi(z_hat + beta0t) * obj.gradh(x_hat) * obj.g(x_hat)];
        end

        function drift = AugmentedDrift(obj, t, s_hat)
            n = obj.params.n;
            x_hat = s_hat(1:n); z_hat = s_hat(n+1);
            beta0t = obj.betaHat(t, [0; 0]);
            beta_hat = obj.betaHat(t, x_hat);
            gamma = obj.params.gamma;
            drift = [obj.f(x_hat);
                     obj.Phi(z_hat + beta0t) * obj.gradh(x_hat) * obj.f(x_hat) ...
                     - gamma * (z_hat + beta0t - beta_hat)];
        end

        function out = Psi(obj, t, x_hat)
            beta0t = obj.betaHat(t, [0; 0]);
            obs = obj.params.obs_center;
            K = obj.params.K;
            if K == 0, out = zeros(1, 2); return; end
            out = (1/K) * obj.betaHat(t, x_hat) * beta0t * ...
                  [-x_hat(1) + 2*obs(1), -x_hat(2) + 2*obs(2)];
        end

        function [s_hat_dot, theta_dot] = observerUpdate(obj, t, x_hat, z_hat, theta, u, y, y_hat)
            p = obj.params;
            K_theta = theta' * p.C' * p.SIG;
            beta0t = obj.betaHat(t, [0; 0]);

            e_y = y - y_hat;
            phi = obj.Phi(z_hat + beta0t);
            grad = obj.gradh(x_hat);
            h0 = obj.constraint([0;0]);
            beta_prime = obj.gradb(h0 - obj.errorBound(t));

            rho = phi * obj.errorBound(t) * (-max(eig(p.M))) ...
                + beta_prime * obj.errorBound(t) * (-max(eig(p.M)));

            correction = [K_theta * e_y;
                          -rho + phi * grad * K_theta * e_y];

            drift = obj.AugmentedDrift(t, [x_hat; z_hat]);
            control = obj.AugmentedControlEffect(t, [x_hat; z_hat]) * u;

            s_hat_dot = drift + control + correction;

            A_x = obj.SDCMatrix(x_hat);
            term1 = (A_x + p.alpha * eye(p.n)) * theta;
            term2 = theta * (A_x' + p.alpha * eye(p.n));
            term3 = theta * p.C' * (inv(p.SIG) * (p.C * theta));
            theta_dot = term1 + term2 - term3 + p.M;
        end
    end
end
