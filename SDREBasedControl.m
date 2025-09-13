classdef SDREBasedControl < handle
    properties
        params
        t_span
        init_state
        root_folder  % will depend on systemType
        systemType
        cases
    end

    methods
        function obj = SDREBasedControl(x0, x0_hat, fun_G, fun_A, systemType)

            % Store system type
            obj.systemType = systemType;

            % Problem dimensions
            obj.params.n = 2;
            obj.params.m = 1;
            obj.params.q = 1;

            % Controller and observer parameters
            obj.params.Q = diag([1, 1]);
            obj.params.R = eye(obj.params.m);
            obj.params.M = 0.001 * eye(obj.params.n);
            obj.params.alpha = 1;
            obj.params.SIG = eye(obj.params.q);
            obj.params.C = [1 0];

            % Dynamics
            obj.params.fun_A = fun_A;
            obj.params.fun_G = fun_G;

            % Initial conditions
            theta0 = reshape(0.01 * eye(obj.params.n), obj.params.n^2, 1);
            obj.init_state = [x0; x0_hat; theta0];

            % Simulation time span
            obj.t_span = [0 25];

             % Update root folder to include systemType
            obj.root_folder = fullfile("data", systemType);

            % Ensure folder exists
            if ~exist(obj.root_folder, "dir")
                mkdir(obj.root_folder);
            end
        end

        function run(obj)
            % Run simulation without barrier
            case_name = "NoBarrier";
            ode = @(t, states) obj.closedLoopDynamics(t, states);
            [t, sol] = ode45(ode, obj.t_span, obj.init_state);

            % Extract components
            n = obj.params.n;
            x = sol(:, 1:n);
            x_hat = sol(:, n+1:2*n);
            e = x - x_hat;
            theta = sol(:, 2*n+1:end);

            % Save results
            outdir = fullfile(obj.root_folder, case_name);
            if ~exist(outdir, "dir"), mkdir(outdir); end
            
            save(fullfile(outdir, "t_data.dat"), "t", "-ascii");
            save(fullfile(outdir, "x_data.dat"), "x", "-ascii");
            save(fullfile(outdir, "x_hat_data.dat"), "x_hat", "-ascii");
            save(fullfile(outdir, "e_data.dat"), "e", "-ascii");
            save(fullfile(outdir, "theta_hat_data.dat"), "theta", "-ascii");

            % Saving data with time series for tikz
            e_with_time = [t,e];
            save(fullfile(outdir, "e_data_with_time.dat"), "e_with_time", "-ascii");

            fprintf("Saved all data for case: %s\n", case_name);
        end

        function states_dot = closedLoopDynamics(obj, ~, states)
            n = obj.params.n;
            x = states(1:n);
            x_hat = states(n+1:2*n);
            theta = reshape(states(2*n+1:end,1), n, n);

            u = obj.controlLaw(x_hat);
            x_dot = obj.f(x) + obj.g(x) * u;

            y = obj.params.C * x;
            y_hat = obj.params.C * x_hat;

            [x_hat_dot, theta_dot] = obj.observerUpdate(x_hat, theta, u, y, y_hat);

            states_dot = [x_dot;
                          x_hat_dot;
                          reshape(theta_dot, [], 1)];
        end

        function u = controlLaw(obj, x_hat)
            A_x_hat = obj.SDCMatrix(x_hat);
            G_x_hat = obj.g(x_hat);

            Q = obj.params.Q;
            R = obj.params.R;

            try
                Pn = care(A_x_hat, G_x_hat, Q, R);
            catch
                Pn = eye(obj.params.n);
            end

            u = -inv(R) * G_x_hat' * Pn * x_hat;
        end

        function [x_hat_dot, theta_dot] = observerUpdate(obj, x_hat, theta, u, y, y_hat)
            K_theta = theta' * obj.params.C' * obj.params.SIG;
            drift = obj.f(x_hat);
            controlEff = obj.g(x_hat) * u;
            correction = K_theta * (y - y_hat);
            x_hat_dot = drift + controlEff + correction;

            A_xhat = obj.SDCMatrix(x_hat);
            term1 = (A_xhat + obj.params.alpha * eye(obj.params.n)) * theta;
            term2 = theta * (A_xhat' + obj.params.alpha * eye(obj.params.n));
            term3 = theta * obj.params.C' * (inv(obj.params.SIG) * (obj.params.C * theta));

            theta_dot = term1 + term2 - term3 + obj.params.M;
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
    end
end
