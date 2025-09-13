classdef DevanshControl < handle
    properties
        params
        t_span
        init_state
        root_folder  % will depend on systemType
        systemType
        cases
    end

    methods
        function obj = DevanshControl(x0, x0_hat, obs_center, obs_radius, ...
                fun_G, fun_A, b_minus, b_plus, systemType)

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

            % RCBF parameters (Tunable)
            obj.params.k_alpha = 1;
            obj.params.b_minus = b_minus;
            obj.params.b_plus = b_plus;
            obj.params.umin = -10;
            obj.params.umax =  10;

            % Dynamics
            obj.params.fun_A = fun_A;
            obj.params.fun_G = fun_G;

            % Circular obstacle parameters
            obj.params.obs_center = obs_center;
            obj.params.obs_radius = obs_radius;

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
            case_name = "DevanshRCBF";
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

            u_des = obj.controlLaw(x_hat);
            [u, ~] = obj.safeController( x_hat, u_des); % Devansh Controller

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

            A_x_hat = obj.SDCMatrix(x_hat);
            term1 = (A_x_hat + obj.params.alpha * eye(obj.params.n)) * theta;
            term2 = theta * (A_x_hat' + obj.params.alpha * eye(obj.params.n));
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


        function [u, info] = safeController(obj, x_hat, u_des)
            % SAFE_CONTROLLER  RCBF-based safe control without slack variable
            % Inputs:
            %   x_hat : estimated state [x1; x2]
            %   u_des : desired input
            %
            % Outputs:
            %   u     : safe control input
            %   info  : diagnostics

            p = obj.params;

            % Dynamics
            f_hat = obj.f(x_hat);
            g_hat = obj.g(x_hat);
            h_hat = obj.constraint(x_hat);
            grad_h = obj.gradh(x_hat);

            % Lie derivatives
            Lf_h = grad_h * f_hat;
            Lg_h = grad_h * g_hat;

            % Barrier condition value
            a_val = Lf_h + p.k_alpha * h_hat;

            % Assumption-2 bounds on Lg h = C B = 1
            bminus = obj.params.b_minus;
            bplus  = obj.params.b_plus;

            % ----- QP : decision z = [u; s] -----
            % min  ||u - u_des||^2   s.t.
            %   s <= b^- u
            %   s <= b^+ u
            %   a + s >= 0   (i.e., -s <=  a)
            H = diag([2, 0]);         % 1/2 z'H z + f'z  with z=[u;s]
            f = [-2*u_des; 0];

            A = [ -bminus,  1;       % s - b^- u <= 0
                -bplus ,  1;       % s - b^+ u <= 0
                0     , -1];      % -s <= a   -> a + s >= 0
            b = [0; 0; a_val];

            lb = [obj.params.umin; -inf];
            ub = [obj.params.umax;  inf];

            qpopt = optimoptions('quadprog','Display','off');
            [z, ~, exitflag, output, lambda] = quadprog(H, f, A, b, [], [], lb, ub, [], qpopt);

            if exitflag <= 0
                % Hard constraint can be infeasible. Fall back: project u_des onto
                % the feasible set corners (choose u s.t. min{b^-u, b^+u} >= -a).
                % Conservative projection:
                req = -a_val;                 % need min{b^-u, b^+u} >= req
                u_req = max(req/bminus, req/bplus);
                u = min(max(max(u_des, u_req), obj.params.umin), obj.params.umax);
                feas_flag = 0;
            else
                u = z(1);
                feas_flag = 1;
            end

            % Diagnostics
            info.exitflag = exitflag;
            info.output   = output;
            info.lambda   = lambda;
            info.h        = h_hat;
            info.a        = a_val;
            info.bminus   = bminus;
            info.bplus    = bplus;
            info.feasible = feas_flag;
        end

    end
end
