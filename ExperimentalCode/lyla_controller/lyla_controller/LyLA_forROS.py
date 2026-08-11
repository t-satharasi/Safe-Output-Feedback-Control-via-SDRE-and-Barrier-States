import torch
import torch.nn as nn
import numpy as np
import math
# import matplotlib.pyplot as plt
import json
import os
import rclpy


# ================== Dynamical System ====================== #
class Dynamics:
    """ a five dimensional nonlinear stochastic system dynamics"""

    @staticmethod
    def drift_vector (x):
        """ compute f(x) - the drift vector """
        x1, x2, x3, x4, x5, x6 = x

        f1 = 5 * torch.tanh(50 * x1) * x5**2 + torch.cos(x4)
        f2 = torch.cos(20 * x3) + 2 * torch.sin(x1 * x2) * torch.sin(x4 * x5)
        f3 = 10 * torch.exp(-25 * x4**2) * x3 - 0.1 * x3**3
        f4 = 2 * torch.sin(15 * (x1 * x5 - x2 * x3))
        f5 = -x1 * x5 + 5 * torch.tanh(20 * (x2 - x4))
        f6 = torch.tensor(0.0)

        return torch.stack ([f1, f2, f3, f4, f5, f6])
    
    @staticmethod
    def control_effectiveness ():
        return torch.eye(6)
    
    @staticmethod
    def desired_trajectory(t):
        height = 2.5  # meters
        omega = 0.15  # rad/s
        r = 2.5

        z_tilt = 0.0 
        a = 7.5  # Half-width of the long side (x-direction)
        b = 3.0  # Half-width of the short side (y-direction)
        
        # Position (figure 8 with major axis along x)
        xd1 = a * torch.sin(omega * t)
        xd2 = b * torch.sin(2.0 * omega * t)
        xd3 = torch.tensor(height, dtype=torch.float32) + (z_tilt / 2) * torch.sin(omega * t)
        # Velocity
        xd4 = a * omega * torch.cos(omega * t)
        xd5 = 2 * b * omega * torch.cos(2.0 * omega * t)
        xd6 = (z_tilt / 2) * omega * torch.cos(omega * t)

        xd4_dot = -a * omega**2 * torch.sin(omega * t)
        xd5_dot = -4 * b * omega**2 * torch.sin(2.0 * omega * t)
        xd6_dot = -(z_tilt / 2) * omega**2 * torch.sin(omega * t)

        xd = torch.stack([xd1, xd2, xd3, xd4, xd5, xd6])
        xd_dot = torch.stack([xd4, xd5, xd6, xd4_dot, xd5_dot, xd6_dot])
        
        return xd, xd_dot

    #=============================== Lb-DNN Architecture ===============================#
class LbDNN_arch(nn.Module):
    def __init__(self, n_inputs, n_hidden, n_outputs, num_layers=3):
        super().__init__()
        layers = []

        # input layer
        layers.append(nn.Linear(n_inputs, n_hidden))
        layers.append(nn.Tanh())
        
        for _ in range(num_layers - 2):
            layers.append(nn.Linear(n_hidden, n_hidden))
            layers.append(nn.ReLU())
        layers.append(nn.Linear(n_hidden, n_outputs))
        self.model = nn.Sequential(*layers)
        self._initialize_weights()
    
    def _initialize_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.xavier_uniform_(m.weight, gain=0.01)
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)
    
    def forward(self, x):
        return self.model(x)
    
    def count_parameters(self):
        return sum(p.numel() for p in self.parameters() if p.requires_grad)


# ==================== Lyapunov-Based DNN Controller ======================== #    

class LbDNN_Controller:
    def __init__(self, config):
        self.config = config
        self.n_states = config['n_states']
        self.n_inputs = self.n_states # Standardized to n_inputs
        self.n_hidden = config.get('n_neurons', 64)
        self.num_layers = config.get('num_layers', 3)
        self.gamma = config.get('gamma', 1.0)

        self.ke = config['ke']
        self.learning_rate = config.get('learning_rate')
        self.kT = config.get('kT')
        self.forgetting_factor = config.get('forgetting_factor')
        self.dt = config['dt']
        self.theta_bar = config['theta_bar']

        # Fix: Ensure this matches self.n_inputs
        self.nn = LbDNN_arch(self.n_inputs, self.n_hidden, self.n_states, self.num_layers)

        self.n_parameters = self.nn.count_parameters()
        self.Gamma = torch.eye(self.n_parameters) * self.gamma
        
    def projection_operator(self, update, weights):
        """ Projects the update to keep weights within theta_bar """
        weight_norm = torch.norm(weights)
        if weight_norm <= self.theta_bar:
            return update
        else:
            # Formula: update - projection_factor * (dot(update, weights) / norm^2) * weights
            projection_factor = 1.0 - (self.theta_bar / weight_norm)
            return update - projection_factor * (torch.dot(update, weights) / weight_norm**2) * weights

    @torch.enable_grad()
    def compute_jacobian(self, nn_input):
        for param in self.nn.parameters():
            param.requires_grad = True
        
        self.n_parameters = self.nn.count_parameters()
        jacobian = torch.zeros(self.n_states, self.n_parameters)

        for i in range(self.n_states):
            self.nn.zero_grad()
            Phi = self.nn(nn_input).squeeze()
            Phi_i = Phi[i]
            Phi_i.backward(retain_graph=True)
            
            grads = []
            for param in self.nn.parameters():
                grads.append(param.grad.view(-1))
            
            jacobian[i, :] = torch.cat(grads)
        return jacobian

    def parameter_adaptation(self, x, t):
        xd, xd_dot = Dynamics.desired_trajectory(torch.tensor(t, dtype=torch.float32))
        e = x - xd
        nn_input = x.unsqueeze(0)

        with torch.enable_grad():
            Phi = self.nn(nn_input).squeeze(0)
            jacobian = self.compute_jacobian(nn_input)

        theta = torch.cat([p.view(-1) for p in self.nn.parameters()])

        mu = 2.0 * e
        drift_vec = self.learning_rate * (jacobian.T @ e - self.forgetting_factor * theta)
        
        diffusion_scale = self.learning_rate * torch.sqrt(self.kT * torch.abs(torch.dot(e, mu)))
        dw = torch.randn(self.n_parameters) * math.sqrt(self.dt)
        diffusion_vec = diffusion_scale * dw
        drift_proj = self.projection_operator(drift_vec, theta)
        diff_proj = self.projection_operator(diffusion_vec, theta)
        
        # 5. Update Weights
        new_theta = theta + drift_proj * self.dt + diff_proj

        idx = 0
        with torch.no_grad():
            for param in self.nn.parameters():
                param_size = param.numel()
                param.data = new_theta[idx:idx+param_size].view_as(param)
                idx += param_size
            
        g1 = Dynamics.control_effectiveness()
        g1_inv = torch.inverse(g1) 
        u = g1_inv @ (xd_dot - self.ke * e - Phi - 0.5 * (self.n_parameters + 1) * self.gamma * self.kT * mu)
        
        return u.detach(), Phi.detach()

# ================= Simulation =================== #
class Simulation:
    def __init__(self, config_path='config_LyLA.json'):
        with open(config_path, 'r') as f:
            self.config = json.load(f)

        self.n_states = self.config['n_states']
        self.dt = self.config['dt']
        self.T_final = self.config['T_final']
        self.time_steps = int(self.T_final / self.dt)

        self.x = torch.zeros((self.n_states, self.time_steps))
        # Fix: Added a 6th element to match n_states=6
        self.x[:, 0] = torch.tensor([0, -1, 3, -3, 3, 0], dtype=torch.float32)

    def update_state(self, step, u, t):
        x_current = self.x[:, step - 1]
        f = Dynamics.drift_vector(x_current)
        g = Dynamics.control_effectiveness()
        
        self.x[:, step] = self.x[:, step - 1] + (f + g @ u) * self.dt 

    def run(self): 
        controller = LbDNN_Controller(self.config)
        tracking_errors = []
        control_inputs = []
        parameter_norms = []
        
        for step in range(1, self.time_steps):
            t = step * self.dt

            if step % 100 == 0:
                progress = 100 * step / self.time_steps
                # Fix: Changed controller.transformer to controller.nn
                theta = torch.cat([p.view(-1) for p in controller.nn.parameters()])
                param_norm = torch.norm(theta).item()
                parameter_norms.append(param_norm) 
                print(f"\rProgress: {progress:.1f}% | ||θ||: {param_norm:.2f}", end="", flush=True)

            x = self.x[:, step - 1]
            xd, xd_dot = Dynamics.desired_trajectory(torch.tensor(t, dtype=torch.float32))
            u, Phi = controller.parameter_adaptation(x, t)
            self.update_state(step, u, t)

            e = self.x[:, step] - xd
            error_norm = torch.norm(e).item()
            tracking_errors.append(error_norm)
            control_inputs.append(torch.norm(u).item())

        print(f"\n\nSimulation complete!")
        print(f"RMS tracking error: {np.sqrt(np.mean(np.square(tracking_errors))):.6f}")  
        print(f"RMS control input: {np.sqrt(np.mean(np.square(control_inputs))):.6f}") 

        self.results = {
            'x': self.x,
            'tracking_errors': tracking_errors,
            'control_inputs': control_inputs,
            'parameter_norms': parameter_norms,
            'time': np.arange(1, self.time_steps) * self.dt
        }

        return self.results
        
    def save_results(self, filename='LyLA_results.json'):
        results_dict = {
            'tracking_errors': [float(e) for e in self.results['tracking_errors']],
            'control_inputs': [float(u) for u in self.results['control_inputs']],
            'time': [float(t) for t in self.results['time']],
            'rms_error': float(np.sqrt(np.mean(np.square(self.results['tracking_errors']))))
        }
        
        with open(filename, 'w') as f:
            json.dump(results_dict, f, indent=4)
            
        print(f"\nResults saved to {filename}")

    def plot_results(self):
        import matplotlib.pyplot as plt
    
        time = self.results['time']
        tracking_errors = self.results['tracking_errors']
        
        plt.figure(figsize=(10, 6))
        plt.plot(time, tracking_errors, 'b-', linewidth=2)
        plt.xlabel('Time (s)', fontsize=14)
        plt.ylabel('Tracking Error ||e(t)||', fontsize=14)
        plt.title('LyLA-Therm Controller: Tracking Error', fontsize=16, fontweight='bold')
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig('tracking_error.png', dpi=300)
        print("\nPlot saved as 'tracking_error.png'")
        plt.show()


# ================= Main =================== #
def main ():
    torch.manual_seed(42)
    np.random.seed(42)
    with open('config_LyLA.json', 'r') as config_file: 
        config = json.load(config_file)
    # Save config
    with open('config_LyLA.json', 'w') as f:
        json.dump(config, f, indent=4)
    print("Config file created: config_LyLA.json")
    
    # Run simulation
    sim = Simulation('config_LyLA.json')
    results = sim.run()
    
    # Save results
    sim.save_results('LyLA_results.json')

    sim.plot_results()
    
    print("\n" + "="*70)
    print("SIMULATION FINISHED!")
    print("="*70)


if __name__ == "__main__":
    main()
