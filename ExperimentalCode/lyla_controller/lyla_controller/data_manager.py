import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl

# Ensure output directory exists
def ensure_directory_exists(directory: str):
    os.makedirs(directory, exist_ok=True)

# Save agent and target states to CSV
def save_state_to_csv(step, time, agent_position, target_position, control_input):
    ensure_directory_exists('src/lyla_controller/simulation_data')

    tracking_error_norm = np.linalg.norm(target_position - agent_position)

    state_data = pd.DataFrame({
        'Step': [step],
        'Time': [time],
        'Position_X': [agent_position[0]],
        'Position_Y': [agent_position[1]],
        'Position_Z': [agent_position[2]],
        'Tracking_Error_Norm': [tracking_error_norm],
        'Control_Input_X': [control_input[0]],
        'Control_Input_Y': [control_input[1]],
        'Control_Input_Z': [control_input[2]],
    })

    # Construct the file path using the new ID
    state_file_path = f'src/lyla_controller/simulation_data/state_data.csv'

    if step == 1:
        # If file exists, remove it so a fresh CSV is created
        if os.path.exists(state_file_path):
            os.remove(state_file_path)
        # Write header on the first write
        state_data.to_csv(state_file_path, index=False, header=True)
    else:
        # Append subsequent data
        state_data.to_csv(state_file_path, mode='a', header=False, index=False)

    target_state_data = pd.DataFrame({
        'Time': [time],
        'Position_X': [target_position[0]],
        'Position_Y': [target_position[1]],
        'Position_Z': [target_position[2]],
    })

    target_file_path = 'src/lyla_controller/simulation_data/target_state_data.csv'
    if step == 1:
        if os.path.exists(target_file_path):
            os.remove(target_file_path)
        target_state_data.to_csv(target_file_path, index=False, header=True)
    else:
        target_state_data.to_csv(target_file_path, mode='a', header=False, index=False)

def save_theta_to_csv(step, time, theta):
    ensure_directory_exists('src/lyla_controller/simulation_data')

    state_data = pd.DataFrame({
        'Step': [step],
        'Time': [time],
        **{f'Theta{i+1}': [theta[i]] for i in range(len(theta))}
    })

    # Construct the file path using the new ID
    state_file_path = f'src/lyla_controller/simulation_data/theta_data.csv'

    if step == 1:
        # If file exists, remove it so a fresh CSV is created
        if os.path.exists(state_file_path):
            os.remove(state_file_path)
        # Write header on the first write
        state_data.to_csv(state_file_path, index=False, header=True)
    else:
        # Append subsequent data
        state_data.to_csv(state_file_path, mode='a', header=False, index=False)


# Constants for IEEE standard plotting
IEEE_FIGSIZE = (10, 8)
IEEE_FONTSIZE = 10
IEEE_LINEWIDTH = 1.5
IEEE_GRID_STYLE = {'linestyle': '--', 'linewidth': 0.5, 'color': 'gray'}

mpl.rcParams['savefig.format'] = 'eps'

def plot_from_csv():
    # Read state data and target state data
    state_data = pd.read_csv('src/lyla_controller/simulation_data/state_data.csv')
    target_state_data = pd.read_csv('src/lyla_controller/simulation_data/target_state_data.csv')
    time_array = target_state_data['Time']
    theta_data = pd.read_csv('src/lyla_controller/simulation_data/theta_data.csv')
    
    # Read nn_state_data file
    #_________________________________________________________________________________________________________________________________ 
    # Plot tracking error over time
    plt.figure(figsize=IEEE_FIGSIZE)
    tracking_error_norm = state_data['Tracking_Error_Norm']
    rms_tracking_error = np.sqrt(np.mean(tracking_error_norm**2))
    plt.plot(time_array.to_numpy(), tracking_error_norm.to_numpy(), label=f'Agent: RMS {rms_tracking_error:.4f} m')
    print(f'Mean RMS Tracking Error: {rms_tracking_error} m')
    plt.xlabel('Time (s)')
    plt.ylabel('Tracking Error Norm $(m)$')
    plt.legend(loc='best', fontsize=IEEE_FONTSIZE, frameon=True)
    plt.grid(**IEEE_GRID_STYLE)
    plt.tight_layout()
    #_________________________________________________________________________________________________________________________________ 
    # Plot tracking error over time
    plt.figure(figsize=IEEE_FIGSIZE)
    weight_columns = [col for col in theta_data.columns if col.startswith('Theta')]
    for col in weight_columns:
        plt.plot(time_array.to_numpy(), theta_data[col].to_numpy(), label=col)

    plt.xlabel('Time (s)')
    plt.ylabel('theta')
    plt.legend(loc='best', fontsize=IEEE_FONTSIZE, frameon=True)
    plt.grid(**IEEE_GRID_STYLE)
    plt.tight_layout()
    #_________________________________________________________________________________________________________________________________ 
    # Plot control input over time
    Control_input_X = state_data['Control_Input_X']
    Control_input_Y = state_data['Control_Input_Y']
    Control_input_Z = state_data['Control_Input_Z']
    plt.figure(figsize=IEEE_FIGSIZE)
    plt.plot(time_array.to_numpy(), Control_input_X.to_numpy(), label='X Control Input', linewidth=1.5)
    plt.plot(time_array.to_numpy(), Control_input_Y.to_numpy(), label='Y Control Input', linewidth=1.5)
    plt.plot(time_array.to_numpy(), Control_input_Z.to_numpy(), label='Z Control Input', linewidth=1.5)
    plt.xlabel('Time (s)')
    plt.ylabel('Control Input $(m/s^2)$')
    plt.legend(loc='best', fontsize=IEEE_FONTSIZE, frameon=True)
    plt.grid(**IEEE_GRID_STYLE)
    plt.tight_layout()
    #_________________________________________________________________________________________________________________________________     
    # 3D Trajectories
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(state_data['Position_X'].to_numpy(), state_data['Position_Y'].to_numpy(), state_data['Position_Z'].to_numpy(), label='Agent')
    ax.plot(target_state_data['Position_X'].to_numpy(), target_state_data['Position_Y'].to_numpy(), target_state_data['Position_Z'].to_numpy(), label='Target', linestyle='--')
    ax.scatter(state_data['Position_X'].iloc[0], state_data['Position_Y'].iloc[0], state_data['Position_Z'].iloc[0], marker='x', color='blue', s=100, label='Agent Start')
    ax.scatter(state_data['Position_X'].iloc[-1], state_data['Position_Y'].iloc[-1], state_data['Position_Z'].iloc[-1], marker='o', color='blue', s=100, label='Agent End')
    ax.scatter(target_state_data['Position_X'].iloc[0], target_state_data['Position_Y'].iloc[0], target_state_data['Position_Z'].iloc[0], marker='x', color='orange', s=100, label='Target Start')
    ax.scatter(target_state_data['Position_X'].iloc[-1], target_state_data['Position_Y'].iloc[-1], target_state_data['Position_Z'].iloc[-1], marker='o', color='orange', s=100, label='Target End')
    x_min, x_max, y_min, y_max, z_min, z_max = -30, 30, -6, 6, 0, 5
    corners = [
        [x_min, y_min, z_min], [x_max, y_min, z_min], [x_max, y_max, z_min], [x_min, y_max, z_min],
        [x_min, y_min, z_max], [x_max, y_min, z_max], [x_max, y_max, z_max], [x_min, y_max, z_max]
    ]
    edges = [
        [0, 1], [1, 2], [2, 3], [3, 0],
        [4, 5], [5, 6], [6, 7], [7, 4],
        [0, 4], [1, 5], [2, 6], [3, 7]
    ]
    for edge in edges:
        ax.plot([corners[edge[0]][0], corners[edge[1]][0]],
                [corners[edge[0]][1], corners[edge[1]][1]],
                [corners[edge[0]][2], corners[edge[1]][2]],
                'r-', linewidth=2)
    ax.set_xlabel('X Position $(m)$')
    ax.set_ylabel('Y Position $(m)$')
    ax.set_zlabel('Z Position $(m)$')
    ax.set_xlim([x_min-5, x_max+5])
    ax.set_ylim([y_min-5, y_max+5])
    ax.set_zlim([z_min-2, z_max+2])
    ax.view_init(elev=30, azim=45)
    ax.legend()
    ax.grid(True)
    plt.title('3D Trajectory with Bounding Box')
    plt.tight_layout()

    plt.show()

def results():
    plot_from_csv()