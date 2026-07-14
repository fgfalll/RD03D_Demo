import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Wedge
import tkinter as tk
from tkinter import filedialog
import sys
import numpy as np

# --- Configuration for Radar's Field of View (Corrected per datasheet) ---
MAX_RANGE_MM = 8000
FOV_ANGLE_DEG = 60  # Corrected to +/- 60 degrees

def load_and_process_data(filepath):
    """
    Loads radar data and adds calculated columns for distance, angle, and FOV status.
    """
    try:
        print(f"Attempting to load data from: {filepath}")
        df = pd.read_csv(filepath, skipinitialspace=True)
        
        required_columns = ['Timestamp', 'TargetID', 'X_mm', 'Y_mm', 'IsConfirmedHuman']
        if not all(col in df.columns for col in required_columns):
            print(f"Error: CSV file must contain the columns: {required_columns}")
            print(f"Columns found: {df.columns.tolist()}")
            return None

        print("CSV loaded. Calculating distance and angle for each point...")
        
        df['Calculated_Distance'] = np.sqrt(df['X_mm']**2 + df['Y_mm']**2)
        df['Angle_Deg'] = np.degrees(np.arctan2(df['X_mm'], df['Y_mm']))
        df['InRange'] = (df['Calculated_Distance'] <= MAX_RANGE_MM) & \
                        (df['Angle_Deg'].abs() <= FOV_ANGLE_DEG)

        print("Data processing complete.")
        print("Data preview:")
        print(df.head())
        return df
        
    except Exception as e:
        print(f"An error occurred while reading the file: {e}")
        return None

def draw_fov(ax):
    """Helper function to draw the FOV cone on a plot."""
    fov_wedge = Wedge(center=(0, 0), r=MAX_RANGE_MM, 
                        theta1=90 - FOV_ANGLE_DEG, 
                        theta2=90 + FOV_ANGLE_DEG, 
                        facecolor='blue', alpha=0.1)
    ax.add_patch(fov_wedge)

def plot_static_paths(df):
    """
    Creates a static plot showing all paths, with different styles for confirmed humans.
    """
    fig, ax = plt.subplots(figsize=(12, 12))
    draw_fov(ax)
    
    df_in_range = df[df['InRange']]
    df_out_of_range = df[~df['InRange']]
    
    df_confirmed = df_in_range[df_in_range['IsConfirmedHuman'] == 1]
    df_unconfirmed = df_in_range[df_in_range['IsConfirmedHuman'] == 0]

    ax.plot(df_out_of_range['X_mm'], df_out_of_range['Y_mm'], 'rx', 
            markersize=5, alpha=0.4, label='Out-of-Range Detections')

    # Plot unconfirmed paths first with a subtle style
    for target_id in sorted(df_unconfirmed['TargetID'].unique()):
        target_df = df_unconfirmed[df_unconfirmed['TargetID'] == target_id]
        ax.plot(target_df['X_mm'], target_df['Y_mm'], 'o--', alpha=0.5, label=f'Unconfirmed Target {target_id}')
        
    # Plot confirmed human paths on top with a prominent style
    for target_id in sorted(df_confirmed['TargetID'].unique()):
        target_df = df_confirmed[df_confirmed['TargetID'] == target_id]
        ax.plot(target_df['X_mm'], target_df['Y_mm'], 'o-', linewidth=2.5, label=f'Confirmed Human {target_id}')

    ax.plot(0, 0, 'ko', markersize=12, label='Radar Position')
    ax.set_title('Complete Radar Target Paths')
    ax.set_xlabel('X Position (mm)')
    ax.set_ylabel('Y Position (mm)')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend()
    ax.set_aspect('equal', adjustable='box')
    fig.canvas.manager.set_window_title('Static Plot with Target Classification')

def plot_animated_tracking(df):
    """
    Creates an animated plot replaying all detections with different styles.
    """
    fig, ax = plt.subplots(figsize=(12, 12))
    timestamps = sorted(df['Timestamp'].unique())
    
    padding = 1000
    x_min, x_max = df['X_mm'].min() - padding, df['X_mm'].max() + padding
    y_min, y_max = df['Y_mm'].min() - padding, df['Y_mm'].max() + padding
    
    def update(frame):
        current_time_ms = timestamps[frame]
        current_time_s = current_time_ms / 1000.0 # Convert to seconds for display
        ax.clear()
        
        frame_df = df[df['Timestamp'] == current_time_ms]
        frame_in_range = frame_df[frame_df['InRange']]
        frame_out_of_range = frame_df[~frame_df['InRange']]
        
        frame_confirmed = frame_in_range[frame_in_range['IsConfirmedHuman'] == 1]
        frame_unconfirmed = frame_in_range[frame_in_range['IsConfirmedHuman'] == 0]

        draw_fov(ax)
        ax.plot(0, 0, 'ko', markersize=12)
        
        ax.scatter(frame_out_of_range['X_mm'], frame_out_of_range['Y_mm'], 
                   c='red', marker='x', s=50, alpha=0.7, label='Out-of-Range')
        
        if not frame_unconfirmed.empty:
            ax.scatter(frame_unconfirmed['X_mm'], frame_unconfirmed['Y_mm'], 
                       c='gray', s=100, alpha=0.6, label='Unconfirmed Target')
            for _, row in frame_unconfirmed.iterrows():
                ax.text(row['X_mm'] + 100, row['Y_mm'], f"T{row['TargetID']}", fontsize=12, color='gray')

        if not frame_confirmed.empty:
            ax.scatter(frame_confirmed['X_mm'], frame_confirmed['Y_mm'], 
                       c=frame_confirmed['TargetID'], cmap='viridis', s=200, alpha=1.0,
                       label='Confirmed Human')
            for _, row in frame_confirmed.iterrows():
                ax.text(row['X_mm'] + 100, row['Y_mm'], f"T{row['TargetID']}", fontsize=12, fontweight='bold')
                
        ax.set_title(f'Radar Tracking at Time: {current_time_s:.3f} s') # Formatted title
        ax.set_xlabel('X Position (mm)')
        ax.set_ylabel('Y Position (mm)')
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.set_aspect('equal', adjustable='box')
        ax.legend(loc='upper right')

    ani = animation.FuncAnimation(fig, update, frames=len(timestamps), interval=50, repeat=False)
    fig.canvas.manager.set_window_title('Animated Plot with Target Classification')
    return ani

def main():
    """Main function to run the script."""
    root = tk.Tk()
    root.withdraw()
    
    filepath = filedialog.askopenfilename(
        title="Select your Radar CSV log file",
        filetypes=(("CSV Files", "*.csv"), ("All files", "*.*"))
    )
    
    if not filepath:
        print("No file selected. Exiting.")
        sys.exit()
        
    radar_data = load_and_process_data(filepath)
    
    if radar_data is not None:
        plot_static_paths(radar_data)
        
        global anim
        anim = plot_animated_tracking(radar_data)
        
        print("\nDisplaying plots. Close the plot windows to exit.")
        plt.show()

if __name__ == "__main__":
    main()