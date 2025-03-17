import os
import pandas as pd
import matplotlib.pyplot as plt
from tkinter import Tk, Checkbutton, Button
from tkinter import BooleanVar
import matplotlib.cm as cm
import re  # Import regex module to extract numbers from filenames

def plot_selected_columns(filenames, dt, selected_columns):
    # Create a figure for plotting
    fig, axs = plt.subplots(len(selected_columns), 1, figsize=(8, len(selected_columns) * 3))

    # If there is only one selected plot, axs is a single axis, so make it iterable
    if len(selected_columns) == 1:
        axs = [axs]

    # Generate a colormap (e.g., 'plasma') for progressively lighter to darker colors
    colormap = cm.Reds  # You can change this to other colormaps like 'inferno', 'cividis', etc.
    num_files = len(filenames)
    
    # Generate a list of colors for the files, progressing from light to dark
    colors = [colormap(i / num_files) for i in range(num_files)]

    # Map each file to a specific color
    file_colors = {filenames[i]: colors[i] for i in range(num_files)}

    # Loop through the selected plot types
    for i, label in enumerate(selected_columns):
        ax = axs[i]
        
        # Plot each CSV file's data on the same axis
        for filename in filenames:
            df = pd.read_csv(filename)
            time = df.index * dt  # Multiply the index by dt to get time in seconds

            # Get the color assigned to this file
            color = file_colors[filename]
            file_label = os.path.basename(filename)  # Use the filename for the label
            
            # Plot according to the selected label
            if label == 'x_ref vs x':
                ax.plot(time, df['x_ref'], label=f'{file_label} - x_ref', color=color)
                ax.plot(time, df['x'], label=f'{file_label} - x', linestyle='--', color=color)
                ax.set_ylabel('Position')
            elif label == 'x_dot':
                ax.plot(time, df['x_dot'], label=f'{file_label} - x_dot', color=color)
                ax.set_ylabel('Velocity')
            elif label == 'x_ddot':
                ax.plot(time, df['x_ddot'], label=f'{file_label} - x_ddot', color=color)
                ax.set_ylabel('Acceleration')
            elif label == 'theta':
                ax.plot(time, df['theta'], label=f'{file_label} - theta', color=color)
                ax.set_ylabel('Angle (radians)')
            elif label == 'theta_dot':
                ax.plot(time, df['theta_dot'], label=f'{file_label} - theta_dot', color=color)
                ax.set_ylabel('Angular Velocity')
            elif label == 'theta_ddot':
                ax.plot(time, df['theta_ddot'], label=f'{file_label} - theta_ddot', color=color)
                ax.set_ylabel('Angular Acceleration')
            elif label == 'F':
                ax.plot(time, df['F'], label=f'{file_label} - Force', color=color)
                ax.set_ylabel('Force')
            elif label == 'control_input':
                ax.plot(time, df['control_input'], label=f'{file_label} - Control Input', color=color)
                ax.set_ylabel('Control Input')
            elif label == 'random_force':
                ax.plot(time, df['random_force'], label=f'{file_label} - Random Force', color=color)
                ax.set_ylabel('Random Force')
            elif label == 'score':
                ax.plot(time, df['score'], label=f'{file_label} - Score', color=color)
                ax.set_ylabel('Score')
            elif label == 'time_left':
                ax.plot(time, df['time_left'], label=f'{file_label} - Time Left', color=color)
                ax.set_ylabel('Time Left')

        # Set the x-axis label for the last plot only
        if i == len(selected_columns) - 1:
            ax.set_xlabel('Time (seconds)')
        
        # Set legend and move it outside the plot area
        ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1), borderaxespad=0.)

        # Remove the x-axis labels and ticks for all but the bottom-most plot
        if i != len(selected_columns) - 1:
            ax.set_xticklabels([])  # Remove x-axis labels
            ax.set_xticks([])       # Remove x-axis ticks

    # Adjust layout to prevent title overlap
    plt.tight_layout()  # Ensures everything fits within the figure area
    plt.subplots_adjust(hspace=0.1, bottom=0.05, top=0.975, left=0.05, right=0.95)  # Adjust space between subplots

    # Show the plot
    plt.show()

def on_confirm_click(selected_labels, selected_files):
    # Show selected columns and plot them from the selected files
    if selected_files:
        plot_selected_columns(selected_files, dt, selected_labels)

def show_checkboxes():
    # Create a new Tkinter window
    window = Tk()
    window.title("Select Plots and CSV Files to Display")

    selected_labels = []
    selected_files = []

    # Specify the path to your folder containing the .csv files
    folder_path = 'saved_states'  # Modify this to the correct folder path

    # Get the list of .csv files in the specified folder and sort them numerically
    csv_files = sorted([f for f in os.listdir(folder_path) if f.endswith('.csv')],
                       key=lambda x: int(re.search(r'(\d+)', x).group()))  # Sort by number in filename

    if not csv_files:
        print("No CSV files found in the specified folder.")
        return

    # Create checkboxes for each CSV file
    file_checkboxes = {}
    file_vars = {}
    for file in csv_files:
        var = BooleanVar()
        checkbox = Checkbutton(window, text=file, variable=var, command=lambda f=file, v=var: file_selected(f, v, selected_files))
        checkbox.pack()
        file_vars[file] = var

    # Create checkboxes for the plots
    checkboxes = {
        'x_ref vs x': BooleanVar(),
        'x_dot': BooleanVar(),
        'x_ddot': BooleanVar(),
        'theta': BooleanVar(),
        'theta_dot': BooleanVar(),
        'theta_ddot': BooleanVar(),
        'F': BooleanVar(),
        'control_input': BooleanVar(),
        'random_force': BooleanVar(),
        'score': BooleanVar(),
        'time_left': BooleanVar()
    }

    # Function to handle checkbox state change for selected columns
    def checkbox_callback():
        selected_labels.clear()
        for label, var in checkboxes.items():
            if var.get():  # If checkbox is selected
                selected_labels.append(label)

    # Add checkboxes for the plots to the window
    for label, var in checkboxes.items():
        checkbox = Checkbutton(window, text=label, variable=var, command=checkbox_callback)
        checkbox.pack()

    # Function to update selected files based on checkbox state
    def file_selected(file, var, selected_files):
        if var.get():  # If checkbox is selected
            selected_files.append(os.path.join(folder_path, file))  # Save full path to file
        else:
            # If checkbox is deselected, remove file from selected_files
            selected_files.remove(os.path.join(folder_path, file))

    # Add the "Confirm" button to confirm selection
    confirm_button = Button(window, text="Confirm", command=lambda: on_confirm_click(selected_labels, selected_files))
    confirm_button.pack()

    # Start the Tkinter event loop
    window.mainloop()

# Specify the dt value
dt = 1 / 200  # Given dt value

# Show the checkboxes window
show_checkboxes()
