import ezdxf
import matplotlib.pyplot as plt

def extract_connected_coordinates(file_path):
    """
    Extracts connected coordinates from a DXF file to form a continuous path for drawing.
    This function ensures that the path is constructed in the order entities are defined
    in the DXF file and handles different types of geometry.

    Args:
        file_path (str): Path to the DXF file.

    Returns:
        list: A list of tuples containing coordinates (x, y) in the drawing order.
    """
    try:
        # Load the DXF file and access the model space
        doc = ezdxf.readfile(file_path)
        msp = doc.modelspace()

        # Initialize the list to store the drawing path
        drawing_path = []

        # Iterate through each entity in the DXF file
        for entity in msp:
            # Handle LINE entities (straight connections between two points)
            if entity.dxftype() == "LINE":
                start = (entity.dxf.start.x, entity.dxf.start.y)
                end = (entity.dxf.end.x, entity.dxf.end.y)
                if not drawing_path or drawing_path[-1] != start:
                    drawing_path.append(start)  # Ensure continuity if disconnected
                drawing_path.append(end)

            # Handle POLYLINE and LWPOLYLINE entities (multi-segment lines)
            elif entity.dxftype() in ["LWPOLYLINE", "POLYLINE"]:
                points = entity.get_points()
                for point in points:
                    xy = (point[0], point[1])  # Extract (x, y), ignoring z
                    if not drawing_path or drawing_path[-1] != xy:
                        drawing_path.append(xy)

        print(f"Generated drawing path with {len(drawing_path)} points.")
        return drawing_path

    except Exception as e:
        print(f"Error reading DXF file: {e}")
        return []

def rescale_coordinates(path, x_range, y_range):
    """
    Rescales the extracted coordinates to fit within specified ranges for X and Y.
    The rescaling is linear, ensuring that the shape is preserved.

    Args:
        path (list): List of tuples (x, y).
        x_range (tuple): Target range for X as (min, max).
        y_range (tuple): Target range for Y as (min, max).

    Returns:
        list: Rescaled coordinates, rounded to the nearest hundredth.
    """
    # Define target ranges for X and Y
    x_min, x_max = x_range
    y_min, y_max = y_range

    # Determine the original min and max values for X and Y
    x_coords = [coord[0] for coord in path]
    y_coords = [coord[1] for coord in path]

    x_orig_min, x_orig_max = min(x_coords), max(x_coords)
    y_orig_min, y_orig_max = min(y_coords), max(y_coords)

    # Rescale each point to the target range
    rescaled = [
        (
            round(x_min + (x - x_orig_min) * (x_max - x_min) / (x_orig_max - x_orig_min), 2),
            round(y_min + (y - y_orig_min) * (y_max - y_min) / (y_orig_max - y_orig_min), 2)
        )
        for x, y in path
    ]

    print("Coordinates rescaled successfully!")
    return rescaled

def visualize_path(path):
    """
    Visualizes the drawing path as a 2D plot.
    The path simulates how the robot arm would draw the image.

    Args:
        path (list): List of tuples containing (x, y) coordinates in drawing order.
    """
    if not path:
        print("No path to visualize.")
        return

    # Extract X and Y coordinates for plotting
    x_coords, y_coords = zip(*path)

    # Create a 2D plot to show the drawing path
    plt.figure(figsize=(8, 6))
    plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='blue', alpha=0.7)
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.title("Robot Arm Drawing Path")
    plt.grid(True)
    plt.show()

def main():
    """
    Load a DXF file, process it, and return the final coordinates.

    Returns:
        list: List of final rescaled (x, y) coordinates.
    """
    dxf_file = r"C:\Users\rhk03\Downloads\final\ex.dxf"  # Replace with your DXF file path

    # Step 1: Extract connected coordinates
    drawing_path = extract_connected_coordinates(dxf_file)

    # Step 2: Rescale coordinates to the desired range
    x_target_range = (-0.4, 0.4)  # Target X range
    y_target_range = (0.2, 1.0)   # Target Y range
    rescaled_path = rescale_coordinates(drawing_path, x_target_range, y_target_range)

    # Step 3: Return the final rescaled coordinates
    return rescaled_path

if __name__ == "__main__":
    final_coords = main()
    print("Final Rescaled Coordinates (Rounded):")
    print(final_coords)

    # Visualize the final drawing path
    visualize_path(final_coords)
