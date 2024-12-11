import numpy as np
from svgpathtools import svg2paths
import matplotlib.pyplot as plt


class InteractivePlot:
    """
    A class to create an interactive plot where users can click to remove line segments.
    """

    def __init__(self, coords):
        """
        Initialize the interactive plot with given coordinates.

        Args:
            coords (list): List of (x, y) tuples representing the coordinates.
        """
        self.coords = coords  # Store the coordinates
        self.fig, self.ax = plt.subplots()  # Create the matplotlib figure and axis
        self.lines = []  # Store line segment objects for visualization

        self.draw_lines()  # Draw the initial plot
        self.cid = self.fig.canvas.mpl_connect("button_press_event", self.on_click)  # Connect click event

    def draw_lines(self):
        """Draw all line segments based on the current coordinates."""
        self.ax.clear()  # Clear any existing plot
        self.ax.set_aspect("equal", adjustable="box")  # Ensure equal scaling
        self.lines = []  # Reset the lines list
        for i in range(len(self.coords) - 1):
            # Draw a line between consecutive points
            line, = self.ax.plot(
                [self.coords[i][0], self.coords[i + 1][0]],
                [self.coords[i][1], self.coords[i + 1][1]],
                color="black"
            )
            self.lines.append(line)  # Add the line object to the list
        self.fig.canvas.draw()  # Update the figure

    def on_click(self, event):
        """
        Handle click events to remove the nearest line segment.

        Args:
            event: The click event object.
        """
        if event.inaxes != self.ax:  # Ignore clicks outside the plot area
            return

        # Find the closest line segment to the click point
        min_dist = float("inf")
        closest_idx = -1
        for i, line in enumerate(self.lines):
            x1, y1 = self.coords[i]
            x2, y2 = self.coords[i + 1]
            dist = self.point_to_segment_distance(event.xdata, event.ydata, x1, y1, x2, y2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Remove the closest line segment
        if closest_idx != -1:
            del self.coords[closest_idx + 1]  # Remove the endpoint of the segment
            self.draw_lines()  # Redraw the plot

    @staticmethod
    def point_to_segment_distance(px, py, x1, y1, x2, y2):
        """
        Calculate the shortest distance from a point to a line segment.

        Args:
            px, py: The point coordinates.
            x1, y1, x2, y2: The line segment endpoints.

        Returns:
            float: The shortest distance from the point to the segment.
        """
        # Vector math to calculate the projection
        line_vec = np.array([x2 - x1, y2 - y1])
        point_vec = np.array([px - x1, py - y1])
        line_len = np.linalg.norm(line_vec)
        line_unit_vec = line_vec / line_len  # Normalize the line vector
        proj_length = np.dot(point_vec, line_unit_vec)
        proj_vec = proj_length * line_unit_vec

        # Clamp projection to segment endpoints
        if proj_length < 0:
            proj_vec = np.array([0, 0])
        elif proj_length > line_len:
            proj_vec = line_vec

        # Calculate perpendicular distance
        closest_point = np.array([x1, y1]) + proj_vec
        return np.linalg.norm(np.array([px, py]) - closest_point)

    def show(self):
        """Display the interactive plot."""
        plt.show()


def normalize_coordinates(coords, x_range=(-0.4, 0.4), y_range=(0.2, 1.0)):
    """
    Normalize coordinates to fit within the specified ranges.

    Args:
        coords (list): List of (x, y) tuples.
        x_range, y_range (tuple): Target ranges for x and y axes.

    Returns:
        list: List of normalized (x, y) tuples.
    """
    x_coords, y_coords = zip(*coords)
    x_min, x_max = min(x_coords), max(x_coords)
    y_min, y_max = min(y_coords), max(y_coords)

    # Normalize each coordinate
    x_normalized = [
        (x - x_min) / (x_max - x_min) * (x_range[1] - x_range[0]) + x_range[0]
        for x in x_coords
    ]
    y_normalized = [
        (y - y_min) / (y_max - y_min) * (y_range[1] - y_range[0]) + y_range[0]
        for y in y_coords
    ]

    return list(zip(x_normalized, y_normalized))


def resample_coordinates(coords, num_points):
    """
    Resample coordinates to limit the number of points.

    Args:
        coords (list): List of (x, y) tuples.
        num_points (int): Target number of points.

    Returns:
        list: Resampled coordinates.
    """
    if len(coords) <= num_points:
        return coords  # Return original if no resampling is needed

    # Uniformly select points
    indices = np.linspace(0, len(coords) - 1, num_points, dtype=int)
    return [coords[i] for i in indices]


def convert_svg_to_coordinates(svg_file, num_points=1000):
    """
    Convert an SVG file into a continuous list of (x, y) coordinates.

    Args:
        svg_file (str): Path to the SVG file.
        num_points (int): Number of points per path segment.

    Returns:
        list: List of (x, y) coordinates.
    """
    paths, _ = svg2paths(svg_file)

    all_coords = []
    for path in paths:
        for segment in path:
            # Resample the segment into points
            points = [segment.point(t) for t in np.linspace(0, 1, num_points)]
            coords = [(p.real, p.imag) for p in points]
            all_coords.extend(coords)

    return all_coords


def main():
    """
    Load an SVG file, process it, and return the final coordinates after interactive editing.

    Returns:
        list: List of final (x, y) coordinates.
    """
    svg_file = r"C:\Users\rhk03\Downloads\final\ex.svg"  # Replace with your SVG file path
    initial_coords = convert_svg_to_coordinates(svg_file)

    # Normalize and resample the coordinates
    normalized_coords = normalize_coordinates(initial_coords)
    resample_to = 100  # Adjust the number of points
    resampled_coords = resample_coordinates(normalized_coords, num_points=resample_to)

    # Round coordinates to the nearest hundredth
    rounded_coords = [(round(x, 2), round(y, 2)) for x, y in resampled_coords]

    # Interactive plotting
    interactive_plot = InteractivePlot(rounded_coords)
    interactive_plot.show()

    # Return the final coordinates
    return interactive_plot.coords


if __name__ == "__main__":
    final_coords = main()
    print("Final Coordinates Array:")
    print(final_coords)
