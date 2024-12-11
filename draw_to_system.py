import tkinter as tk
from tkinter import messagebox

class LineDrawer:
    """
    A simple application for drawing lines by clicking points on a canvas.
    The application converts canvas pixel coordinates to normalized coordinates
    within a defined range and stores them.
    """

    def __init__(self, master):
        """
        Initialize the LineDrawer application.

        Args:
            master: The Tkinter root window.
        """
        self.master = master
        self.master.title("Line Drawer")

        # Canvas dimensions (smaller size)
        self.canvas_width = 500
        self.canvas_height = 500
        self.canvas = tk.Canvas(master, width=self.canvas_width, height=self.canvas_height, bg="white")
        self.canvas.pack(pady=10)

        # Coordinate system bounds (normalized ranges)
        self.x_min, self.x_max = -0.4, 0.4
        self.y_min, self.y_max = 0.2, 1.0

        # Initialize variables for drawing
        self.previous_point = None  # Stores the last clicked point for line drawing
        self.coordinates = []  # Stores clicked points in normalized form

        # Bind mouse click to canvas
        self.canvas.bind("<Button-1>", self.on_click)

        # Add a button to display the coordinates
        self.show_button = tk.Button(master, text="Show Coordinates", command=self.show_coordinates)
        self.show_button.pack(pady=10)  # Add padding to ensure spacing

    def on_click(self, event):
        """
        Handle canvas click events to draw lines and save coordinates.

        Args:
            event: The Tkinter event object containing click details.
        """
        # Convert pixel coordinates to normalized coordinates
        x, y = self.pixel_to_coord(event.x, event.y)

        # Round the coordinates to the nearest hundredth for precision
        x_rounded, y_rounded = round(x, 2), round(y, 2)

        # Save the clicked point
        self.coordinates.append((x_rounded, y_rounded))

        # Draw a small oval on the canvas to mark the clicked point
        self.canvas.create_oval(
            event.x - 3, event.y - 3, event.x + 3, event.y + 3, fill="blue"
        )

        # Draw a line from the previous point to the current point, if applicable
        if self.previous_point:
            prev_x, prev_y = self.coord_to_pixel(*self.previous_point)
            self.canvas.create_line(prev_x, prev_y, event.x, event.y, fill="black")

        # Update the previous point for the next click
        self.previous_point = (x, y)

    def pixel_to_coord(self, x, y):
        """
        Convert canvas pixel coordinates to normalized coordinates.

        Args:
            x, y: Pixel coordinates on the canvas.

        Returns:
            tuple: Normalized coordinates (x, y).
        """
        coord_x = self.x_min + (x / self.canvas_width) * (self.x_max - self.x_min)
        coord_y = self.y_max - (y / self.canvas_height) * (self.y_max - self.y_min)
        return coord_x, coord_y

    def coord_to_pixel(self, x, y):
        """
        Convert normalized coordinates to canvas pixel coordinates.

        Args:
            x, y: Normalized coordinates.

        Returns:
            tuple: Pixel coordinates (x, y).
        """
        pixel_x = int((x - self.x_min) / (self.x_max - self.x_min) * self.canvas_width)
        pixel_y = int((self.y_max - y) / (self.y_max - self.y_min) * self.canvas_height)
        return pixel_x, pixel_y

    def show_coordinates(self):
        """
        Display the stored coordinates in a message box and print them to the console.
        """
        messagebox.showinfo("Coordinates", f"Clicked Coordinates:\n{self.coordinates}")
        print("Clicked Coordinates:", self.coordinates)

    def get_coordinates(self):
        """
        Return the stored coordinates for external use.
        """
        return self.coordinates


def main():
    """
    Create a Tkinter application to collect coordinates and return them.
    """
    root = tk.Tk()
    app = LineDrawer(root)
    root.mainloop()
    return app.get_coordinates()


if __name__ == "__main__":
    # Run the script independently
    coordinates = main()
    print("Final Coordinates:", coordinates)
