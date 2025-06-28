from PIL import Image, ImageTk
import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class MapGUI(Node):
    def __init__(self):
        super().__init__('map_gui')
        self.pose_sub = self.create_subscription(Pose, '/car_sim_pose', self.pose_callback, 10)
        self.x = 0.0
        self.y = 0.0

        # --- GUI setup ---
        self.root = tk.Tk()
        self.root.title('Car Virtual Map')

        # load map image
        map_img = Image.open(r"ACC_Full_Node_Visual.png")
        self.map_width, self.map_height = map_img.size

        # optionally resize to fit a square window (uncomment if desired)
        # map_img = map_img.resize((500,500), Image.ANTIALIAS)
        # self.map_width, self.map_height = 500,500

        self.tk_map = ImageTk.PhotoImage(map_img)

        self.canvas = tk.Canvas(self.root,
                                width=self.map_width,
                                height=self.map_height)
        self.canvas.pack()
        # draw the map
        self.canvas.create_image(0, 0, anchor='nw', image=self.tk_map)

        # car marker
        self.car_radius = 5
        self.car_dot = self.canvas.create_oval(0,0,0,0, fill='red')

        # how many meters per pixel? tune to your map scale
        self.resolution = 0.05  # e.g. 5 cm/pixel

        # start ROS spinning in background
        t = threading.Thread(target=self._ros_spin, daemon=True)
        t.start()

        # schedule periodic redraw
        self._update_canvas()
        self.root.mainloop()

    def pose_callback(self, msg: Pose):
        self.x = msg.position.x
        self.y = msg.position.y

    def _update_canvas(self):
        # translate (x,y) meters → pixel coords on image
        px = self.x / self.resolution
        py = self.map_height - (self.y / self.resolution)
        self.canvas.coords(
            self.car_dot,
            px-self.car_radius, py-self.car_radius,
            px+self.car_radius, py+self.car_radius
        )
        self.root.after(50, self._update_canvas)

    def _ros_spin(self):
        rclpy.spin(self)

def main():
    rclpy.init()
    MapGUI()
    rclpy.shutdown()

if __name__ == '__main__':
    main()