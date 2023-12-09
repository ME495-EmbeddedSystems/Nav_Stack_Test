"""

Controls the movement of the Nubot in RVIZ to map a surrounding.

Publishers:
  + goal_pose PoseStamped - Sending goal pose to RVIZ
  
Service:
  + None

Subscriptions:
  + map OccupancyGrid - Getting the map information

Parameters
----------
  + None

"""


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped as RPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import tf2_ros
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from enum import Enum, auto

class State(Enum):
    """

    Current state of the system.

    Determines what the main timer function should be doing on each iteration.

    """

    BOTTOM = auto()
    LEFT = auto()
    TOP = auto()
    RIGHT = auto()
    DONE = auto()
 

class Explore(Node):
    """
    This node publishes a goal pose value for the nubot robot
    
    It moves to a wall and the follows a clockwise movement.

    """
     
    def __init__(self):
        super().__init__("explore")
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_pub = self.create_publisher(RPose, "goal_pose", 10)
        self.data = None
        self.width = None
        self.height = None
        self.maze_mat=None
        self.nnum_zero=None
        self.basex,self.basey,self.basez=0.0,0.0,0.0
        self.resolution = 0.05
        self.goalx = 0.0
        self.goaly = 0.0
        self.goalz = 0.0
        self.state = State.BOTTOM
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        
    def map_callback(self, msg:OccupancyGrid):
      """Gets map data and converts it to a 2D matrix"""
      self.header = msg.header
      self.sec = msg.header.stamp.sec
      self.frame = self.header.frame_id
      self.info = msg.info
      self.width = msg.info.width
      self.height = msg.info.height
      self.origin = msg.info.origin
      self.pos = msg.info.origin.position
      self.ori = msg.info.origin.orientation
      self.data = msg.data
      self.maze_mat = np.reshape(msg.data, (self.height,self.width))
      self.nnum_zero = np.count_nonzero(self.data == -1)
      
    def goal(self):
        """Publish function, when called will publish the goalpose."""
        self.goall = RPose()
        self.goall.header.stamp = self.get_clock().now().to_msg()
        self.goall.header.frame_id = "map"
        self.goall.pose.position.x = float(self.goalx)
        self.goall.pose.position.y = float(self.goaly)
        self.goall.pose.position.z = 0.0
        self.goal_pub.publish(self.goall)
    
    def plotting(self):
      """Helper function, used to plot the graphs"""
      plt.clf()
      plt.ion
      colors = {
        0: 'white',    # Color for 0
        -1: 'red',    # Color for -1
        100: 'green',  # Color for 100\
        }
      cmap = ListedColormap([colors[val] for val in sorted(colors)])
      
      # Create a dictionary to map each value to an index for coloring
      unique_values = np.unique(self.maze_mat)
      value_to_index = {val: i for i, val in enumerate(unique_values)}

      # Normalize the data to map to discrete indices for colormap
      normalized_data = np.vectorize(value_to_index.get)(self.maze_mat)
      
      # Create a plot - map each value to a specific color
      plt.imshow(normalized_data, cmap=cmap, interpolation='nearest')
      plt.plot(self.basex/self.resolution, self.basey/self.resolution, marker='o',c='green')
      plt.title('Data Matrix Visualization')
      plt.xlabel('X-axis')
      plt.ylabel('Y-axis')
      plt.grid(False)
      plt.pause(0.1)
      plt.ioff
      plt.show()
    
    def use_index(self,y,x):
      """Converts positions to the index for the 2D map data matrix.

      Args:
          y (float): The y location of the bot
          x (float): The x location of the bot
          
      Returns
        -------
            An Empty response
              
      """
      
      print(x,y)
      if abs(int(y/self.resolution)) < self.height-1 and abs(int(x/self.resolution)) < self.width-1:
        return(self.maze_mat[abs(int(x/self.resolution)),abs(int(y/self.resolution))])

      elif abs(int(y/self.resolution)) >= self.width-1:
        return(self.maze_mat[abs(self.width)-1,abs(int(y/self.resolution))])

      elif abs(int(x/self.resolution)) >= self.width-1:
        return(self.maze_mat[abs(int(x/self.resolution)),abs(self.width)-1])

    def timer_callback(self):
      if self.data is not None:
        try:
              # get the latest transform between world and brick
              # (rclpy.time.Time() means get the latest information)
              trans = self.buffer.lookup_transform("map", "base_link", rclpy.time.Time())

              self.basex = trans.transform.translation.x -self.pos.x
              self.basey = trans.transform.translation.y -self.pos.y
              self.basez = trans.transform.translation.z
        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().info(f"Lookup exception: {e}")
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().info(f"Connectivity exception: {e}")
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().info(f"Extrapolation exception: {e}")
        

        if self.nnum_zero ==0:
          if self.use_index(self.basex,self.basey-3) == -1 or self.use_index(self.basex,self.basey-3) == 0 and self.state == State.BOTTOM:
            self.goalx = self.basex+self.pos.x
            self.goaly = self.basey-1.0+self.pos.y
            self.goal()
          elif self.use_index(self.basex,self.basey-3) > 95 and self.state== State.BOTTOM:
              self.state = State.RIGHT
          if (self.use_index(self.basex-3,self.basey) == -1 or self.use_index(self.basex-3,self.basey) == 0) and self.state == State.RIGHT:
            self.goalx = self.basex-1.0+self.pos.x
            self.goaly = self.basey+self.pos.y
            self.goal()
          elif self.use_index(self.basex-3,self.basey) > 95 and self.state == State.RIGHT:
              self.state = State.TOP
          if self.use_index(self.basex,self.basey+3) == -1 or self.use_index(self.basex,self.basey+3) == 0 and self.state == State.TOP:
            self.goalx = self.basex + self.pos.x
            self.goaly = self.basey+1.0+ self.pos.y
            self.goal()
          elif self.use_index(self.basex,self.basey+3) > 95 and self.state== State.TOP:
              self.state = State.RIGHT
          if self.use_index(self.basex+2,self.basey) == -1 or self.use_index(self.basex+2,self.basey) ==0 and self.state == State.LEFT:
            self.goalx = self.basex+0.5-self.pos.x
            self.goaly = self.basey - self.pos.y
            self.goal()

def main(args=None):
    """Run main function."""
    rclpy.init(args=args)
    node = Explore()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
