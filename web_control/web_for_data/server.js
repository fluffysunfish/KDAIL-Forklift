const express = require("express");
const http = require("http");
const socketIo = require("socket.io");
const path = require("path");
const rclnodejs = require("rclnodejs");

const app = express();
const server = http.createServer(app);
const io = socketIo(server);

// Serve static files
app.use(express.static("public"));

// Serve the main page
app.get("/", (req, res) => {
  res.sendFile(path.join(__dirname, "public", "index.html"));
});

let node;
let currentPose = { x: 0, y: 0, yaw: 0 };
let mapBounds = { x_max: 700, y_max: 702 }; // Match anchor positions

// Initialize ROS2
async function initROS2() {
  try {
    await rclnodejs.init();
    node = new rclnodejs.Node("web_control_node");

    // Publishers
    const gotoXYPublisher = node.createPublisher(
      "geometry_msgs/msg/Point",
      "/goto_pose/xy",
    );
    const gotoYawPublisher = node.createPublisher(
      "std_msgs/msg/Float64",
      "/goto_pose/yaw",
    );
    const motorCommandsPublisher = node.createPublisher(
      "std_msgs/msg/String",
      "/motor_commands",
    );

    // Subscribers
    const finalXYSubscriber = node.createSubscription(
      "geometry_msgs/msg/Point",
      "/final_xy",
      (msg) => {
        currentPose.x = msg.x;
        currentPose.y = msg.y;
        io.emit("poseUpdate", currentPose);
      },
    );

    const imuYawSubscriber = node.createSubscription(
      "std_msgs/msg/Float64",
      "/imu_1_yaw",
      (msg) => {
        currentPose.yaw = msg.data;
        io.emit("poseUpdate", currentPose);
      },
    );

    const uwbDataSubscriber = node.createSubscription(
      "std_msgs/msg/String",
      "/uwb_detailed_data",
      (msg) => {
        try {
          const uwbData = JSON.parse(msg.data);
          io.emit("uwbDataUpdate", uwbData);
        } catch (error) {
          console.error("Error parsing UWB data:", error);
        }
      },
    );

    // Socket.IO connections
    io.on("connection", (socket) => {
      console.log("Client connected");

      // Send current pose and map bounds to new client
      socket.emit("poseUpdate", currentPose);
      socket.emit("mapBounds", mapBounds);

      // Handle map bounds update
      socket.on("updateMapBounds", (bounds) => {
        mapBounds = bounds;
        io.emit("mapBounds", mapBounds);
      });

      // Handle goto pose commands
      socket.on("gotoXY", (pose) => {
        const pointMsg = {
          x: parseFloat(pose.x),
          y: parseFloat(pose.y),
          z: 0.0,
        };
        gotoXYPublisher.publish(pointMsg);
        console.log("Published goto XY:", pointMsg);
      });

      socket.on("gotoYaw", (yaw) => {
        const yawMsg = { data: parseFloat(yaw) };
        gotoYawPublisher.publish(yawMsg);
        console.log("Published goto Yaw:", yawMsg);
      });

      // Handle motor commands
      socket.on("motorCommand", (command) => {
        const commandMsg = { data: command };
        motorCommandsPublisher.publish(commandMsg);
        console.log("Published motor command:", command);
      });

      socket.on("disconnect", () => {
        console.log("Client disconnected");
      });
    });

    // Start spinning
    rclnodejs.spin(node);
    console.log("ROS2 node initialized and spinning");
  } catch (error) {
    console.error("Failed to initialize ROS2:", error);
  }
}

// Start server with custom address
const PORT = process.env.PORT || 3000;
const HOST = process.env.HOST || "0.0.0.0"; // Default to all interfaces
server.listen(PORT, HOST, () => {
  console.log(`Server running on http://${HOST}:${PORT}`);
  initROS2();
});

// Graceful shutdown
process.on("SIGINT", () => {
  console.log("Shutting down...");
  if (node) {
    node.destroy();
  }
  rclnodejs.shutdown();
  process.exit(0);
});
