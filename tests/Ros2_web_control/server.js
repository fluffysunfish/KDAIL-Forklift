const express = require("express");
const http = require("http");
const path = require("path");

const app = express();
const server = http.createServer(app);

// Serve static files
app.use(express.static("public"));

// Serve the main page
app.get("/", (req, res) => {
  res.sendFile(path.join(__dirname, "public", "index.html"));
});

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
      "std_msgs/msg/Float32",
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
      "std_msgs/msg/Float32",
      "/imu_1_yaw",
      (msg) => {
        currentPose.yaw = msg.data;
        io.emit("poseUpdate", currentPose);
      },
    );

    // MJPEG Stream endpoint
    app.get('/stream', (req, res) => {
      res.writeHead(200, {
        'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
        'Cache-Control': 'no-cache',
        'Connection': 'keep-alive',
        'Pragma': 'no-cache'
      });

      const imageSubscriber = node.createSubscription(
        "sensor_msgs/msg/Image",
        "/image",
        (msg) => {
          try {
            const imageData = Buffer.from(msg.data);
            let frame = Buffer.concat([
              Buffer.from('--frame\r\n'),
              Buffer.from('Content-Type: image/jpeg\r\n\r\n'),
              imageData,
              Buffer.from('\r\n')
            ]);
            res.write(frame);
          } catch (error) {
            console.error("Error processing image:", error);
          }
        },
        {
          qos: {
            depth: 1,
            reliability: rclnodejs.QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability: rclnodejs.QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            history: rclnodejs.QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
          }
        }
      );

      req.on('close', () => {
        node.destroySubscription(imageSubscriber);
      });
    });

    // Socket.IO connections
    io.on("connection", (socket) => {
      console.log("Client connected");

      // Send current pose to new client
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

// Start server
const PORT = process.env.PORT || 3000;
server.listen(PORT, () => {
  console.log(`Server running on http://localhost:${PORT}`);
  console.log('Make sure rosbridge_server is running with:');
  console.log('ros2 launch rosbridge_server rosbridge_websocket_launch.xml');
});

// Graceful shutdown
process.on("SIGINT", () => {
  console.log("Shutting down...");
  process.exit(0);
});
