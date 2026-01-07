---
sidebar_position: 3
---

# Chapter 3: Nav2 for Humanoid Navigation

## Understanding Nav2 Architecture for Humanoid Robots

Navigation2 (Nav2) is ROS 2's state-of-the-art navigation framework that provides path planning, obstacle avoidance, and motion control capabilities. For humanoid robots, Nav2 requires specialized configuration to account for bipedal locomotion, balance constraints, and unique kinematic properties.

### Core Nav2 Components

Nav2 consists of several key components that work together to enable autonomous navigation:

- **Navigation Server**: Central coordinator that manages navigation tasks
- **Planner Server**: Global path planning component
- **Controller Server**: Local path following and obstacle avoidance
- **Recovery Server**: Behavior management for getting unstuck
- **BT Navigator**: Behavior tree-based execution of navigation tasks
- **Lifecycle Manager**: Manages the lifecycle of all navigation components

### Humanoid-Specific Navigation Challenges

Humanoid robots face unique navigation challenges that require specialized Nav2 configuration:

- **Balance Constraints**: Maintaining stability during movement
- **Footstep Planning**: Coordinating foot placement for stable locomotion
- **Narrow Spaces**: Navigating through spaces that require precise positioning
- **Dynamic Stability**: Maintaining balance during motion and obstacle avoidance
- **Multi-Contact Support**: Managing contact with the environment for stability

### Nav2 Architecture Diagram

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Navigation    │    │   Behavior      │    │   Global        │
│   Server        │    │   Tree          │    │   Planner       │
│                 │───▶│   Navigator     │───▶│                 │
│                 │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                              │
                              ▼
                    ┌─────────────────┐    ┌─────────────────┐
                    │   Recovery      │    │   Local         │
                    │   Server        │    │   Controller    │
                    │                 │    │                 │
                    └─────────────────┘    └─────────────────┘
```

## Configuring Navigation Stack for Bipedal Locomotion

### Nav2 Configuration for Humanoid Robots

Humanoid robots require specialized Nav2 configuration parameters to account for their unique kinematics:

```yaml
# Nav2 configuration for humanoid robot
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_count: 60
    do_beamskip: False
    lambda_short: 0.1
    likelihood_max_dist: 2.0
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.5
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    scan_topic: "scan"
    sigma_hit: 0.2
    tf_broadcast: True
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "/odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Humanoid-specific behavior tree
    default_nav_through_poses_bt_xml: "humanoid_nav_through_poses.xml"
    default_nav_to_pose_bt_xml: "humanoid_nav_to_pose.xml"

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0  # Lower frequency for humanoid stability
    min_x_velocity_threshold: 0.05  # Minimum velocity for humanoid
    min_y_velocity_threshold: 0.1   # Higher threshold for bipedal robots
    min_theta_velocity_threshold: 0.1
    # Humanoid-specific controller
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific path follower
    FollowPath:
      plugin: "nav2_mppi_controller::MppiController"
      time_steps: 26
      control_freq: 20
      horizon: 1.3
      # Humanoid-specific parameters for balance
      reference_time: 0.5
      motion_forward_only: True  # Humanoids typically move forward
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
      # Cost function weights
      goal_pos_weight: 32.0
      goal_yaw_weight: 32.0
      ref_vel_weight: 0.0
      path_pos_weight: 64.0
      path_yaw_weight: 0.0
      obstacle_weight: 64.0
      obstacle_scale: 1.0
      trap_scale: 1.0
      prefer_forward_weight: 64.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0  # Lower for humanoid stability
      publish_frequency: 5.0
      global_frame: "odom"
      robot_base_frame: "base_link"
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05  # Higher resolution for precise humanoid navigation
      robot_radius: 0.3  # Humanoid-specific footprint
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: False
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: "/scan"
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: "map"
      robot_base_frame: "base_link"
      use_sim_time: True
      robot_radius: 0.3
      resolution: 0.05  # Higher resolution for humanoid precision
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: "/scan"
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

### Humanoid-Specific Navigation Parameters

```yaml
# Additional humanoid-specific parameters
humanoid_navigation:
  ros__parameters:
    # Balance constraints
    max_linear_velocity: 0.5      # Slower for stability
    max_angular_velocity: 0.6     # Limited for balance
    min_linear_velocity: 0.1      # Minimum for humanoid gait
    acceleration_limit: 0.25      # Gentle acceleration for stability
    deceleration_limit: 0.5       # Controlled stopping
    
    # Footstep planning parameters
    step_width: 0.2               # Distance between feet
    step_length: 0.3              # Forward step distance
    step_height: 0.05             # Step clearance
    support_margin: 0.05          # Stability margin
    
    # Path planning for bipedal robots
    min_turn_radius: 0.4          # Minimum turning radius
    preferred_speed: 0.3          # Optimal walking speed
    obstacle_avoidance_gain: 2.0  # Higher gain for safety
```

## Path Planning Algorithms for Humanoid Movement

### Humanoid Path Planning Considerations

Path planning for humanoid robots must account for:

- **Kinematic Constraints**: Joint limits and workspace limitations
- **Dynamic Stability**: Maintaining balance during movement
- **Footstep Planning**: Coordinating foot placement
- **Obstacle Avoidance**: Navigating around obstacles while maintaining stability
- **Energy Efficiency**: Optimizing for battery life

### Example Path Planner Configuration

```cpp
// Humanoid-specific path planner
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.h>

class HumanoidPathPlanner : public nav2_core::GlobalPlanner
{
public:
    void configure(
        const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
        std::string name,
        const std::shared_ptr<tf2_ros::Buffer>& tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>& costmap_ros) override
    {
        node_ = node;
        name_ = name;
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();
        
        // Humanoid-specific parameters
        node->declare_parameter(name + ".foot_separation", 0.2);
        node->declare_parameter(name + ".max_step_length", 0.3);
        node->declare_parameter(name + ".balance_margin", 0.1);
        
        foot_separation_ = node->get_parameter(name + ".foot_separation").as_double();
        max_step_length_ = node->get_parameter(name + ".max_step_length").as_double();
        balance_margin_ = node->get_parameter(name + ".balance_margin").as_double();
    }

    void cleanup() override
    {
        RCLCPP_INFO(
            node_->get_logger(), "Cleaning up planner: %s", name_.c_str());
    }

    void activate() override
    {
        RCLCPP_INFO(
            node_->get_logger(), "Activating planner: %s", name_.c_str());
    }

    void deactivate() override
    {
        RCLCPP_INFO(
            node_->get_logger(), "Deactivating planner: %s", name_.c_str());
    }

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) override
    {
        nav_msgs::msg::Path path = nav2_navfn_planner::NavfnPlanner::createPlan(start, goal);
        
        // Post-process path for humanoid compatibility
        return makeHumanoidPath(path);
    }

private:
    nav_msgs::msg::Path makeHumanoidPath(const nav_msgs::msg::Path& original_path)
    {
        nav_msgs::msg::Path humanoid_path;
        humanoid_path.header = original_path.header;
        
        // Process path to ensure humanoid compatibility
        for (size_t i = 0; i < original_path.poses.size(); ++i) {
            // Check if step is within humanoid capabilities
            if (i == 0 || isStepValid(original_path.poses[i-1], original_path.poses[i])) {
                humanoid_path.poses.push_back(original_path.poses[i]);
            } else {
                // Add intermediate waypoints to ensure valid steps
                auto intermediate_waypoints = generateIntermediateSteps(
                    original_path.poses[i-1], original_path.poses[i]);
                humanoid_path.poses.insert(
                    humanoid_path.poses.end(),
                    intermediate_waypoints.begin(),
                    intermediate_waypoints.end());
            }
        }
        
        return humanoid_path;
    }
    
    bool isStepValid(const geometry_msgs::msg::Pose& from, const geometry_msgs::msg::Pose& to)
    {
        // Check if the step is within humanoid capabilities
        double dist = sqrt(pow(to.position.x - from.position.x, 2) + 
                          pow(to.position.y - from.position.y, 2));
        return dist <= max_step_length_;
    }
    
    std::vector<geometry_msgs::msg::Pose> generateIntermediateSteps(
        const geometry_msgs::msg::Pose& from, const geometry_msgs::msg::Pose& to)
    {
        std::vector<geometry_msgs::msg::Pose> steps;
        // Generate intermediate steps that are within humanoid step limits
        // Implementation details...
        return steps;
    }
    
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::string name_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D* costmap_;
    
    // Humanoid-specific parameters
    double foot_separation_;
    double max_step_length_;
    double balance_margin_;
};
```

### Footstep Planning Integration

Footstep planning is crucial for humanoid navigation and can be integrated with Nav2:

```cpp
// Footstep planner interface
class FootstepPlanner
{
public:
    FootstepPlanner()
    {
        // Initialize footstep planning algorithms
        // Typically uses preview control, DCM (Divergent Component of Motion)
        // or other humanoid-specific planning methods
    }
    
    std::vector<Footstep> planFootsteps(const nav_msgs::msg::Path& body_path)
    {
        std::vector<Footstep> footsteps;
        
        // Convert body path to footstep sequence
        // This involves complex kinematic and dynamic calculations
        for (size_t i = 0; i < body_path.poses.size(); ++i) {
            // Calculate appropriate footstep positions
            // considering balance, stability, and kinematic constraints
            Footstep left_foot, right_foot;
            
            calculateFootPositions(body_path.poses[i], left_foot, right_foot);
            footsteps.push_back(left_foot);
            footsteps.push_back(right_foot);
        }
        
        return footsteps;
    }
    
private:
    void calculateFootPositions(const geometry_msgs::msg::Pose& body_pose,
                               Footstep& left_foot, Footstep& right_foot)
    {
        // Calculate foot positions based on body pose
        // considering humanoid kinematics and balance
    }
};
```

## Obstacle Avoidance and Dynamic Re-planning

### Humanoid-Aware Obstacle Avoidance

```cpp
// Humanoid-aware obstacle avoidance
#include <nav2_core/controller.hpp>

class HumanoidController : public nav2_core::Controller
{
public:
    void configure(
        const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
        std::string name,
        const std::shared_ptr<tf2_ros::Buffer>& tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>& costmap_ros) override
    {
        node_ = node;
        name_ = name;
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();
        
        // Humanoid-specific controller parameters
        node->declare_parameter(name + ".max_linear_speed", 0.5);
        node->declare_parameter(name + ".max_angular_speed", 0.6);
        node->declare_parameter(name + ".balance_threshold", 0.1);
        
        max_linear_speed_ = node->get_parameter(name + ".max_linear_speed").as_double();
        max_angular_speed_ = node->get_parameter(name + ".max_angular_speed").as_double();
        balance_threshold_ = node->get_parameter(name + ".balance_threshold").as_double();
    }

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped& pose,
        const geometry_msgs::msg::Twist& velocity,
        nav2_core::GoalChecker * goal_checker) override
    {
        geometry_msgs::msg::TwistStamped cmd_vel;
        
        // Calculate desired velocity based on path following
        auto desired_twist = calculatePathFollowingVelocity(pose, velocity);
        
        // Apply humanoid-specific constraints
        cmd_vel.twist = applyHumanoidConstraints(desired_twist, pose);
        
        // Check for obstacles and adjust if needed
        cmd_vel.twist = avoidObstacles(cmd_vel.twist, pose);
        
        return cmd_vel;
    }

private:
    geometry_msgs::msg::Twist applyHumanoidConstraints(
        const geometry_msgs::msg::Twist& desired,
        const geometry_msgs::msg::PoseStamped& pose)
    {
        geometry_msgs::msg::Twist constrained = desired;
        
        // Apply humanoid-specific velocity limits
        constrained.linear.x = std::min(constrained.linear.x, max_linear_speed_);
        constrained.linear.x = std::max(constrained.linear.x, -max_linear_speed_);
        
        constrained.angular.z = std::min(constrained.angular.z, max_angular_speed_);
        constrained.angular.z = std::max(constrained.angular.z, -max_angular_speed_);
        
        // Consider balance constraints
        if (std::abs(constrained.angular.z) > balance_threshold_) {
            // Reduce linear speed when turning to maintain balance
            constrained.linear.x *= 0.7;
        }
        
        return constrained;
    }
    
    geometry_msgs::msg::Twist avoidObstacles(
        const geometry_msgs::msg::Twist& cmd_vel,
        const geometry_msgs::msg::PoseStamped& pose)
    {
        // Check costmap for obstacles in the path
        auto cost = getCostInDirection(cmd_vel.linear.x, cmd_vel.angular.z);
        
        if (cost > nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            // Implement humanoid-specific obstacle avoidance
            return executeAvoidanceManeuver(cmd_vel, pose);
        }
        
        return cmd_vel;
    }
    
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::string name_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D* costmap_;
    
    double max_linear_speed_;
    double max_angular_speed_;
    double balance_threshold_;
};
```

## Coordination Between Navigation and Motion Control

### Integration with Humanoid Motion Control

The navigation system must be tightly integrated with the humanoid's motion control system:

```cpp
// Navigation-Motion Control Interface
class NavigationMotionInterface
{
public:
    NavigationMotionInterface()
    {
        // Initialize interfaces to both systems
        nav_client_ = std::make_shared<Nav2Client>();
        motion_controller_ = std::make_shared<HumanoidMotionController>();
        
        // Set up coordination callbacks
        nav_client_->setProgressCallback(
            std::bind(&NavigationMotionInterface::onNavigationProgress, this, std::placeholders::_1));
        nav_client_->setGoalCallback(
            std::bind(&NavigationMotionInterface::onNavigationGoal, this, std::placeholders::_1));
    }
    
    void executeNavigation(const geometry_msgs::msg::PoseStamped& goal)
    {
        // Prepare humanoid for navigation
        motion_controller_->setWalkingMode();
        
        // Execute navigation with motion control coordination
        nav_client_->navigateToPose(goal);
    }

private:
    void onNavigationProgress(const nav2_msgs::action::NavigateToPose::Feedback& feedback)
    {
        // Monitor navigation progress and adjust motion control as needed
        auto robot_state = motion_controller_->getRobotState();
        
        // Check balance and adjust gait if necessary
        if (!robot_state.in_safe_balance_region) {
            nav_client_->cancelGoal();
            motion_controller_->executeRecoveryStep();
            nav_client_->resume();
        }
    }
    
    void onNavigationGoal(const nav2_msgs::action::NavigateToPose::Result& result)
    {
        // Handle navigation completion and transition motion control
        if (result.status == nav2_msgs::action::NavigateToPose::Result::SUCCEEDED) {
            motion_controller_->setStandbyMode();
        } else {
            motion_controller_->setEmergencyStop();
        }
    }
    
    std::shared_ptr<Nav2Client> nav_client_;
    std::shared_ptr<HumanoidMotionController> motion_controller_;
};
```

### Multi-Level Mapping for Humanoid Navigation

Humanoid robots benefit from multi-level mapping approaches:

```cpp
// Multi-level mapping system for humanoid navigation
class MultiLevelHumanoidMap
{
public:
    MultiLevelHumanoidMap()
    {
        // Initialize different map levels
        // 2D costmap for general navigation
        costmap_2d_ = std::make_unique<nav2_costmap_2d::Costmap2DROS>("costmap_2d");
        
        // 3D map for obstacle detection and footstep planning
        occupancy_grid_3d_ = std::make_unique<OctomapManager>();
        
        // Semantic map for understanding environment
        semantic_map_ = std::make_unique<SemanticMapManager>();
    }
    
    void updateMaps(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan,
                   const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud,
                   const vision_msgs::msg::Detection2DArray::SharedPtr detections)
    {
        // Update all map levels simultaneously
        update2DCostmap(laser_scan);
        update3DOccupancyGrid(point_cloud);
        updateSemanticMap(detections);
    }
    
    nav2_costmap_2d::Costmap2D* getTraversableMap() { return costmap_2d_->getCostmap(); }
    octomap::OcTree* get3DMap() { return occupancy_grid_3d_->getOctree(); }
    SemanticMap* getSemanticMap() { return semantic_map_->getMap(); }

private:
    std::unique_ptr<nav2_costmap_2d::Costmap2DROS> costmap_2d_;
    std::unique_ptr<OctomapManager> occupancy_grid_3d_;
    std::unique_ptr<SemanticMapManager> semantic_map_;
};
```

## Navigation Safety and Recovery Behaviors

### Humanoid-Specific Safety Measures

```yaml
# Recovery behaviors for humanoid robots
recoveries_server:
  ros__parameters:
    use_sim_time: True
    recovery_plugins: ["spin", "backup", "homing"]
    spin:
      plugin: "nav2_recoveries/Spin"
      # Humanoid-specific spin parameters
      min_duration: 5.0
      max_duration: 15.0
      spin_dist: 1.57  # 90 degrees for humanoid
    backup:
      plugin: "nav2_recoveries/BackUp"
      # Humanoid-specific backup parameters
      backup_dist: 0.3  # Shorter for humanoid stability
      backup_speed: 0.1
    homing:
      plugin: "nav2_recoveries/Homing"  # Humanoid-specific recovery
      # Move to a safe, stable position
      safe_pose_x: 0.0
      safe_pose_y: 0.0
      safe_tolerance: 0.5
```

### Emergency Recovery Procedures

```cpp
// Emergency recovery for humanoid robots
class HumanoidRecoveryManager
{
public:
    HumanoidRecoveryManager()
    {
        // Initialize recovery behaviors specific to humanoid robots
        recovery_behaviors_ = {
            std::make_shared<StopRecovery>(),
            std::make_shared<BalanceRecovery>(),
            std::make_shared<SafePoseRecovery>(),
            std::make_shared<HomingRecovery>()
        };
    }
    
    RecoveryStatus executeRecovery()
    {
        // Execute recovery based on current situation
        auto robot_state = getRobotState();
        
        if (robot_state.imminent_fall) {
            return executeBalanceRecovery();
        } else if (robot_state.stuck_in_collision) {
            return executeSafeEscape();
        } else if (robot_state.lost_localization) {
            return executeHomingBehavior();
        }
        
        return RecoveryStatus::SUCCESS;
    }

private:
    std::vector<std::shared_ptr<RecoveryBehavior>> recovery_behaviors_;
    
    RecoveryStatus executeBalanceRecovery()
    {
        // Execute immediate balance recovery
        // This might involve stepping, crouching, or other balance recovery actions
        return RecoveryStatus::SUCCESS;
    }
    
    RecoveryStatus executeSafeEscape()
    {
        // Navigate to a safe position away from obstacles
        return RecoveryStatus::SUCCESS;
    }
    
    RecoveryStatus executeHomingBehavior()
    {
        // Navigate to a known safe position
        return RecoveryStatus::SUCCESS;
    }
};
```

The Nav2 framework provides a robust foundation for humanoid robot navigation, but requires careful configuration to account for the unique challenges of bipedal locomotion. By properly configuring path planning, obstacle avoidance, and recovery behaviors for humanoid-specific constraints, Nav2 can provide safe and effective navigation capabilities that form an essential component of the robot's AI brain.