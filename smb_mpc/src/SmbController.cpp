/*!
 * @file 	  SmbController.tpp
 * @author   Johannes Pankert
 * @date		  03/05/2019
 * @version 	1.0
 * @brief    A controller that ...
 */

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TwistStamped.h>

#include "smb_mpc/SmbController.hpp"

namespace smb_mpc
{

  SmbController::SmbController(ros::NodeHandle &nh)
      : nh_(nh), buffer_(), tfListener_(buffer_), smbInterface_(), trajectoriesPublisher_()
  {
    initialize();
  }

  SmbController::~SmbController() {}

  bool SmbController::initialize()
  {

    initializeMPC();

    trajectoriesPublisher_.reset(new TargetPublisher(nh_, "smb"));

    pathSubscriber_ =
        nh_.subscribe("reference_trajectory", 0, &SmbController::pathCallback,
                      this, ros::TransportHints().tcpNoDelay());

    joyTwistInterventionSubscriber_ = nh_.subscribe(
        "joy_twist", 0, &SmbController::joyTwistInverventionCallback, this,
        ros::TransportHints().tcpNoDelay());

    commandTwistPublisher_ =
        nh_.advertise<geometry_msgs::Twist>("command_twist", 1, false);
    commandTwistStampedPublisher_ = nh_.advertise<geometry_msgs::TwistStamped>(
        "command_twist_stamped", 1, false);
    currentPosePublisher_ =
        nh_.advertise<geometry_msgs::PoseStamped>("current_pose", 1, false);

    optimalPosePublisher_ =
        nh_.advertise<geometry_msgs::PoseStamped>("optimal_pose", 1, false);

    optimalPathPublisher_ =
        nh_.advertise<nav_msgs::Path>("optimal_path", 1, false);

    maxLinearVelocity_ =
        nh_.param<double>("max_linear_velocity", maxLinearVelocity_);
    maxAngularVelocity_ =
        nh_.param<double>("max_angular_velocity", maxAngularVelocity_);

    baseFrame_ = nh_.param<std::string>("base_frame", baseFrame_);
    controlFrame_ = nh_.param<std::string>("control_frame", controlFrame_);
    adjustTimeStamps_ = nh_.param<bool>("adjust_time_stamps", adjustTimeStamps_);

    rosPublishingWorker_ = nh_.createTimer(
        ros::Duration(1 / nh_.param<double>("ros_publishing_rate", 10.0)),
        [this](const auto &)
        { this->publishRos(); });

    // Start mpc update worker.
    mpcUpdateWorker_ = nh_.createTimer(
        ros::Duration(1 / smbInterface_->mpcSettings().mpcDesiredFrequency_),
        [this](const auto &)
        { this->mpcUpdate(); });

    trackingControllerWorker_ = nh_.createTimer(
        ros::Duration(1 / smbInterface_->mpcSettings().mrtDesiredFrequency_),
        [this](const auto &)
        { this->advance(); });

    return true;
  }

  bool SmbController::advance()
  {
    try
    {
      auto basePose = buffer_.lookupTransform(controlFrame_, baseFrame_,
                                              ros::Time(0.0), ros::Duration(1.0));

      SmbConversions::writeMpcObservation(observation_, basePose);
      observationAvailable_ = true;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      return true;
    }

    {
      std::unique_lock<std::mutex> lockObservation(observationMutex_,
                                                   std::defer_lock);

      std::unique_lock<std::mutex> lockDesiredTrajectory(
          costDesiredTrajectoriesMutex_, std::defer_lock);

      std::lock(lockObservation, lockDesiredTrajectory);

      auto currentSequence = path_.header.seq;
      if (currentSequence != lastSequence_)
      {
        {
          std::lock_guard<std::mutex> lockGuard(pathMutex_);
          writeDesiredTrajectory(path_, costDesiredTrajectories_, observation_);
        }
        trajectoriesPublisher_->publishTargetTrajectories(costDesiredTrajectories_);

        desiredTrajectoryAvailable_ = true;
        lastSequence_ = currentSequence;
      }
    }

    double controlQueryTime = ros::Time::now().toSec();
    geometry_msgs::TwistStamped twistCommand;
    twistCommand.header.stamp = ros::Time(controlQueryTime);
    twistCommand.header.frame_id = baseFrame_;

    if (planAvailable_)
    {
      ocs2::vector_t controlInput(
          (int)SmbDefinitions::INPUT_DIM);
      ocs2::vector_t optimalState(
          (int)SmbDefinitions::STATE_DIM);
      size_t subsystem;
      mpcInterface_->spinMRT();
      if (mpcInterface_->initialPolicyReceived())
      {
        mpcInterface_->updatePolicy();
        mpcInterface_->evaluatePolicy(controlQueryTime, observation_.state,
                                      optimalState, controlInput, subsystem);

        SmbConversions::readMpcInput(controlInput, twistCommand.twist);
      }
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(3, "[SmbController::advance] no plan "
                                  "available yet. Commanding zero base twist.");
    }

    commandTwistStampedPublisher_.publish(twistCommand);
    commandTwistPublisher_.publish(twistCommand.twist);
    return true;
  }

  bool SmbController::mpcUpdate()
  {
    if (!observationAvailable_)
    {
      ROS_WARN_STREAM_THROTTLE(3, "[SmbController::mpcUpdate] observation "
                                  "not available. Skipping mpc update.");
      return true;
    }
    if (!desiredTrajectoryAvailable_)
    {
      ROS_WARN_STREAM_THROTTLE(3,
                               "[SmbController::mpcUpdate] desired "
                               "trajectory not available. Skipping mpc update.");
      return true;
    }
    {
      std::lock_guard<std::mutex> lockGuard(observationMutex_);
      mpcInterface_->setCurrentObservation(observation_);
    }
    mpcInterface_->updatePolicy();

    planAvailable_ = true;
    return true;
  }

  void SmbController::writeDesiredTrajectory(
      const nav_msgs::Path &path,
      SmbController::CostDesiredTrajectories &costDesiredTrajectories,
      const Observation &currentObservation)
  {
    costDesiredTrajectories.clear();

    // set the desired robot state to the current state in case no path has been
    // received
    if (path.poses.size() == 0)
    {
      costDesiredTrajectories.stateTrajectory.push_back(
          currentObservation.state);
      costDesiredTrajectories.timeTrajectory.push_back(
          currentObservation.time);
      costDesiredTrajectories.inputTrajectory.push_back(
          ocs2::vector_t::Zero(SmbDefinitions::INPUT_DIM));
      ROS_WARN_STREAM_THROTTLE(3,
                               "[SmbController::writeDesiredTrajectory] received "
                               "trajectory is empty!");
    }
    else
    {
      costDesiredTrajectories.stateTrajectory.reserve(path.poses.size());
      costDesiredTrajectories.inputTrajectory.reserve(path.poses.size());
      costDesiredTrajectories.timeTrajectory.reserve(path.poses.size());

      for (int i = 0; i < path.poses.size(); ++i)
      {
        ocs2::vector_t targetState(SmbDefinitions::STATE_DIM);
        SmbConversions::writeMpcState(targetState, path.poses[i].pose);

        costDesiredTrajectories.stateTrajectory.push_back(targetState);
        costDesiredTrajectories.inputTrajectory.push_back(
            ocs2::vector_t::Zero(SmbDefinitions::INPUT_DIM));
        costDesiredTrajectories.timeTrajectory.push_back(
            ros::Time(path.poses[i].header.stamp).toSec());
      }
    }
  }

  void SmbController::pathCallback(const nav_msgs::PathConstPtr &path)
  {
    nav_msgs::Path pathTemp = *path;
    if (pathTemp.poses.size() > 1 && adjustTimeStamps_)
      adjustTimeStamps(pathTemp);
    static uint pathSequence = 1;
    pathTemp.header.seq = pathSequence++;
    {
      std::lock_guard<std::mutex> lockGuard(pathMutex_);
      path_ = pathTemp;
    }
  }

  void SmbController::initializeMPC()
  {
    std::string taskFile, libFolder;
    nh_.getParam("/taskFile", taskFile);
    nh_.getParam("/libFolder", libFolder);

    // Load the SMB interface
    smbInterface_.reset(new SmbInterface(taskFile, libFolder));
    // Setup the MRT
    mpcInterface_.reset(new MpcInterface("smb"));
    mpcInterface_->initRollout(&smbInterface_->getRollout());
    mpcInterface_->launchNodes(nh_);

    // Set first observation
    observation_.state = smbInterface_->getInitialState();
    observation_.input.setZero(SmbDefinitions::INPUT_DIM);
    observation_.time = ros::Time().now().toSec();

    costDesiredTrajectories_ = CostDesiredTrajectories(0);
    planAvailable_ = false;
    observationAvailable_ = false;
    desiredTrajectoryAvailable_ = false;
    lastSequence_ = -1;
    nav_msgs::Path emptyPath;
    emptyPath.header.seq = lastSequence_;
    path_ = emptyPath;
    const TargetTrajectories initTrajectory({observation_.time}, {vector_t::Zero(SmbDefinitions::STATE_DIM)}, {vector_t::Zero(SmbDefinitions::INPUT_DIM)});
    mpcInterface_->resetMpcNode(initTrajectory);
  }

  void SmbController::joyTwistInverventionCallback(
      const geometry_msgs::TwistConstPtr &twist)
  {
    if (path_.header.seq != -1)
    {
      ROS_ERROR_STREAM(
          "Manual joystick intervention detected. Delete MPC setpoint!");
      rosPublishingWorker_.stop();
      mpcUpdateWorker_.stop();
      trackingControllerWorker_.stop();

      // TODO: resetMpc();

      mpcInterface_->reset();
      rosPublishingWorker_.start();
      mpcUpdateWorker_.start();
      trackingControllerWorker_.start();
    }
  }

  void SmbController::adjustTimeStamps(nav_msgs::Path &path)
  {
    if (path.poses.size() > 0)
    {
      ros::Time lastPoseOriginalTimestamp(path.poses[0].header.stamp);
      auto timeSec = ros::Time::now().toSec() -
                     ros::Time(path.header.stamp).toSec() +
                     path.poses[0].header.stamp.toSec();
      path.poses[0].header.stamp = ros::Time(timeSec);
      path.header.stamp = ros::Time::now();
      for (int i = 1; i < path.poses.size(); i++)
      {
        auto poseFrom = path.poses[i - 1].pose;
        Eigen::Vector3d positionFrom;
        tf::pointMsgToEigen(poseFrom.position, positionFrom);
        Eigen::Quaterniond orientationFrom;
        tf::quaternionMsgToEigen(poseFrom.orientation, orientationFrom);

        auto poseTo = path.poses[i].pose;
        Eigen::Vector3d positionTo;
        tf::pointMsgToEigen(poseTo.position, positionTo);
        Eigen::Quaterniond orientationTo;
        tf::quaternionMsgToEigen(poseTo.orientation, orientationTo);

        double minTimeLinear =
            (positionTo - positionFrom).norm() / maxLinearVelocity_;

        double minTimeAngular =
            std::abs(orientationTo.angularDistance(orientationFrom)) /
            maxAngularVelocity_;
        double deltaOriginal = ros::Time(path.poses[i].header.stamp).toSec() -
                               lastPoseOriginalTimestamp.toSec();

        timeSec +=
            std::max(std::max(minTimeLinear, minTimeAngular), deltaOriginal);
        lastPoseOriginalTimestamp = ros::Time(path.poses[i].header.stamp);
        path.poses[i].header.stamp = ros::Time(timeSec);
      }

      double duration =
          (path.poses.back().header.stamp - path.poses.front().header.stamp)
              .toSec();
      ROS_WARN_STREAM_THROTTLE(
          5, "[DesiredBasePathModule::adjustTimeStamps] adjusted timestamps of "
                 << path.poses.size()
                 << " poses in path. Duration: " << duration);
    }
  }

  bool SmbController::publishRos()
  {
    if (!planAvailable_)
    {
      return true;
    }

    // publish current state
    geometry_msgs::PoseStamped currentBasePose;
    currentBasePose.header.frame_id = controlFrame_;
    {
      std::lock_guard<std::mutex> lockGuard(observationMutex_);
      SmbConversions::readMpcObservation(observation_, currentBasePose);
    }
    currentPosePublisher_.publish(currentBasePose);

    // publish optimal state
    geometry_msgs::PoseStamped optimalPose;
    optimalPose.header.frame_id = controlFrame_;
    optimalPose.header.stamp = ros::Time::now();

    ocs2::vector_t controlInput(SmbDefinitions::INPUT_DIM);
    ocs2::vector_t optimalState(SmbDefinitions::STATE_DIM);
    size_t subsystem;
    {
      std::lock_guard<std::mutex> lockGuard(observationMutex_);
      mpcInterface_->evaluatePolicy(observation_.time, observation_.state,
                                    optimalState, controlInput, subsystem);
    }

    SmbConversions::readMpcState(optimalState, optimalPose.pose);
    optimalPosePublisher_.publish(optimalPose);

    // publish current rollout
    const auto &timeTrajectory = mpcInterface_->getPolicy().timeTrajectory_;
    const auto &stateTrajectory = mpcInterface_->getPolicy().stateTrajectory_;
    nav_msgs::Path optimalTrajectory;
    optimalTrajectory.header.frame_id = controlFrame_;
    optimalTrajectory.header.stamp - ros::Time::now();
    for (int i = 0; i < timeTrajectory.size(); i++)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = controlFrame_;
      pose.header.stamp = ros::Time(timeTrajectory[i]);
      pose.header.seq = i;
      SmbConversions::readMpcState(stateTrajectory[i], pose.pose);
      optimalTrajectory.poses.push_back(pose);
    }
    optimalPathPublisher_.publish(optimalTrajectory);

    // command line debugging output
    ROS_WARN_STREAM_THROTTLE(
        3, "[SmbController::advance]"
               << std::endl
               << "current_state:" << observation_.state.transpose() << std::endl
               << "optimalState:" << optimalState.transpose() << std::endl
               << "controlInput:" << controlInput.transpose() << std::endl
               << "current time: " << observation_.time
               << " current trajectory timespan: [" << timeTrajectory.front()
               << ", " << timeTrajectory.back() << "]");

    return true;
  }

} // namespace smb_path_following
