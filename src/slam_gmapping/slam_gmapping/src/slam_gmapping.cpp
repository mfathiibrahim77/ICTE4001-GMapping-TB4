/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */
/* Modified by: Charles DuHadway */

//
// Created by shivesh on 29/10/18.
//

#include "slam_gmapping/slam_gmapping.h"

#include "tf2_ros/create_timer_ros.h"

#include <algorithm>
#include <cmath>

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

using std::placeholders::_1;

SlamGmapping::SlamGmapping():
    Node("slam_gmapping"),
    scan_filter_sub_(nullptr),
    scan_filter_(nullptr),
    laser_count_(0),
    transform_thread_(nullptr)
{
    buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
     auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface());
    buffer_->setCreateTimerInterface(timer_interface);
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    tfB_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    map_to_odom_.setIdentity();
    seed_ = static_cast<unsigned long>(time(nullptr));
    init();
    startLiveSlam();
}

void SlamGmapping::init() {
    gsp_ = new GMapping::GridSlamProcessor();

    gsp_laser_ = nullptr;
    gsp_odom_ = nullptr;
    got_first_scan_ = false;
    got_map_ = false;

    // [ICTE4001] All parameters were hard-coded in the upstream ROS 2 port, so a YAML
    // file had no effect. They are now declared as ROS 2 parameters (same names as the
    // original ROS 1 gmapping wrapper); the defaults below are the upstream values.
    // See tb4_gmapping_nav/config/gmapping.yaml for the meaning of each one.

    // --- frames / timing ---
    throttle_scans_ = this->declare_parameter<int>("throttle_scans", 1);
    if (throttle_scans_ < 1) throttle_scans_ = 1;
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    transform_publish_period_ = this->declare_parameter<double>("transform_publish_period", 0.05);
    tf_delay_ = this->declare_parameter<double>("tf_delay", transform_publish_period_);
    map_update_interval_ = tf2::durationFromSec(
        this->declare_parameter<double>("map_update_interval", 0.5));
    publish_particles_ = this->declare_parameter<bool>("publish_particles", true);

    // --- laser / scan-matcher (likelihood field + hill-climbing optimiser) ---
    // maxRange / maxUrange <= 0 means "take it from the LaserScan's range_max"
    maxRange_ = this->declare_parameter<double>("maxRange", -1.0);
    maxUrange_ = this->declare_parameter<double>("maxUrange", -1.0);
    minimum_score_ = this->declare_parameter<double>("minimumScore", 0.0);
    sigma_ = this->declare_parameter<double>("sigma", 0.05);
    kernelSize_ = this->declare_parameter<int>("kernelSize", 1);
    lstep_ = this->declare_parameter<double>("lstep", 0.05);
    astep_ = this->declare_parameter<double>("astep", 0.05);
    iterations_ = this->declare_parameter<int>("iterations", 5);
    lsigma_ = this->declare_parameter<double>("lsigma", 0.075);
    ogain_ = this->declare_parameter<double>("ogain", 3.0);
    lskip_ = this->declare_parameter<int>("lskip", 0);

    // --- odometry motion model noise ---
    srr_ = this->declare_parameter<double>("srr", 0.1);
    srt_ = this->declare_parameter<double>("srt", 0.2);
    str_ = this->declare_parameter<double>("str", 0.1);
    stt_ = this->declare_parameter<double>("stt", 0.2);

    // --- filter update policy ---
    linearUpdate_ = this->declare_parameter<double>("linearUpdate", 1.0);
    angularUpdate_ = this->declare_parameter<double>("angularUpdate", 0.5);
    temporalUpdate_ = this->declare_parameter<double>("temporalUpdate", 1.0);
    resampleThreshold_ = this->declare_parameter<double>("resampleThreshold", 0.5);
    particles_ = this->declare_parameter<int>("particles", 30);

    // --- map ---
    xmin_ = this->declare_parameter<double>("xmin", -10.0);
    ymin_ = this->declare_parameter<double>("ymin", -10.0);
    xmax_ = this->declare_parameter<double>("xmax", 10.0);
    ymax_ = this->declare_parameter<double>("ymax", 10.0);
    delta_ = this->declare_parameter<double>("delta", 0.05);
    occ_thresh_ = this->declare_parameter<double>("occ_thresh", 0.25);

    // --- likelihood sampling used when computing the proposal distribution ---
    llsamplerange_ = this->declare_parameter<double>("llsamplerange", 0.01);
    llsamplestep_ = this->declare_parameter<double>("llsamplestep", 0.01);
    lasamplerange_ = this->declare_parameter<double>("lasamplerange", 0.005);
    lasamplestep_ = this->declare_parameter<double>("lasamplestep", 0.005);

    last_map_update_ = tf2::TimePointZero;

    RCLCPP_INFO(this->get_logger(),
        "GMapping params: particles=%d delta=%.3f srr=%.3f srt=%.3f str=%.3f stt=%.3f "
        "linearUpdate=%.2f angularUpdate=%.2f resampleThreshold=%.2f",
        particles_, delta_, srr_, srt_, str_, stt_, linearUpdate_, angularUpdate_, resampleThreshold_);
}

void SlamGmapping::startLiveSlam() {
    entropy_publisher_ = this->create_publisher<std_msgs::msg::Float64>("entropy", rclcpp::SystemDefaultsQoS());
    neff_publisher_ = this->create_publisher<std_msgs::msg::Float64>("~/neff", rclcpp::SystemDefaultsQoS());
    // [ICTE4001] The map is published "latched" (reliable + transient_local) so that late
    // subscribers such as Nav2's global costmap static_layer, RViz and map_saver still
    // receive the latest map. With the upstream SystemDefaultsQoS (volatile) Nav2 never
    // got a map unless it happened to be subscribed before the next update.
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    sst_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", map_qos);
    sstm_ = this->create_publisher<nav_msgs::msg::MapMetaData>("map_metadata", map_qos);
    if (publish_particles_) {
        particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "~/particles", rclcpp::SensorDataQoS());
    }
    scan_filter_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>
            (node_, "scan", rclcpp::SensorDataQoS().get_rmw_qos_profile());
//    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//        "scan", rclcpp::SensorDataQoS(),
//        std::bind(&SlamGmapping::laserCallback, this, std::placeholders::_1));
    scan_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>
            (*scan_filter_sub_, *buffer_, odom_frame_, 10, node_);
    scan_filter_->registerCallback(std::bind(&SlamGmapping::laserCallback, this, std::placeholders::_1));
    transform_thread_ = std::make_shared<std::thread>
            (std::bind(&SlamGmapping::publishLoop, this, transform_publish_period_));
}

void SlamGmapping::publishLoop(double transform_publish_period){
    if (transform_publish_period == 0)
        return;
    rclcpp::Rate r(1.0 / transform_publish_period);
    while (rclcpp::ok()) {
        publishTransform();
        r.sleep();
    }
}

SlamGmapping::~SlamGmapping()
{
    if(transform_thread_){
        transform_thread_->join();
    }

    delete gsp_;
    delete gsp_laser_;
    delete gsp_odom_;
}

bool SlamGmapping::getOdomPose(GMapping::OrientedPoint& gmap_pose, const rclcpp::Time& t)
{
    // Get the pose of the centered laser at the right time
    centered_laser_pose_.header.stamp = t;
    // Get the laser's pose that is centered
    geometry_msgs::msg::PoseStamped odom_pose;
    try
    {
        buffer_->transform(centered_laser_pose_, odom_pose, odom_frame_, tf2::durationFromSec(1.0));
    }
    catch(tf2::TransformException& e)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to compute odom pose, skipping scan (%s)", e.what());
        return false;
    }

    double yaw = tf2::getYaw(odom_pose.pose.orientation);

    gmap_pose = GMapping::OrientedPoint(odom_pose.pose.position.x,
                                        odom_pose.pose.position.y,
                                        yaw);
    return true;
}

bool SlamGmapping::initMapper(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
    laser_frame_ = scan->header.frame_id;
    // Get the laser's pose, relative to base.
    geometry_msgs::msg::PoseStamped ident;
    geometry_msgs::msg::PoseStamped laser_pose;

    try{
        ident.header.frame_id = laser_frame_;
        ident.header.stamp = scan->header.stamp;
        tf2::Transform transform;
        transform.setIdentity();
        tf2::toMsg(transform, ident.pose);
        buffer_->transform(ident, laser_pose, base_frame_);
    }
    catch (tf2::TransformException& e){
        RCLCPP_WARN(this->get_logger(), "Failed to compute laser pose, aborting initialization (%s)", e.what());
        return false;
    }

    // create a point 1m above the laser position and transform it into the laser-frame
    geometry_msgs::msg::PointStamped up;
    up.header.stamp = scan->header.stamp;
    up.header.frame_id = base_frame_;
    up.point.x = up.point.y = 0;
    up.point.z = 1 + laser_pose.pose.position.z;
    try
    {
        buffer_->transform(up, up, laser_frame_);
    }
    catch(tf2::TransformException& e)
    {
        RCLCPP_WARN(this->get_logger(), "Unable to determine orientation of laser: %s", e.what());
        return false;
    }

    // gmapping doesnt take roll or pitch into account. So check for correct sensor alignment.
    if (fabs(fabs(up.point.z) - 1) > 0.001)
    {
        RCLCPP_INFO(this->get_logger(),
                "Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f", up.point.z);
        return false;
    }

    gsp_laser_beam_count_ = static_cast<unsigned int>(scan->ranges.size());

    double angle_center = (scan->angle_min + scan->angle_max)/2;

    centered_laser_pose_.header.frame_id = laser_frame_;
    centered_laser_pose_.header.stamp = get_clock()->now();
    tf2::Quaternion q;

    if (up.point.z > 0)
    {
        do_reverse_range_ = scan->angle_min > scan->angle_max;
        q.setEuler(angle_center, 0, 0);
        RCLCPP_INFO(this->get_logger(),"Laser is mounted upwards.");
    }
    else
    {
        do_reverse_range_ = scan->angle_min < scan->angle_max;
        q.setEuler(-angle_center, 0, M_PI);
        RCLCPP_INFO(this->get_logger(), "Laser is mounted upside down.");
    }

    centered_laser_pose_.pose.position.x = 0;
    centered_laser_pose_.pose.position.y = 0;
    centered_laser_pose_.pose.position.z = 0;

    centered_laser_pose_.pose.orientation.w = q.getW();
    centered_laser_pose_.pose.orientation.x = q.getX();
    centered_laser_pose_.pose.orientation.y = q.getY();
    centered_laser_pose_.pose.orientation.z = q.getZ();

    // Compute the angles of the laser from -x to x, basically symmetric and in increasing order
    laser_angles_.resize(scan->ranges.size());
    // Make sure angles are started so that they are centered
    double theta = - std::fabs(scan->angle_min - scan->angle_max)/2;
    for(unsigned int i=0; i<scan->ranges.size(); ++i)
    {
        laser_angles_[i]=theta;
        theta += std::fabs(scan->angle_increment);
    }

    RCLCPP_DEBUG(this->get_logger(), "Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f",
            scan->angle_min, scan->angle_max, scan->angle_increment);
    RCLCPP_DEBUG(this->get_logger(), "Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: %.3f",
            laser_angles_.front(), laser_angles_.back(), std::fabs(scan->angle_increment));

    GMapping::OrientedPoint gmap_pose(0, 0, 0);

    // [ICTE4001] Upstream always overwrote both values with the scan's range_max, ignoring
    // configuration. Like the ROS 1 wrapper, only fall back to range_max when unset (<= 0).
    // Setting maxRange slightly ABOVE the sensor's range_max (and maxUrange below it) makes
    // "no return" beams clear free space along the ray instead of being discarded.
    if (maxRange_ <= 0.0)
        maxRange_ = scan->range_max - 0.01;
    if (maxUrange_ <= 0.0 || maxUrange_ > maxRange_)
        maxUrange_ = std::min<double>(maxRange_, scan->range_max - 0.01);
    RCLCPP_INFO(this->get_logger(), "Laser: %u beams, range_max=%.2f -> maxRange=%.2f maxUrange=%.2f",
                gsp_laser_beam_count_, scan->range_max, maxRange_, maxUrange_);

    // The laser must be called "FLASER".
    // We pass in the absolute value of the computed angle increment, on the
    // assumption that GMapping requires a positive angle increment.  If the
    // actual increment is negative, we'll swap the order of ranges before
    // feeding each scan to GMapping.
    gsp_laser_ = new GMapping::RangeSensor("FLASER", gsp_laser_beam_count_, fabs(scan->angle_increment), gmap_pose,
                                           0.0, maxRange_);

    GMapping::SensorMap smap;
    smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
    gsp_->setSensorMap(smap);

    gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);

    /// @todo Expose setting an initial pose
    GMapping::OrientedPoint initialPose;
    if(!getOdomPose(initialPose, scan->header.stamp))
    {
        RCLCPP_WARN(this->get_logger(), "Unable to determine inital pose of laser! Starting point will be set to zero.");
        initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
    }

    gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                                kernelSize_, lstep_, astep_, iterations_,
                                lsigma_, ogain_, static_cast<unsigned int>(lskip_));

    gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
    gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
    gsp_->setUpdatePeriod(temporalUpdate_);
    gsp_->setgenerateMap(false);
    gsp_->GridSlamProcessor::init(static_cast<unsigned int>(particles_), xmin_, ymin_, xmax_, ymax_,
                                  delta_, initialPose);
    gsp_->setllsamplerange(llsamplerange_);
    gsp_->setllsamplestep(llsamplestep_);
    /// @todo Check these calls; in the gmapping gui, they use
    /// llsamplestep and llsamplerange intead of lasamplestep and
    /// lasamplerange.  It was probably a typo, but who knows.
    gsp_->setlasamplerange(lasamplerange_);
    gsp_->setlasamplestep(lasamplestep_);
    gsp_->setminimumScore(minimum_score_);

    // Call the sampling function once to set the seed.
    GMapping::sampleGaussian(1, static_cast<unsigned int>(seed_));

    RCLCPP_INFO(this->get_logger(), "Initialization complete");

    return true;
}

bool SlamGmapping::addScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan, GMapping::OrientedPoint& gmap_pose) {
    if (!getOdomPose(gmap_pose, scan->header.stamp))
        return false;

    if (scan->ranges.size() != gsp_laser_beam_count_)
        return false;

    // GMapping wants an array of doubles...
    auto *ranges_double = new double[scan->ranges.size()];
    // If the angle increment is negative, we have to invert the order of the readings.
    if (do_reverse_range_) {
        RCLCPP_DEBUG(this->get_logger(), "Inverting scan");
        int num_ranges = static_cast<int>(scan->ranges.size());
        for (int i = 0; i < num_ranges; i++) {
            // Must filter out short readings, because the mapper won't.
            // [ICTE4001] +inf ("no return", as published by the TB4 RPLIDAR and the Gazebo
            // lidar) is also mapped to range_max so it can be used to clear free space.
            const float r = scan->ranges[num_ranges - i - 1];
            if (r < scan->range_min || std::isinf(r))
                ranges_double[i] = (double) scan->range_max;
            else
                ranges_double[i] = (double) r;
        }
    } else {
        for (unsigned int i = 0; i < scan->ranges.size(); i++) {
            const float r = scan->ranges[i];
            if (r < scan->range_min || std::isinf(r))
                ranges_double[i] = (double) scan->range_max;
            else
                ranges_double[i] = (double) r;
        }
    }

    GMapping::RangeReading reading(static_cast<unsigned int>(scan->ranges.size()),
                                   ranges_double,
                                   gsp_laser_,
                                   // [ICTE4001] upstream passed stamp.sec (whole seconds),
                                   // which quantised temporalUpdate to 1 s steps
                                   rclcpp::Time(scan->header.stamp).seconds());

    // ...but it deep copies them in RangeReading constructor, so we don't
    // need to keep our array around.
    delete[] ranges_double;

    reading.setPose(gmap_pose);

    RCLCPP_DEBUG(this->get_logger(), "processing scan");

    return gsp_->processScan(reading);
}


void SlamGmapping::laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
    laser_count_++;
    if ((laser_count_ % throttle_scans_) != 0)
        return;

    // We can't initialize the mapper until we've got the first scan
    if(!got_first_scan_)
    {
        if(!initMapper(scan))
            return;
        got_first_scan_ = true;
    }

    GMapping::OrientedPoint odom_pose;

    if(addScan(scan, odom_pose))
    {
        GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;

        tf2::Quaternion q;
        q.setRPY(0, 0, mpose.theta);
        tf2::Transform laser_to_map = tf2::Transform(q, tf2::Vector3(mpose.x, mpose.y, 0.0)).inverse();
        q.setRPY(0, 0, odom_pose.theta);
        tf2::Transform odom_to_laser = tf2::Transform(q, tf2::Vector3(odom_pose.x, odom_pose.y, 0.0));

        map_to_odom_mutex_.lock();
        map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
        map_to_odom_mutex_.unlock();

        if (publish_particles_)
            publishParticles(scan->header.stamp);

        // [ICTE4001] filter diagnostics after every filter update (was: only at map updates)
        if (!gsp_->getWeights().empty()) {
            std_msgs::msg::Float64 msg;
            msg.data = computePoseEntropy();
            entropy_publisher_->publish(msg);
            msg.data = gsp_->getneff();   // N_eff = 1 / sum(w~_i^2), drives selective resampling
            neff_publisher_->publish(msg);
        }

        tf2::TimePoint timestamp = tf2_ros::fromMsg(scan->header.stamp);
        if(!got_map_ || (timestamp - last_map_update_) > map_update_interval_)
        {
            updateMap(scan);
            last_map_update_ = timestamp;
        }
    }
}

// [ICTE4001] Publish every particle's (laser) pose in the map frame so the
// Rao-Blackwellised particle filter can be visualised in RViz (PoseArray display).
void SlamGmapping::publishParticles(const rclcpp::Time & stamp)
{
    if (!particles_pub_)
        return;
    geometry_msgs::msg::PoseArray cloud;
    cloud.header.frame_id = map_frame_;
    cloud.header.stamp = stamp;
    const auto & particles = gsp_->getParticles();
    cloud.poses.reserve(particles.size());
    for (const auto & p : particles) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = p.pose.x;
        pose.position.y = p.pose.y;
        tf2::Quaternion q;
        q.setRPY(0, 0, p.pose.theta);
        pose.orientation = tf2::toMsg(q);
        cloud.poses.push_back(pose);
    }
    particles_pub_->publish(cloud);
}

// [ICTE4001] Upstream (and the original ROS 1 wrapper) computed this from Particle::weight,
// which holds the accumulated LOG-likelihood, not a probability. The result was meaningless
// (~ln(N) almost always). Use the normalised weights w~_i of the last normalize() call.
double SlamGmapping::computePoseEntropy()
{
    double entropy = 0.0;
    for (double w : gsp_->getWeights()) {
        if (w > 0.0)
            entropy -= w * std::log(w);
    }
    return entropy;   // 0 = one particle dominates, ln(N) = all particles equally likely
}

void SlamGmapping::updateMap(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
    RCLCPP_DEBUG(this->get_logger(), "Update map");
    map_mutex_.lock();
    GMapping::ScanMatcher matcher;

    matcher.setLaserParameters(static_cast<unsigned int>(scan->ranges.size()), &(laser_angles_[0]),
                               gsp_laser_->getPose());

    matcher.setlaserMaxRange(maxRange_);
    matcher.setusableRange(maxUrange_);
    matcher.setgenerateMap(true);

    GMapping::GridSlamProcessor::Particle best =
            gsp_->getParticles()[gsp_->getBestParticleIndex()];

    if(!got_map_) {
        map_.info.resolution = static_cast<nav_msgs::msg::MapMetaData::_resolution_type>(delta_);
        map_.info.origin.position.x = 0.0;
        map_.info.origin.position.y = 0.0;
        map_.info.origin.position.z = 0.0;
        map_.info.origin.orientation.x = 0.0;
        map_.info.origin.orientation.y = 0.0;
        map_.info.origin.orientation.z = 0.0;
        map_.info.origin.orientation.w = 1.0;
    }

    GMapping::Point center;
    center.x=(xmin_ + xmax_) / 2.0;
    center.y=(ymin_ + ymax_) / 2.0;

    GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_,
                                  delta_);

    RCLCPP_DEBUG(this->get_logger(), "Trajectory tree:");
    for(GMapping::GridSlamProcessor::TNode* n = best.node;
        n;
        n = n->parent)
    {
        RCLCPP_DEBUG(this->get_logger(), "  %.3f %.3f %.3f",
                  n->pose.x,
                  n->pose.y,
                  n->pose.theta);
        if(!n->reading)
        {
            RCLCPP_DEBUG(this->get_logger(), "Reading is NULL");
            continue;
        }
        matcher.invalidateActiveArea();
        matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
        matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
    }

    // the map may have expanded, so resize ros message as well
    if(map_.info.width != (unsigned int) smap.getMapSizeX() || map_.info.height != (unsigned int) smap.getMapSizeY()) {

        // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
        //       so we must obtain the bounding box in a different way
        GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
        GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
        xmin_ = wmin.x; ymin_ = wmin.y;
        xmax_ = wmax.x; ymax_ = wmax.y;

        RCLCPP_DEBUG(this->get_logger(), "map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
                  xmin_, ymin_, xmax_, ymax_);

        map_.info.width = static_cast<nav_msgs::msg::MapMetaData::_width_type>(smap.getMapSizeX());
        map_.info.height = static_cast<nav_msgs::msg::MapMetaData::_height_type>(smap.getMapSizeY());
        map_.info.origin.position.x = xmin_;
        map_.info.origin.position.y = ymin_;
        map_.data.resize(map_.info.width * map_.info.height);

        RCLCPP_DEBUG(this->get_logger(), "map origin: (%f, %f)", map_.info.origin.position.x, map_.info.origin.position.y);
    }

    for(int x=0; x < smap.getMapSizeX(); x++)
    {
        for(int y=0; y < smap.getMapSizeY(); y++)
        {
            /// @todo Sort out the unknown vs. free vs. obstacle thresholding
            GMapping::IntPoint p(x, y);
            double occ=smap.cell(p);
            assert(occ <= 1.0);
            if(occ < 0)
                map_.data[MAP_IDX(map_.info.width, x, y)] = -1;
            else if(occ > occ_thresh_)
            {
                //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
                map_.data[MAP_IDX(map_.info.width, x, y)] = 100;
            }
            else
                map_.data[MAP_IDX(map_.info.width, x, y)] = 0;
        }
    }
    got_map_ = true;

    //make sure to set the header information on the map
    map_.header.stamp = get_clock()->now();
    map_.header.frame_id = map_frame_;

    sst_->publish(map_);
    sstm_->publish(map_.info);
    map_mutex_.unlock();
}

void SlamGmapping::publishTransform()
{
    map_to_odom_mutex_.lock();
    // [ICTE4001] upstream cast tf_delay (0.05 s) to whole seconds, i.e. always 0.
    // Future-dating map->odom slightly lets consumers look it up at the latest scan time.
    rclcpp::Time tf_expiration = get_clock()->now() + rclcpp::Duration::from_seconds(tf_delay_);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = map_frame_;
    transform.header.stamp = tf_expiration;
    transform.child_frame_id = odom_frame_;
    try {
        transform.transform = tf2::toMsg(map_to_odom_);
        tfB_->sendTransform(transform);
    }
    catch (tf2::LookupException& te){
        RCLCPP_INFO(this->get_logger(), "%s", te.what());
    }
    map_to_odom_mutex_.unlock();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto slam_gmapping_node = std::make_shared<SlamGmapping>();
    rclcpp::spin(slam_gmapping_node);
    return(0);
}
