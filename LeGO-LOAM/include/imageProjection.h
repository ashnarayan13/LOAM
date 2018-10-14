/** @file imageProjection.h
 * This class is for processing the incoming pointcloud
 * converting it to a range image and extract features
 * */

#include "utility.h"
#include <cv_bridge/cv_bridge.h>

class ImageProjection{
private:
	ros::NodeHandle nh;

    ros::Subscriber subLaserCloud;
    
    ros::Publisher pubFullCloud;
    ros::Publisher pubFullInfoCloud;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubSegmentedCloud;
    ros::Publisher pubSegmentedCloudPure;
    ros::Publisher pubSegmentedCloudInfo;
    ros::Publisher pubOutlierCloud;

    //debug
    ros::Publisher pubRangeImage;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;

    pcl::PointCloud<PointType>::Ptr fullCloud;
    pcl::PointCloud<PointType>::Ptr fullInfoCloud;

    pcl::PointCloud<PointType>::Ptr groundCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloudPure;
    pcl::PointCloud<PointType>::Ptr outlierCloud;

    PointType nanPoint;

    cv::Mat rangeMat;
    cv::Mat labelMat;
    cv::Mat groundMat;
    int labelCount;

    float startOrientation;
    float endOrientation;

    cloud_msgs::cloud_info segMsg;
    std_msgs::Header cloudHeader; /**<Store pointcloud header */

    std::vector<std::pair<uint8_t, uint8_t> > neighborIterator;

    uint16_t *allPushedIndX;
    uint16_t *allPushedIndY;

    uint16_t *queueIndX;
    uint16_t *queueIndY;

public:
    /**
     * Constructor that initializes node handle.
     * Subscribes to Pointcloud and advertises the
     * published pointclouds.
     *
     * It additionally allocates memory by calling allocateMemory().
     * Resets all parameters with resetParameters().
     */
	ImageProjection();

	/**
	 * First clears all the pointclouds.
	 * Then allocates memory to the pointclouds based on the N_SCAN
	 * and Horizon_SCAN found in utility.h.
	 * The memory is also allocated to the queue used for searching and the pointcloud segmentation
	 * variables.
	 */
	void allocateMemory();

	/**
	 * Clears data in pointcloud -> setting them to 0
	 * Sets the Mat files used for the range images as well.
	 */
	void resetParameters();

	~ImageProjection();

	/**
	 * Receives the Pointcloud message sent from the LiDAR
	 * @param laserCloudMsg message of type PointCloud2ConstPtr
	 * The message is converted from ROS type to a standard PCL pointcloud
	 * The header os the message is allocated to \link #cloudHeader. <br>
	 * Called by cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
	 */
	void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);

	/**
	 * Is the callback method when a new pointcloud is available.
	 * Calls <br>
	 * copyPointCloud(laserCloudMsg) <br>
	 * findStartEndAngle() <br>
	 * projectPointCloud() <br>
	 * groundRemoval() <br>
	 * cloudSegmentation() <br>
	 * publishCloud() <br>
	 * resetParameters() <br>
	 * @param laserCloudMsg message of type PointCloud2ConstPtr
	 */
	void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);

	/**
	 * Calculates the starting and ending orientation of the Pointcloud.
	 * This is required for /link #segMsg .
	 */
	void findStartEndAngle();


	void projectPointCloud();
	void groundRemoval();
	void cloudSegmentation();
	void labelComponents(int row, int col);
	void publishCloud();
};