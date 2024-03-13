#ifndef LASERPROCESSOR_HH
#define LASERPROCESSOR_HH

#include <unistd.h>
#include <math.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2/LinearMath/Vector3.h>

#include <list>
#include <set>
#include <vector>
#include <map>
#include <utility>
#include <algorithm>


/** @brief A namespace containing the laser processor helper classes */
namespace laser_processor {

/**
* @brief A struct representing a single sample (i.e., scan point) from the laser.
*/
class Sample {
public:
  int   index;
  float range;
  float intensity;
  float x;
  float y;

  /**
  * @brief Return pointer to sample of index <ind>
  */
  static Sample* Extract(int ind, const sensor_msgs::msg::LaserScan& scan);
};

/**
* @brief The comparator structure allowing the creation of an ordered set of Samples
*/
struct CompareSample {
  /**
  * @brief The comparator allowing the creation of an ordered set of Samples
  */  
  inline bool operator() (const Sample* a, const Sample* b) const
  {
    return (a->index <  b->index);
  }
};


/**
* @brief An ordered set of Samples
*
* Ordered based on sample index
*/
class SampleSet : public std::set<Sample*, CompareSample> {
public:
  /**
  * @brief Destructor
  */
  ~SampleSet() { clear(); }

  /**
  * @brief Delete all pointers to samples in the set
  */
  void clear();

  /**
  * @brief Get the centroid of the sample points
  * @return Centriod in (x,y,0) (z-element assumed 0)
  */
  tf2::Vector3 getPosition();
};


/**
* @brief A scan processor to split the scan into clusters
*/
class ScanProcessor
{
  std::list<SampleSet*> clusters_;
  sensor_msgs::msg::LaserScan scan_;

public:
  /**
  * @brief Get all the clusters in the scan
  * @return List of clusters
  */
  std::list<SampleSet*>& getClusters() { return clusters_; }

  /**
  * @brief Constructor
  * @param scan Scan to be processed
  */
  ScanProcessor(const sensor_msgs::msg::LaserScan& scan);

  /**
  * @brief Destructor
  */
  ~ScanProcessor();

  /**
  * @brief Remove and delete all references to scan clusters less than a minimum size
  * @param num Minimum number of points in cluster
  */
  void removeLessThan(uint32_t num);

  /**
  * @brief Split scan into clusters
  * @param thresh Euclidian distance threshold for clustering
  */
  void splitConnected(float thresh);
};
};

#endif
