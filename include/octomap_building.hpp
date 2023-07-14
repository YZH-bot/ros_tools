#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/math/Vector3.h>
#include <octomap_ros/conversions.h>
#include <octomap/OcTreeKey.h>

#include <tf/transform_datatypes.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>

typedef octomap::OcTree OcTreeT;
typedef octomath::Vector3 point3d;
typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
typedef pcl::PointXYZ PCLPoint;

class OctomapServer
{
public:
    OctomapServer()
        : m_octree(NULL),
          m_res(0.1),
          m_maxRange(50.0),
          m_minRange(1.0),
          m_compressMap(true),
          probHit(0.7),
          probMiss(0.4),
          thresMin(0.12),
          thresMax(0.97),
          m_occupancyMinZ(-std::numeric_limits<double>::max()),
          m_occupancyMaxZ(std::numeric_limits<double>::max())
    {
        // initialize octomap object & params
        m_octree = new OcTreeT(m_res);
        m_octree->setProbHit(probHit);
        m_octree->setProbMiss(probMiss);
        m_octree->setClampingThresMin(thresMin);
        m_octree->setClampingThresMax(thresMax);
        m_treeDepth = m_octree->getTreeDepth();
        m_maxTreeDepth = m_treeDepth;
    }

    /// Conversion from tf::Point to octomap::point3d
    static inline point3d pointTfToOctomap(const tf::Point &ptTf)
    {
        return point3d(ptTf.x(), ptTf.y(), ptTf.z());
    }

    inline static void updateMinKey(const octomap::OcTreeKey &in, octomap::OcTreeKey &min)
    {
        for (unsigned i = 0; i < 3; ++i)
            min[i] = std::min(in[i], min[i]);
    };

    inline static void updateMaxKey(const octomap::OcTreeKey &in, octomap::OcTreeKey &max)
    {
        for (unsigned i = 0; i < 3; ++i)
            max[i] = std::max(in[i], max[i]);
    };

    void insertScan(const tf::Point &sensorOriginTf, const PCLPointCloud &ground, const PCLPointCloud &nonground)
    {
        point3d sensorOrigin = pointTfToOctomap(sensorOriginTf);

        if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin) || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
        {
            ROS_ERROR_STREAM("Could not generate Key for origin " << sensorOrigin);
        }

        // instead of direct scan insertion, compute update to filter ground:
        octomap::KeySet free_cells, occupied_cells;
        // insert ground points only as free:                        // pc_nonground is empty without ground segmentation
        for (PCLPointCloud::const_iterator it = ground.begin(); it != ground.end(); ++it)
        {
            point3d point(it->x, it->y, it->z);

            if ((m_minRange > 0) && (point - sensorOrigin).norm() < m_minRange)
                continue;

            // maxrange check
            if ((m_maxRange > 0.0) && ((point - sensorOrigin).norm() > m_maxRange))
            {
                point = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
            }

            // only clear space (ground points)
            // computeRayKeys(const point3d & origin,const point3d & end,KeyRay & ray ) : 参数origin（光束起点）和
            // 参数end（传感器末端击中点）都是世界坐标系下的坐标！
            // 实际使用时，KeyRay用于保存单条光束在三维空间中raytracing的结果，KeySet收纳所有光束（也即点云数据）raytracing的结果。
            if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay))
            {
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
            }

            octomap::OcTreeKey endKey;
            if (m_octree->coordToKeyChecked(point, endKey))
            {
                updateMinKey(endKey, m_updateBBXMin);
                updateMaxKey(endKey, m_updateBBXMax);
            }
            else
            {
                ROS_ERROR_STREAM("Could not generate Key for endpoint " << point);
            }
        }

        // all other points: free on ray, occupied on endpoint:
        for (PCLPointCloud::const_iterator it = nonground.begin(); it != nonground.end(); ++it)
        {
            point3d point(it->x, it->y, it->z);

            if ((m_minRange > 0) && (point - sensorOrigin).norm() < m_minRange)
                continue;

            // maxrange check
            if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange))
            {

                // free cells
                if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay))
                {
                    free_cells.insert(m_keyRay.begin(), m_keyRay.end());
                }
                // occupied endpoint
                octomap::OcTreeKey key;
                if (m_octree->coordToKeyChecked(point, key))
                {
                    occupied_cells.insert(key);

                    updateMinKey(key, m_updateBBXMin);
                    updateMaxKey(key, m_updateBBXMax);
                }
            }
            else
            { // ray longer than maxrange:;
                point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
                if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay))
                {
                    free_cells.insert(m_keyRay.begin(), m_keyRay.end());

                    octomap::OcTreeKey endKey;
                    if (m_octree->coordToKeyChecked(new_end, endKey))
                    {
                        free_cells.insert(endKey);
                        updateMinKey(endKey, m_updateBBXMin);
                        updateMaxKey(endKey, m_updateBBXMax);
                    }
                    else
                    {
                        ROS_ERROR_STREAM("Could not generate Key for endpoint " << new_end);
                    }
                }
            }
        }

        // mark free cells only if not seen occupied in this cloud
        for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it)
        {
            if (occupied_cells.find(*it) == occupied_cells.end())
            {
                m_octree->updateNode(*it, false);
            }
        }

        // now mark all occupied cells:
        for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; it++)
        {
            m_octree->updateNode(*it, true);
        }

        // TODO: eval lazy+updateInner vs. proper insertion
        // non-lazy by default (updateInnerOccupancy() too slow for large maps)
        // m_octree->updateInnerOccupancy();
        octomap::point3d minPt, maxPt;
        ROS_DEBUG_STREAM("Bounding box keys (before): " << m_updateBBXMin[0] << " " << m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " << m_updateBBXMax[0] << " " << m_updateBBXMax[1] << " " << m_updateBBXMax[2]);

        // TODO: snap max / min keys to larger voxels by m_maxTreeDepth
        //   if (m_maxTreeDepth < 16)
        //   {
        //      OcTreeKey tmpMin = getIndexKey(m_updateBBXMin, m_maxTreeDepth); // this should give us the first key at depth m_maxTreeDepth that is smaller or equal to m_updateBBXMin (i.e. lower left in 2D grid coordinates)
        //      OcTreeKey tmpMax = getIndexKey(m_updateBBXMax, m_maxTreeDepth); // see above, now add something to find upper right
        //      tmpMax[0]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
        //      tmpMax[1]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
        //      tmpMax[2]+= m_octree->getNodeSize( m_maxTreeDepth ) - 1;
        //      m_updateBBXMin = tmpMin;
        //      m_updateBBXMax = tmpMax;
        //   }

        // TODO: we could also limit the bbx to be within the map bounds here (see publishing check)
        minPt = m_octree->keyToCoord(m_updateBBXMin);
        maxPt = m_octree->keyToCoord(m_updateBBXMax);
        ROS_DEBUG_STREAM("Updated area bounding box: " << minPt << " - " << maxPt);
        ROS_DEBUG_STREAM("Bounding box keys (after): " << m_updateBBXMin[0] << " " << m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " << m_updateBBXMax[0] << " " << m_updateBBXMax[1] << " " << m_updateBBXMax[2]);

        ROS_DEBUG_STREAM("m_octree size: " << m_octree->size());
        std::cout << "m_octree size: " << m_octree->size() << std::endl;
        if (m_compressMap)
            m_octree->prune();
    }

    void toPointcloud()
    {
        // now, traverse all leafs in the tree:
        for (OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth),
                               end = m_octree->end();
             it != end; ++it)
        {
            std::cout << it->getLogOdds() << " ";
            if (m_octree->isNodeOccupied(*it))
            {
                double z = it.getZ();
                double half_size = it.getSize() / 2.0;
                if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ)
                {
                    double size = it.getSize();
                    double x = it.getX();
                    double y = it.getY();

                    // insert into pointcloud:
                    pclCloud.push_back(PCLPoint(x, y, z));
                }
            }
        }
    }

protected:
    OcTreeT *m_octree;
    double m_res;
    octomap::KeyRay m_keyRay; // temp storage for ray casting
    octomap::OcTreeKey m_updateBBXMin;
    octomap::OcTreeKey m_updateBBXMax;
    double m_minRange;
    double m_maxRange;

    bool m_compressMap;
    unsigned m_treeDepth;
    unsigned m_maxTreeDepth;
    double m_occupancyMinZ;
    double m_occupancyMaxZ;

    double probHit, probMiss, thresMin, thresMax;

public:
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
};