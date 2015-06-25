/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author : Christian Potthast
 * Email  : potthast@usc.edu
 *
 */
#include "stdafx.h"
#include "kmeans.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::Kmeans::Kmeans (int num_points, int num_dimensions) 
  : num_points_ (num_points), num_dimensions_ (num_dimensions),
    points_to_clusters_(num_points_, 0)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::Kmeans::~Kmeans ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::Kmeans::initialClusterPoints()
{
  ClusterId i = 0;
  int dim;
  for (; i < num_clusters_; i++){
    Point point;   // each centroid is a point
    for (dim=0; dim<num_dimensions_; dim++) 
      point.push_back(0.0);
    SetPoints set_of_points;

    // init centroids
    centroids_.push_back(point);  

    // init clusterId -> set of points
    clusters_to_points_.push_back(set_of_points);
  }

  ClusterId cid;

  for (PointId pid = 0; pid < num_points_; pid++){
      
    cid = pid % num_clusters_;

    points_to_clusters_[pid] = cid;
    clusters_to_points_[cid].insert(pid);
  }    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::Kmeans::computeCentroids()
{    
  int i;
  ClusterId cid = 0;
  PointId num_points_in_cluster;
  // For each centroid
  BOOST_FOREACH(Centroids::value_type& centroid, centroids_)
  {
    num_points_in_cluster = 0;

    // For earch PointId in this set
    BOOST_FOREACH(SetPoints::value_type pid, clusters_to_points_[cid])
    {
      Point p = data_[pid];
      for (i=0; i<num_dimensions_; i++)
        centroid[i] += p[i];	
      num_points_in_cluster++;
    }
    // if no point in the clusters, this goes to inf (correct!)
    for (i=0; i<num_dimensions_; i++)
    {
      centroid[i] /= static_cast<float> (num_points_in_cluster);
    }
    
    cid++;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::Kmeans::kMeans ()
{
  bool not_converged = true;
  bool move;
  PointId pid;
  ClusterId cid, to_cluster;
  float d, min;
	num_iterations_ = 0;

  // Initial partition of points
  initialClusterPoints();

  // Until not converge
  while (not_converged){

    not_converged = false;

    computeCentroids();

    // for each point
    for (pid=0; pid<num_points_; pid++)
    {
      // distance from current cluster
      min = distance(centroids_[points_to_clusters_[pid]], data_[pid]);

      // foreach centroid
      cid = 0; 
      move = false;
      BOOST_FOREACH(Centroids::value_type c, centroids_)
      {
        d = distance(c, data_[pid]);
        if (d < min)
        {
          min = d;
          move = true;
          to_cluster = cid;

          // remove from current cluster
          clusters_to_points_[points_to_clusters_[pid]].erase(pid);

          not_converged = true;
        }
        cid++;
      }

      // move towards a closer centroid 
      if (move){  
        // insert
        points_to_clusters_[pid] = to_cluster;
        clusters_to_points_[to_cluster].insert(pid);
      }
    }
    num_iterations_++;
  } // end while
}
